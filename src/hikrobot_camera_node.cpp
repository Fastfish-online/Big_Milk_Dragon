#include <arpa/inet.h>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "MvCameraControl.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr int kMvOk = 0;

std::string sdk_error(int code)
{
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "0x%08X", static_cast<unsigned int>(code));
  return buffer;
}

std::string serial_of(const MV_CC_DEVICE_INFO & info)
{
  if (info.nTLayerType == MV_GIGE_DEVICE) {
    return reinterpret_cast<const char *>(info.SpecialInfo.stGigEInfo.chSerialNumber);
  }
  if (info.nTLayerType == MV_USB_DEVICE) {
    return reinterpret_cast<const char *>(info.SpecialInfo.stUsb3VInfo.chSerialNumber);
  }
  return "";
}

std::string ip_of(const MV_CC_DEVICE_INFO & info)
{
  if (info.nTLayerType != MV_GIGE_DEVICE) {
    return "";
  }
  const auto address = info.SpecialInfo.stGigEInfo.nCurrentIp;
  return std::to_string((address >> 24) & 0xff) + "." +
         std::to_string((address >> 16) & 0xff) + "." +
         std::to_string((address >> 8) & 0xff) + "." +
         std::to_string(address & 0xff);
}

std::optional<unsigned int> pixel_type(const std::string & value)
{
  if (value == "BGR8") return PixelType_Gvsp_BGR8_Packed;
  if (value == "RGB8") return PixelType_Gvsp_RGB8_Packed;
  if (value == "Mono8") return PixelType_Gvsp_Mono8;
  return std::nullopt;
}

std::string encoding_for(const std::string & value)
{
  if (value == "BGR8") return "bgr8";
  if (value == "RGB8") return "rgb8";
  return "mono8";
}
}  // namespace

class HikrobotCameraNode : public rclcpp::Node
{
public:
  HikrobotCameraNode() : Node("hikrobot_camera")
  {
    serial_number_ = declare_parameter<std::string>("serial_number", "");
    ip_address_ = declare_parameter<std::string>("ip_address", "");
    image_topic_ = declare_parameter<std::string>("image_topic", "/image_raw");
    frame_id_ = declare_parameter<std::string>("frame_id", "camera_optical_frame");
    exposure_time_ = declare_parameter<double>("exposure_time", 5000.0);
    gain_ = declare_parameter<double>("gain", 0.0);
    frame_rate_ = declare_parameter<double>("frame_rate", 30.0);
    pixel_format_ = declare_parameter<std::string>("pixel_format", "BGR8");
    reconnect_interval_ms_ = declare_parameter<int>("reconnect_interval_ms", 2000);
    grab_timeout_ms_ = declare_parameter<int>("grab_timeout_ms", 1000);

    if (!pixel_type(pixel_format_)) {
      throw std::runtime_error("pixel_format must be BGR8, RGB8, or Mono8");
    }
    image_publisher_ = create_publisher<sensor_msgs::msg::Image>(image_topic_, rclcpp::SensorDataQoS());
    parameter_callback_ = add_on_set_parameters_callback(
      std::bind(&HikrobotCameraNode::on_parameters, this, std::placeholders::_1));
    grab_timer_ = create_wall_timer(1ms, std::bind(&HikrobotCameraNode::grab_frame, this));
    reconnect_timer_ = create_wall_timer(
      std::chrono::milliseconds(reconnect_interval_ms_), std::bind(&HikrobotCameraNode::ensure_connected, this));
    ensure_connected();
  }

  ~HikrobotCameraNode() override
  {
    std::lock_guard<std::mutex> lock(camera_mutex_);
    disconnect_locked();
  }

private:
  rcl_interfaces::msg::SetParametersResult on_parameters(const std::vector<rclcpp::Parameter> & parameters)
  {
    std::lock_guard<std::mutex> lock(camera_mutex_);
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;
    std::string requested_pixel = pixel_format_;
    double requested_exposure = exposure_time_, requested_gain = gain_, requested_rate = frame_rate_;
    for (const auto & parameter : parameters) {
      if (parameter.get_name() == "image_topic" || parameter.get_name() == "serial_number" ||
          parameter.get_name() == "ip_address" || parameter.get_name() == "reconnect_interval_ms" ||
          parameter.get_name() == "grab_timeout_ms") {
        result.reason = parameter.get_name() + " is startup-only";
        return result;
      }
      if (parameter.get_name() == "frame_id") frame_id_ = parameter.as_string();
      if (parameter.get_name() == "exposure_time") requested_exposure = parameter.as_double();
      if (parameter.get_name() == "gain") requested_gain = parameter.as_double();
      if (parameter.get_name() == "frame_rate") requested_rate = parameter.as_double();
      if (parameter.get_name() == "pixel_format") requested_pixel = parameter.as_string();
    }
    if (!pixel_type(requested_pixel) || requested_exposure <= 0.0 || requested_gain < 0.0 || requested_rate <= 0.0) {
      result.reason = "invalid exposure_time, gain, frame_rate, or pixel_format";
      return result;
    }
    if (connected_ && !apply_settings_locked(requested_exposure, requested_gain, requested_rate, requested_pixel)) {
      disconnect_locked();
      result.reason = "camera rejected parameter update; see node log for SDK error or range";
      return result;
    }
    exposure_time_ = requested_exposure;
    gain_ = requested_gain;
    frame_rate_ = requested_rate;
    pixel_format_ = requested_pixel;
    result.successful = true;
    return result;
  }

  bool select_device(MV_CC_DEVICE_INFO & selected)
  {
    MV_CC_DEVICE_INFO_LIST devices{};
    int code = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &devices);
    if (code != kMvOk) {
      RCLCPP_ERROR(get_logger(), "MV_CC_EnumDevices failed: %s", sdk_error(code).c_str());
      return false;
    }
    for (unsigned int i = 0; i < devices.nDeviceNum; ++i) {
      const auto & candidate = *devices.pDeviceInfo[i];
      const bool serial_matches = serial_number_.empty() || serial_of(candidate) == serial_number_;
      const bool ip_matches = ip_address_.empty() || ip_of(candidate) == ip_address_;
      if (serial_matches && ip_matches) {
        std::memcpy(&selected, &candidate, sizeof(MV_CC_DEVICE_INFO));
        return true;
      }
    }
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "No matching camera found (serial='%s', ip='%s')", serial_number_.c_str(), ip_address_.c_str());
    return false;
  }

  void ensure_connected()
  {
    std::lock_guard<std::mutex> lock(camera_mutex_);
    if (connected_) return;
    MV_CC_DEVICE_INFO device{};
    if (!select_device(device)) return;
    int code = MV_CC_CreateHandle(&handle_, &device);
   // 使用序列号强制创建一个指定设备的句柄
    if (!serial_number_.empty()) {
     MV_CC_DEVICE_INFO_LIST stDeviceList;
     memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
     int nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &stDeviceList); // 枚举USB设备
     if (nRet == MV_OK && stDeviceList.nDeviceNum > 0) {
        for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++) {
            MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
            if (pDeviceInfo->nTLayerType == MV_USB_DEVICE) {
                // 检查序列号是否匹配
                if (strcmp((char*)pDeviceInfo->SpecialInfo.stUsb3VInfo.chSerialNumber, serial_number_.c_str()) == 0) {
                    MV_CC_CreateHandle(&handle_, pDeviceInfo); // 用这个特定设备创建句柄
                    break;
                }
            }
        }
    }
}
    if (code == kMvOk) code = MV_CC_OpenDevice(handle_);
    if (code != kMvOk) {
      RCLCPP_ERROR(get_logger(), "Could not open camera: %s", sdk_error(code).c_str());
      disconnect_locked();
      return;
    }
    connected_ = true;
    if (!apply_settings_locked(exposure_time_, gain_, frame_rate_, pixel_format_)) {
      disconnect_locked();
      return;
    }
    code = MV_CC_StartGrabbing(handle_);
    if (code != kMvOk) {
      RCLCPP_ERROR(get_logger(), "MV_CC_StartGrabbing failed: %s", sdk_error(code).c_str());
      disconnect_locked();
      return;
    }
    grabbing_ = true;
    RCLCPP_INFO(get_logger(), "Connected to HIKROBOT camera serial=%s ip=%s", serial_of(device).c_str(), ip_of(device).c_str());
  }

  bool set_float_checked_locked(const char * name, double value)
  {
    MVCC_FLOATVALUE range{};
    int code = MV_CC_GetFloatValue(handle_, name, &range);
    if (code != kMvOk || value < range.fMin || value > range.fMax) {
      RCLCPP_ERROR(get_logger(), "%s=%f is unsupported (range query/result %s, range %f..%f)",
        name, value, sdk_error(code).c_str(), range.fMin, range.fMax);
      return false;
    }
    code = MV_CC_SetFloatValue(handle_, name, static_cast<float>(value));
    if (code != kMvOk) RCLCPP_ERROR(get_logger(), "Setting %s failed: %s", name, sdk_error(code).c_str());
    return code == kMvOk;
  }

  bool apply_settings_locked(double exposure, double gain, double rate, const std::string & pixel)
  {
    const bool restart = grabbing_ && pixel != pixel_format_;
    int code = kMvOk;
    if (restart) {
      code = MV_CC_StopGrabbing(handle_);
      if (code != kMvOk) return false;
      grabbing_ = false;
    }
    const auto type = pixel_type(pixel);
    code = MV_CC_SetEnumValue(handle_, "PixelFormat", *type);
    if (code == kMvOk) code = MV_CC_SetEnumValue(handle_, "ExposureAuto", 0);
    if (code == kMvOk && !set_float_checked_locked("ExposureTime", exposure)) return false;
    if (code == kMvOk) code = MV_CC_SetEnumValue(handle_, "GainAuto", 0);
    if (code == kMvOk && !set_float_checked_locked("Gain", gain)) return false;
    if (code == kMvOk) code = MV_CC_SetBoolValue(handle_, "AcquisitionFrameRateEnable", true);
    if (code == kMvOk && !set_float_checked_locked("AcquisitionFrameRate", rate)) return false;
    if (code != kMvOk) {
      RCLCPP_ERROR(get_logger(), "Applying camera settings failed: %s", sdk_error(code).c_str());
      return false;
    }
    if (restart) {
      code = MV_CC_StartGrabbing(handle_);
      grabbing_ = code == kMvOk;
      if (code != kMvOk) RCLCPP_ERROR(get_logger(), "Could not resume acquisition: %s", sdk_error(code).c_str());
    }
    return code == kMvOk;
  }

  void grab_frame()
  {
    std::lock_guard<std::mutex> lock(camera_mutex_);
    if (!connected_ || !grabbing_) return;
    MV_FRAME_OUT frame{};
    const int code = MV_CC_GetImageBuffer(handle_, &frame, grab_timeout_ms_);
    if (code != kMvOk) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Image grab failed: %s; reconnecting", sdk_error(code).c_str());
      disconnect_locked();
      return;
    }
    const auto desired = pixel_type(pixel_format_);
    std::vector<uint8_t> buffer(frame.stFrameInfo.nWidth * frame.stFrameInfo.nHeight * (pixel_format_ == "Mono8" ? 1U : 3U));
    MV_CC_PIXEL_CONVERT_PARAM convert{};
    convert.nWidth = frame.stFrameInfo.nWidth;
    convert.nHeight = frame.stFrameInfo.nHeight;
    convert.pSrcData = frame.pBufAddr;
    convert.nSrcDataLen = frame.stFrameInfo.nFrameLen;
    convert.enSrcPixelType = frame.stFrameInfo.enPixelType;
    convert.enDstPixelType = static_cast<MvGvspPixelType>(*desired);
    convert.pDstBuffer = buffer.data();
    convert.nDstBufferSize = static_cast<unsigned int>(buffer.size());
    const int convert_code = MV_CC_ConvertPixelType(handle_, &convert);
    MV_CC_FreeImageBuffer(handle_, &frame);
    if (convert_code != kMvOk) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Pixel conversion failed: %s", sdk_error(convert_code).c_str());
      return;
    }
    sensor_msgs::msg::Image image;
    image.header.stamp = now();
    image.header.frame_id = frame_id_;
    image.height = frame.stFrameInfo.nHeight;
    image.width = frame.stFrameInfo.nWidth;
    image.encoding = encoding_for(pixel_format_);
    image.is_bigendian = false;
    image.step = image.width * (pixel_format_ == "Mono8" ? 1U : 3U);
    image.data = std::move(buffer);
    image_publisher_->publish(image);
  }

  void disconnect_locked()
  {
    if (!handle_) return;
    if (grabbing_) MV_CC_StopGrabbing(handle_);
    MV_CC_CloseDevice(handle_);
    MV_CC_DestroyHandle(handle_);
    handle_ = nullptr;
    connected_ = false;
    grabbing_ = false;
  }

  std::mutex camera_mutex_;
  void * handle_{nullptr};
  bool connected_{false};
  bool grabbing_{false};
  std::string serial_number_, ip_address_, image_topic_, frame_id_, pixel_format_;
  double exposure_time_, gain_, frame_rate_;
  int reconnect_interval_ms_, grab_timeout_ms_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  rclcpp::TimerBase::SharedPtr grab_timer_, reconnect_timer_;
  OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HikrobotCameraNode>());
  rclcpp::shutdown();
  return 0;
}
