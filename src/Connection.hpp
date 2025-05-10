#ifndef EXAMPLE_CONNECTION_HPP
#define EXAMPLE_CONNECTION_HPP

#include "LeapC.h"
#include <functional>
#include <thread>
#include <mutex>
#include <atomic>

class leap_device_wrapper
{
public:
  leap_device_wrapper(LEAP_DEVICE device) : device_(device) {}
  ~leap_device_wrapper() { }
  LEAP_DEVICE get() const { return device_; }
  LEAP_DEVICE device_;
};

class Connection
{
public:
  Connection();
  ~Connection();

  bool open();
  void close();
  void destroy();

  LEAP_TRACKING_EVENT *getFrame();
  LEAP_DEVICE_INFO *getDeviceProperties();

  static const char *resultString(eLeapRS r);
  static void millisleep(int milliseconds);

  // Callback setters
  void setOnConnect(std::function<void()> cb);
  void setOnDisconnect(std::function<void()> cb);
  void setOnDeviceFound(std::function<void(const LEAP_DEVICE_INFO *)> cb);
  void setOnDeviceLost(std::function<void()> cb);
  void setOnDeviceFailure(std::function<void(eLeapDeviceStatus)> cb);
  void setOnPolicy(std::function<void(uint32_t)> cb);
  void setOnFrame(std::function<void(const LEAP_TRACKING_EVENT *)> cb);
  void setOnImage(std::function<void(const LEAP_IMAGE_EVENT *)> cb);
  void setOnIMU(std::function<void(const LEAP_IMU_EVENT *)> cb);
  void setOnTrackingMode(std::function<void(const LEAP_TRACKING_MODE_EVENT *)> cb);

  bool isConnected() const;
  LEAP_CONNECTION connectionHandle_ = nullptr;

private:
  void messageLoop();
  void handleEvent(const LEAP_CONNECTION_MESSAGE &msg);
  void cacheFrame(const LEAP_TRACKING_EVENT *frame);
  void cacheDevice(const LEAP_DEVICE_INFO *deviceProps);

  std::thread messageThread_;
  std::mutex dataMutex_;
  std::atomic<bool> running_;
  std::atomic<bool> connected_;

  LEAP_TRACKING_EVENT *lastFrame_ = nullptr;
  LEAP_DEVICE_INFO *lastDevice_ = nullptr;

  // Callbacks
  std::function<void()> onConnect_;
  std::function<void()> onDisconnect_;
  std::function<void(const LEAP_DEVICE_INFO *)> onDeviceFound_;
  std::function<void()> onDeviceLost_;
  std::function<void(eLeapDeviceStatus)> onDeviceFailure_;
  std::function<void(uint32_t)> onPolicy_;
  std::function<void(const LEAP_TRACKING_EVENT *)> onFrame_;
  std::function<void(const LEAP_IMAGE_EVENT *)> onImage_;
  std::function<void(const LEAP_IMU_EVENT *)> onIMU_;
  std::function<void(const LEAP_TRACKING_MODE_EVENT *)> onTrackingMode_;
};

#endif // EXAMPLE_CONNECTION_HPP
