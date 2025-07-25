#include "Connection.hpp"
#include <cstring>
#include <iostream>
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif
#include <thread>

static void *allocate(uint32_t size, eLeapAllocatorType typeHint, void *state)
{
  void *ptr = malloc(size);
  return ptr;
}

static void deallocate(void *ptr, void *state)
{
  if (!ptr)
    return;
  free(ptr);
}

Connection::Connection() : running_(false), connected_(false) {}

Connection::~Connection()
{
  destroy();
}

bool Connection::open()
{
  if (running_)
    return true;

  if (!connectionHandle_)
  {
    if (LeapCreateConnection(nullptr, &connectionHandle_) != eLeapRS_Success)
      return false;
  }

  if (LeapOpenConnection(connectionHandle_) != eLeapRS_Success)
    return false;

  running_ = true;
  messageThread_ = std::thread(&Connection::messageLoop, this);

  {
    LEAP_ALLOCATOR allocator = {allocate, deallocate, nullptr};
    LeapSetAllocator(connectionHandle_, &allocator);
  }
  LeapSetPolicyFlags(connectionHandle_, eLeapPolicyFlag_MapPoints, 0);

  return true;
}

void Connection::close()
{
  if (!running_)
    return;

  running_ = false;
  if (messageThread_.joinable())
  {
    debugMessage("Joining message thread...");
    messageThread_.join();
    debugMessage("Message thread joined.");
  }

  if (connectionHandle_)
  {
    LeapCloseConnection(connectionHandle_);
  }
  messageThread_ = std::thread(); // Reset the thread object after joining
}

void Connection::destroy()
{
  // スレッドとコネクションを安全にクローズ
  close();

  // コネクションハンドルの破棄
  if (connectionHandle_)
  {
    LeapDestroyConnection(connectionHandle_);
    connectionHandle_ = nullptr;
  }

  // メモリの解放
  if (lastFrame_)
  {
    if (lastFrame_->pHands)
    {
      free(lastFrame_->pHands);
      lastFrame_->pHands = nullptr;
    }
    free(lastFrame_);
    lastFrame_ = nullptr;
  }

  if (lastDevice_)
  {
    if (lastDevice_->serial)
    {
      free(lastDevice_->serial);
      lastDevice_->serial = nullptr;
    }
    free(lastDevice_);
    lastDevice_ = nullptr;
  }

  // 接続状態をリセット
  connected_ = false;
}
void Connection::cacheFrame(const LEAP_TRACKING_EVENT *frame)
{
  std::lock_guard<std::mutex> lock(dataMutex_);
  if (!lastFrame_)
  {
    lastFrame_ = (LEAP_TRACKING_EVENT *)malloc(sizeof(LEAP_TRACKING_EVENT));
    lastFrame_->pHands = (LEAP_HAND *)malloc(2 * sizeof(LEAP_HAND));
  }

  if (frame)
  {
    memcpy(&lastFrame_->info, &frame->info, sizeof(LEAP_FRAME_HEADER));
    lastFrame_->tracking_frame_id = frame->tracking_frame_id;
    lastFrame_->nHands = frame->nHands;
    lastFrame_->framerate = frame->framerate;
    memcpy(lastFrame_->pHands, frame->pHands, frame->nHands * sizeof(LEAP_HAND));
  }
}

LEAP_TRACKING_EVENT *Connection::getFrame()
{
  std::lock_guard<std::mutex> lock(dataMutex_);
  if (!lastFrame_)
    return nullptr;

  LEAP_TRACKING_EVENT *copy = (LEAP_TRACKING_EVENT *)malloc(sizeof(LEAP_TRACKING_EVENT));
  copy->pHands = (LEAP_HAND *)malloc(2 * sizeof(LEAP_HAND));
  memcpy(&copy->info, &lastFrame_->info, sizeof(LEAP_FRAME_HEADER));
  copy->tracking_frame_id = lastFrame_->tracking_frame_id;
  copy->nHands = lastFrame_->nHands;
  copy->framerate = lastFrame_->framerate;
  memcpy(copy->pHands, lastFrame_->pHands, lastFrame_->nHands * sizeof(LEAP_HAND));
  return copy;
}

void Connection::cacheDevice(const LEAP_DEVICE_INFO *deviceProps)
{
  std::lock_guard<std::mutex> lock(dataMutex_);
  if (!lastDevice_)
    lastDevice_ = (LEAP_DEVICE_INFO *)malloc(sizeof(LEAP_DEVICE_INFO));
  else
    free(lastDevice_->serial);

  *lastDevice_ = *deviceProps;
  lastDevice_->serial = (char *)malloc(deviceProps->serial_length);
  memcpy(lastDevice_->serial, deviceProps->serial, deviceProps->serial_length);
}

LEAP_DEVICE_INFO *Connection::getDeviceProperties()
{
  std::lock_guard<std::mutex> lock(dataMutex_);
  return lastDevice_;
}

void Connection::messageLoop()
{
  LEAP_CONNECTION_MESSAGE msg;
  while (running_)
  {
    // タイムアウトを短縮（1秒から100ms）してより応答性を向上
    eLeapRS result = LeapPollConnection(connectionHandle_, 100, &msg);

    if (!running_) // 再度チェック
      break;

    if (result == eLeapRS_Success)
    {
      handleEvent(msg);
    }
    else if (result != eLeapRS_Timeout)
    {
      // タイムアウト以外のエラーの場合はログ出力
      debugMessage((std::string("LeapPollConnection error: ") + resultString(result)).c_str());
    }
  }
  debugMessage("Message loop exiting...");
}

void Connection::handleEvent(const LEAP_CONNECTION_MESSAGE &msg)
{
  switch (msg.type)
  {
  case eLeapEventType_Connection:
    connected_ = true;
    if (onConnect_)
      onConnect_();
    break;
  case eLeapEventType_ConnectionLost:
    connected_ = false;
    if (onDisconnect_)
      onDisconnect_();
    break;
  case eLeapEventType_Device:
    if (onDeviceFound_)
    {
      LEAP_DEVICE_INFO props = {sizeof(LEAP_DEVICE_INFO)};
      props.serial_length = 1;
      props.serial = (char *)malloc(1);
      LEAP_DEVICE device;
      LeapOpenDevice(msg.device_event->device, &device);
      if (LeapGetDeviceInfo(device, &props) == eLeapRS_InsufficientBuffer)
      {
        props.serial = (char *)realloc(props.serial, props.serial_length);
        LeapGetDeviceInfo(device, &props);
      }
      cacheDevice(&props);
      onDeviceFound_(&props);
      free(props.serial);
      LeapCloseDevice(device);
    }
    break;
  case eLeapEventType_DeviceLost:
    if (onDeviceLost_)
      onDeviceLost_();
    break;
  case eLeapEventType_DeviceFailure:
    if (onDeviceFailure_)
    {
      onDeviceFailure_(msg.device_failure_event->status);
    }
    break;
  case eLeapEventType_Tracking:
    cacheFrame(msg.tracking_event);
    if (onFrame_)
      onFrame_(msg.tracking_event);
    break;
  case eLeapEventType_Policy:
    if (onPolicy_)
      onPolicy_(msg.policy_event->current_policy);
    break;
  case eLeapEventType_Image:
    if (onImage_)
      onImage_(msg.image_event);
    break;
  case eLeapEventType_IMU:
    if (onIMU_)
      onIMU_(msg.imu_event);
    break;
  case eLeapEventType_TrackingMode:
    if (onTrackingMode_)
      onTrackingMode_(msg.tracking_mode_event);
    break;
  default:
    break;
  }
}

bool Connection::isConnected() const
{
  return connected_;
}

const char *Connection::resultString(eLeapRS r)
{
  switch (r)
  {
  case eLeapRS_Success:
    return "eLeapRS_Success";
  case eLeapRS_UnknownError:
    return "eLeapRS_UnknownError";
  case eLeapRS_InvalidArgument:
    return "eLeapRS_InvalidArgument";
  case eLeapRS_InsufficientResources:
    return "eLeapRS_InsufficientResources";
  case eLeapRS_InsufficientBuffer:
    return "eLeapRS_InsufficientBuffer";
  case eLeapRS_Timeout:
    return "eLeapRS_Timeout";
  case eLeapRS_NotConnected:
    return "eLeapRS_NotConnected";
  case eLeapRS_UnexpectedClosed:
    return "eLeapRS_UnexpectedClosed";
  default:
    return "Unknown";
  }
}

void Connection::millisleep(int ms)
{
#ifdef _WIN32
  Sleep(ms);
#else
  usleep(ms * 1000);
#endif
}

// Callback setters
void Connection::setOnConnect(std::function<void()> cb) { onConnect_ = cb; }
void Connection::setOnDisconnect(std::function<void()> cb) { onDisconnect_ = cb; }
void Connection::setOnDeviceFound(std::function<void(const LEAP_DEVICE_INFO *)> cb) { onDeviceFound_ = cb; }
void Connection::setOnDeviceLost(std::function<void()> cb) { onDeviceLost_ = cb; }
void Connection::setOnDeviceFailure(std::function<void(eLeapDeviceStatus)> cb) { onDeviceFailure_ = cb; }
void Connection::setOnPolicy(std::function<void(uint32_t)> cb) { onPolicy_ = cb; }
void Connection::setOnFrame(std::function<void(const LEAP_TRACKING_EVENT *)> cb) { onFrame_ = cb; }
void Connection::setOnImage(std::function<void(const LEAP_IMAGE_EVENT *)> cb) { onImage_ = cb; }
void Connection::setOnIMU(std::function<void(const LEAP_IMU_EVENT *)> cb) { onIMU_ = cb; }
void Connection::setOnTrackingMode(std::function<void(const LEAP_TRACKING_MODE_EVENT *)> cb) { onTrackingMode_ = cb; }
