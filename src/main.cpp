#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include <pybind11/complex.h>
#include <pybind11/chrono.h>
#include <pybind11/numpy.h>

#include "Connection.hpp"
#include "LeapC.h"

namespace py = pybind11;

int add(int a, int b)
{
    return a + b;
}

PYBIND11_MODULE(leapmotion_conn, m)
{
    m.doc() = "Leap Motion C++ API wrapper"; // Optional module docstring

    m.def("add", &add, "A function that adds two numbers",
          py::arg("a"), py::arg("b"));
    py::class_<LEAP_VECTOR>(m, "Vector")
        .def_readonly("x", &LEAP_VECTOR::x)
        .def_readonly("y", &LEAP_VECTOR::y)
        .def_readonly("z", &LEAP_VECTOR::z)
        .def("__getitem__", [](const LEAP_VECTOR &v, size_t i)
             {
            if (i == 0) return v.x;
            else if (i == 1) return v.y;
            else if (i == 2) return v.z;
            throw py::index_error("Index out of range for LEAP_VECTOR"); })
        .def_property_readonly("array", [](const LEAP_VECTOR &v)
             { 
                py::array_t<float> arr(3);
                auto buf = arr.mutable_data();
                buf[0] = v.x;
                buf[1] = v.y;
                buf[2] = v.z;
                return arr; });
    
    py::enum_<eLeapDevicePID>(m, "DevicePID")
        .value("Unknown", eLeapDevicePID_Unknown)
        .value("Peripheral", eLeapDevicePID_Peripheral)
        .value("Dragonfly", eLeapDevicePID_Dragonfly)
        .value("Nightcrawler", eLeapDevicePID_Nightcrawler)
        .value("Rigel", eLeapDevicePID_Rigel)
        .value("SIR170", eLeapDevicePID_SIR170)
        .value("3Di", eLeapDevicePID_3Di)
        .value("LMC2", eLeapDevicePID_LMC2)
        .value("Invalid", eLeapDevicePID_Invalid)
        .export_values();

    py::enum_<eLeapRS>(m, "Result")
        .value("Success", eLeapRS_Success)
        .value("UnknownError", eLeapRS_UnknownError)
        .value("InvalidArgument", eLeapRS_InvalidArgument)
        .value("InsufficientResources", eLeapRS_InsufficientResources)
        .value("InsufficientBuffer", eLeapRS_InsufficientBuffer)
        .value("Timeout", eLeapRS_Timeout)
        .value("NotConnected", eLeapRS_NotConnected)
        .value("HandshakeIncomplete", eLeapRS_HandshakeIncomplete)
        .value("BufferSizeOverflow", eLeapRS_BufferSizeOverflow)
        .value("ProtocolError", eLeapRS_ProtocolError)
        .value("InvalidClientID", eLeapRS_InvalidClientID)
        .value("UnexpectedClosed", eLeapRS_UnexpectedClosed)
        .value("UnknownImageFrameRequest", eLeapRS_UnknownImageFrameRequest)
        .value("UnknownTrackingFrameID", eLeapRS_UnknownTrackingFrameID)
        .value("RoutineIsNotSeer", eLeapRS_RoutineIsNotSeer)
        .value("TimestampTooEarly", eLeapRS_TimestampTooEarly)
        .value("ConcurrentPoll", eLeapRS_ConcurrentPoll)
        .value("NotStreaming", eLeapRS_NotStreaming)
        .value("CannotOpenDevice", eLeapRS_CannotOpenDevice)
        .value("Unsupported", eLeapRS_Unsupported)
        .export_values();

    py::enum_<eLeapTrackingMode>(m, "TrackingMode")
        .value("Desktop", eLeapTrackingMode_Desktop)
        .value("HMD", eLeapTrackingMode_HMD)
        .value("ScreenTop", eLeapTrackingMode_ScreenTop)
        .value("OptimizedSustained", eLeapTrackingMode_Unknown)
        .export_values();

    py::class_<LEAP_DEVICE_INFO>(m, "DeviceInfo")
        .def_readonly("size", &LEAP_DEVICE_INFO::size)
        .def_readonly("status", &LEAP_DEVICE_INFO::status)
        .def_readonly("caps", &LEAP_DEVICE_INFO::caps)
        .def_readonly("pid", &LEAP_DEVICE_INFO::pid)
        .def_readonly("baseline", &LEAP_DEVICE_INFO::baseline)
        .def_readonly("serial_length", &LEAP_DEVICE_INFO::serial_length)
        .def_readonly("serial", &LEAP_DEVICE_INFO::serial)
        .def_readonly("h_fov", &LEAP_DEVICE_INFO::h_fov)
        .def_readonly("v_fov", &LEAP_DEVICE_INFO::v_fov)
        .def_readonly("range", &LEAP_DEVICE_INFO::range);
    
    py::enum_<eLeapDeviceStatus>(m, "DeviceStatus")
        .value("Streaming", eLeapDeviceStatus_Streaming)
        .value("Paused", eLeapDeviceStatus_Paused)
        .value("Robust", eLeapDeviceStatus_Robust)
        .value("Smudged", eLeapDeviceStatus_Smudged)
        .value("LowResource", eLeapDeviceStatus_LowResource)
        .value("UnknownFailure", eLeapDeviceStatus_UnknownFailure)
        .value("BadCalibration", eLeapDeviceStatus_BadCalibration)
        .value("BadFirmware", eLeapDeviceStatus_BadFirmware)
        .value("BadTransport", eLeapDeviceStatus_BadTransport)
        .value("BadControl", eLeapDeviceStatus_BadControl)
        .export_values();

    py::class_<LEAP_IMU_EVENT>(m, "IMUEvent")
        .def_readonly("timestamp", &LEAP_IMU_EVENT::timestamp)
        .def_readonly("timestamp_hw", &LEAP_IMU_EVENT::timestamp_hw)
        .def_readonly("flags", &LEAP_IMU_EVENT::flags)
        .def_readonly("accelerometer", &LEAP_IMU_EVENT::accelerometer)
        .def_readonly("gyroscope", &LEAP_IMU_EVENT::gyroscope)
        .def_readonly("temperature", &LEAP_IMU_EVENT::temperature);
    
    py::class_<LEAP_IMAGE_EVENT>(m, "ImageEvent");
    
    py::class_<_LEAP_TRACKING_MODE_EVENT>(m, "TrackingModeEvent")
        .def_readonly("reserved", &_LEAP_TRACKING_MODE_EVENT::reserved)
        .def_readonly("current_tracking_mode", &_LEAP_TRACKING_MODE_EVENT::current_tracking_mode);
    
    py::class_<LEAP_FRAME_HEADER>(m, "FrameHeader")
        .def_readonly("frame_id", &LEAP_FRAME_HEADER::frame_id)
        .def_readonly("timestamp", &LEAP_FRAME_HEADER::timestamp);

    py::enum_<eLeapHandType>(m, "HandType")
        .value("Left", eLeapHandType_Left)
        .value("Right", eLeapHandType_Right)
        .export_values();

    py::class_<LEAP_QUATERNION>(m, "Quaternion")
        .def_readonly("x", &LEAP_QUATERNION::x)
        .def_readonly("y", &LEAP_QUATERNION::y)
        .def_readonly("z", &LEAP_QUATERNION::z)
        .def_readonly("w", &LEAP_QUATERNION::w)
        .def("__getitem__", [](const LEAP_QUATERNION &q, size_t i)
             {
            if (i == 0) return q.x;
            else if (i == 1) return q.y;
            else if (i == 2) return q.z;
            else if (i == 3) return q.w;
            throw py::index_error("Index out of range for LEAP_QUATERNION"); })
        .def_property_readonly("array", [](const LEAP_QUATERNION &q)
                { 
                    py::array_t<float> arr(4);
                    auto buf = arr.mutable_data();
                    buf[0] = q.x;
                    buf[1] = q.y;
                    buf[2] = q.z;
                    buf[3] = q.w;
                    return arr; });

    py::class_<LEAP_PALM>(m, "Palm")
        .def_readonly("position", &LEAP_PALM::position)
        .def_readonly("velocity", &LEAP_PALM::velocity)
        .def_readonly("normal", &LEAP_PALM::normal)
        .def_readonly("width", &LEAP_PALM::width)
        .def_readonly("direction", &LEAP_PALM::direction)
        .def_readonly("orientation", &LEAP_PALM::orientation)
        .def_readonly("stabilized_position", &LEAP_PALM::stabilized_position);

    py::class_<LEAP_BONE>(m, "Bone")
        .def_readonly("prev_joint", &LEAP_BONE::prev_joint)
        .def_readonly("next_joint", &LEAP_BONE::next_joint)
        .def_readonly("width", &LEAP_BONE::width)
        .def_readonly("rotation", &LEAP_BONE::rotation);

    py::class_<LEAP_DIGIT>(m, "Digit")
        .def_readonly("finger_id", &LEAP_DIGIT::finger_id)
        .def_readonly("is_extended", &LEAP_DIGIT::is_extended)
        .def_readonly("metacarpal", &LEAP_DIGIT::metacarpal)
        .def_readonly("proximal", &LEAP_DIGIT::proximal)
        .def_readonly("intermediate", &LEAP_DIGIT::intermediate)
        .def_readonly("distal", &LEAP_DIGIT::distal)
        .def_property_readonly("bones", [](const LEAP_DIGIT &digit)
                               {
            std::vector<LEAP_BONE> bones(4);
            std::copy(digit.bones, digit.bones + 4, bones.begin());
            return bones; });

    py::class_<LEAP_HAND>(m, "Hand")
        .def_readonly("id", &LEAP_HAND::id)
        .def_readonly("flags", &LEAP_HAND::flags)
        .def_readonly("type", &LEAP_HAND::type)
        .def_readonly("confidence", &LEAP_HAND::confidence)
        .def_readonly("visible_time", &LEAP_HAND::visible_time)
        .def_readonly("pinch_distance", &LEAP_HAND::pinch_distance)
        .def_readonly("grab_angle", &LEAP_HAND::grab_angle)
        .def_readonly("pinch_strength", &LEAP_HAND::pinch_strength)
        .def_readonly("grab_strength", &LEAP_HAND::grab_strength)
        .def_readonly("palm", &LEAP_HAND::palm)
        .def_readonly("thumb", &LEAP_HAND::thumb)
        .def_readonly("index", &LEAP_HAND::index)
        .def_readonly("middle", &LEAP_HAND::middle)
        .def_readonly("ring", &LEAP_HAND::ring)
        .def_readonly("pinky", &LEAP_HAND::pinky)
        .def_property_readonly("digits", [](const LEAP_HAND &hand)
                               {
            std::vector<LEAP_DIGIT> digits(5);
            std::copy(hand.digits, hand.digits + 5, digits.begin());
            return digits; });

    py::class_<LEAP_TRACKING_EVENT>(m, "TrackingEvent")
        .def_readonly("info", &LEAP_TRACKING_EVENT::info)
        .def_readonly("tracking_frame_id", &LEAP_TRACKING_EVENT::tracking_frame_id)
        .def_readonly("nHands", &LEAP_TRACKING_EVENT::nHands)
        .def_readonly("framerate", &LEAP_TRACKING_EVENT::framerate)
        .def_property_readonly("hands", [](const LEAP_TRACKING_EVENT &event)
                               {
            std::vector<LEAP_HAND> hands(event.nHands);
            std::copy(event.pHands, event.pHands + event.nHands, hands.begin());
            return hands; });

    py::class_<Connection>(m, "Connection")
        .def(py::init<>()) // Constructor
        .def("open", &Connection::open, "Open the connection")
        .def("close", &Connection::close, "Close the connection")
        .def("destroy", &Connection::destroy, "Destroy the connection")
        .def("getFrame", &Connection::getFrame, "Get the current frame")
        .def("getDeviceProperties", &Connection::getDeviceProperties, "Get device properties")
        .def("isConnected", &Connection::isConnected, "Check if connected")
        .def("setOnConnect", &Connection::setOnConnect, "Set on connect callback")
        .def("setOnDisconnect", &Connection::setOnDisconnect, "Set on disconnect callback")
        .def("setOnDeviceFound", &Connection::setOnDeviceFound, "Set on device found callback")
        .def("setOnDeviceLost", &Connection::setOnDeviceLost, "Set on device lost callback")
        .def("setOnDeviceFailure", &Connection::setOnDeviceFailure, "Set on device failure callback")
        .def("setOnPolicy", &Connection::setOnPolicy, "Set on policy callback")
        .def("setOnFrame", &Connection::setOnFrame, "Set on frame callback")
        .def("setOnImage", &Connection::setOnImage, "Set on image callback")
        .def("setOnIMU", &Connection::setOnIMU, "Set on IMU callback")
        .def("setOnTrackingMode", &Connection::setOnTrackingMode, "Set on tracking mode callback")
        .def("setDebugMessageCallback", &Connection::setDebugMessageCallback, "Set debug message callback");
}