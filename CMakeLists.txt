cmake_minimum_required(VERSION 3.4...3.18)
project(leapmotion_pywrapper VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)


# setup leap sdk

if (WIN32)
	set(CMAKE_OBJECT_PATH_MAX 260)
	set(ULTRALEAP_PATH_ROOT "C:/Program Files/Ultraleap")
	set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
elseif (APPLE)
   set(ULTRALEAP_PATH_ROOT "/Applications/Ultraleap\ Hand\ Tracking.app/Contents/LeapSDK/lib/cmake/LeapSDK")
else()
   # Linux and other OSs
   set(ULTRALEAP_PATH_ROOT "")
endif()

message(STATUS "ULTRALEAP_PATH_ROOT: ${ULTRALEAP_PATH_ROOT}")

find_package(LeapSDK
     5
     REQUIRED
     PATHS
	 	"${ULTRALEAP_PATH_ROOT}")

if (UNIX)    
    find_package(Threads REQUIRED)    
endif (UNIX)

add_subdirectory(pybind11)

add_library(
	libConnectionCXX
	OBJECT
	"src/Connection.cpp"
)
target_link_libraries(
	libConnectionCXX
	PUBLIC
	LeapSDK::LeapC)

get_target_property(
	LEAPC_IMPORTED_CONFIG
	LeapSDK::LeapC
	IMPORTED_CONFIGURATIONS
)

get_target_property(
	LEAPC_SHARED_LIB_PATH
	LeapSDK::LeapC
	IMPORTED_LOCATION_${LEAPC_IMPORTED_CONFIG}
)


pybind11_add_module(leapmotion_pywrapper src/main.cpp)

add_custom_command(
	TARGET
		leapmotion_pywrapper
	POST_BUILD
	COMMAND
		${CMAKE_COMMAND} -E copy
		${LEAPC_SHARED_LIB_PATH}
		$<TARGET_FILE_DIR:leapmotion_pywrapper>)

target_link_libraries(
	leapmotion_pywrapper
	PRIVATE
		libConnectionCXX
		LeapSDK::LeapC)

# EXAMPLE_VERSION_INFO is defined by setup.py and passed into the C++ code as a
# define (VERSION_INFO) here.
target_compile_definitions(leapmotion_pywrapper
                           PRIVATE VERSION_INFO=${EXAMPLE_VERSION_INFO})

add_executable(
	leapmotion_pywrapper_test
	"src/test.cpp"
)
target_link_libraries(
	leapmotion_pywrapper_test
	PRIVATE
		libConnectionCXX
		LeapSDK::LeapC)
