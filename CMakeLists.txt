cmake_minimum_required(VERSION 3.13)
# All modules are now .cpp (converted from the original C sources - see
# git history / gitRelease03-09-26 for the pre-conversion C versions); no C
# sources remain, so this is a pure C++ project now.
project(imx8mp_controller_bridge CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LWS REQUIRED libwebsockets)

# libwebsockets here is built with mbedTLS support, so libwebsockets.h
# itself pulls in <mbedtls/ssl.h> - a dependency the libwebsockets.pc file
# used above does not declare, so pkg_check_modules's LWS_CFLAGS alone is
# not enough (this gap predates motor_inference.cpp; it would affect a
# from-scratch build of the original C-only sources the same way). Point
# this at your mbedTLS build/install, e.g. for the cross toolchain used
# elsewhere in this project: /home/<user>/tools/mbedtls-aarch64/include
# and .../lib.
set(MBEDTLS_INCLUDE_DIR "" CACHE PATH "Directory containing mbedtls/*.h headers")
set(MBEDTLS_LIB_DIR "" CACHE PATH "Directory containing libmbedtls/libmbedx509/libmbedcrypto")

# TensorFlow Lite for motor_inference.cpp (NPU by default via the VX
# external delegate, loaded at runtime - see motor_inference.h for the
# accuracy tradeoff that decision carries, and the CPU fallback). Point
# these at your Toradex/NXP eIQ Yocto SDK sysroot, e.g.:
#   -DTFLITE_INCLUDE_DIR=/opt/tdx-xwayland/7.5.0/sysroots/armv8a-tdx-linux/usr/include
#   -DTFLITE_LIB_DIR=/opt/tdx-xwayland/7.5.0/sysroots/armv8a-tdx-linux/usr/lib
set(TFLITE_INCLUDE_DIR "" CACHE PATH "Directory containing tensorflow/lite/... headers")
set(TFLITE_LIB_DIR "" CACHE PATH "Directory containing libtensorflow-lite")

add_executable(controller_bridge
    src/main.cpp
    src/controller_receiver.cpp
    src/bridge_server.cpp
    src/frame_bridge.cpp
    src/frame_stats.cpp
    src/motor_inference.cpp
    src/clock_offset.cpp
    src/timing_log.cpp
    src/gpio_toggle.cpp
)

target_include_directories(controller_bridge PRIVATE ${LWS_INCLUDE_DIRS} src)
target_link_directories(controller_bridge PRIVATE ${LWS_LIBRARY_DIRS})
target_link_libraries(controller_bridge PRIVATE ${LWS_LIBRARIES})

if(MBEDTLS_INCLUDE_DIR)
  target_include_directories(controller_bridge PRIVATE ${MBEDTLS_INCLUDE_DIR})
endif()
if(MBEDTLS_LIB_DIR)
  target_link_directories(controller_bridge PRIVATE ${MBEDTLS_LIB_DIR})
endif()
# libwebsockets.so has these as NEEDED (it was built with mbedTLS support),
# but the linker won't resolve them transitively without both the -L above
# and an explicit -l for each - without this the link fails with pages of
# "undefined reference to mbedtls_*" even though it's really a missing-
# library-path problem, not a missing-symbol one.
target_link_libraries(controller_bridge PRIVATE mbedtls mbedx509 mbedcrypto)
# Applies to both C and C++ sources; LWS_CFLAGS_OTHER is meant for the C
# side (libwebsockets has no C++ API), but the flags it currently carries
# are compiler-neutral (defines/warnings), so scoping it to a single
# language isn't needed.
target_compile_options(controller_bridge PRIVATE ${LWS_CFLAGS_OTHER} -Wall -Wextra)

if(TFLITE_INCLUDE_DIR)
  target_include_directories(controller_bridge PRIVATE ${TFLITE_INCLUDE_DIR})
endif()
if(TFLITE_LIB_DIR)
  target_link_directories(controller_bridge PRIVATE ${TFLITE_LIB_DIR})
endif()
# dl: dlopen'd internally by TFLite's own plugin machinery even on the
# CPU-only path used here. pthread: TFLite's threadpool
# (SetNumThreads() in motor_inference.cpp).
target_link_libraries(controller_bridge PRIVATE tensorflow-lite pthread dl)

install(TARGETS controller_bridge RUNTIME DESTINATION bin)

if(NOT TFLITE_INCLUDE_DIR OR NOT TFLITE_LIB_DIR)
  message(WARNING
    "TFLITE_INCLUDE_DIR / TFLITE_LIB_DIR not set - this build will fail "
    "unless tensorflow/lite headers and libtensorflow-lite are already on "
    "the compiler's default search path. Pass -DTFLITE_INCLUDE_DIR=... "
    "-DTFLITE_LIB_DIR=... pointing into your Yocto/Toradex SDK sysroot.")
endif()
if(NOT MBEDTLS_INCLUDE_DIR OR NOT MBEDTLS_LIB_DIR)
  message(WARNING
    "MBEDTLS_INCLUDE_DIR / MBEDTLS_LIB_DIR not set - this build will fail "
    "with a 'mbedtls/ssl.h: No such file' error unless mbedTLS headers are "
    "already on the compiler's default search path. Pass "
    "-DMBEDTLS_INCLUDE_DIR=... -DMBEDTLS_LIB_DIR=... pointing at your "
    "mbedTLS build (this gap exists in the original C-only build too, not "
    "just the motor_inference.cpp addition).")
endif()
