LOCAL_PATH := $(call my-dir)
OPENCV_CAMERA_MODULES:=off
OPENCV_INSTALL_MODULES:=on
OPENCV_LIB_TYPE:=STATIC
include $(CLEAR_VARS)

include ../sdk/native/jni/OpenCV.mk

LOCAL_MODULE    := sfmlib
LOCAL_SRC_FILES := sfmlib.cpp
LOCAL_LDLIBS    += -lm -llog -landroid
LOCAL_STATIC_LIBRARIES += android_native_app_glue

include $(BUILD_SHARED_LIBRARY)
$(call import-module,android/native_app_glue)
