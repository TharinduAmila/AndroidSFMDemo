LOCAL_PATH := $(call my-dir)
OPENCV_CAMERA_MODULES:=off
OPENCV_INSTALL_MODULES:=on
OPENCV_LIB_TYPE:=STATIC

LOCAL_ALLOW_UNDEFINED_SYMBOLS := true
export MAINDIR:= $(LOCAL_PATH)
#LAPACK, BLAS, F2C compilation
include $(CLEAR_VARS)
include $(MAINDIR)/clapack/Android.mk
LOCAL_PATH := $(MAINDIR)
include $(CLEAR_VARS)
LOCAL_MODULE:= lapack
LOCAL_STATIC_LIBRARIES := tmglib clapack blas f2c
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_C_INCLUDES)
LOCAL_EXPORT_LDLIBS := $(LOCAL_LDLIBS)
include $(BUILD_STATIC_LIBRARY)

#sba compilation
include $(CLEAR_VARS)
SBAOBJ = sba/sba_levmar.c sba/sba_levmar_wrap.c sba/sba_lapack.c sba/sba_crsm.c sba/sba_chkjac.c   
LOCAL_MODULE          := sba
LOCAL_SRC_FILES := $(SBAOBJ)  
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_C_INCLUDES)
LOCAL_EXPORT_LDLIBS := $(LOCAL_LDLIBS)
include $(BUILD_STATIC_LIBRARY)


include $(CLEAR_VARS)
include ../sdk/native/jni/OpenCV.mk
LOCAL_MODULE    := AndroidSFM
LOCAL_SRC_FILES := cvsba.cpp AndroidSFM.cpp BundleAdjustment.cpp Tracking.cpp Triangulation.cpp Utils.cpp SparseReconstruct.cpp

LOCAL_LDLIBS    += -lm -llog -landroid -ldl
LOCAL_STATIC_LIBRARIES += sba lapack android_native_app_glue

include $(BUILD_SHARED_LIBRARY)
$(call import-module,android/native_app_glue)
