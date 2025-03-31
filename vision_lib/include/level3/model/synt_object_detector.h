#ifndef SYNT_OBJECT_DETECTOR_H_
#define SYNT_OBJECT_DETECTOR_H_
#include "synt_detector_type.h"

// 初始化
extern "C" int syntDetectionInit(void** handle, CONFIGURE* configure, NETMSG* net_msg);
extern "C" int syntDetectionInitV7(void** handle, CONFIGURE* configure, NETMSG* net_msg);
extern "C" int syntClassifyInit(void** handle, CONFIGURE* configure, NETMSG* net_msg);
extern "C" int syntClassifyDynamicInit(void** handle, CONFIGURE* configure, NETMSG* net_msg);
extern "C" int syntSegmentionInit(void** handle, CONFIGURE* configure, NETMSG* net_msg);
extern "C" int syntOrientedBoundingBoxInit(void** handle, CONFIGURE* configure, NETMSG* net_msg);

// 预处理
extern "C" int syntDataPre(void* handle, void* src, int input_width, int input_height, int index);

// 推理+后处理
OBJECTLIST * syntDetect(void* handle);

// 预处理+推理+后处理
extern "C" OBJECTLIST * syntDetect(void* handle, void* src, int input_width, int input_height);
extern "C" OBJECTLIST * syntDetectV7(void* handle, void* src, int input_width, int input_height);
extern "C" OBJECTLIST * syntClassify(void *handle, void *src, int input_width, int input_height, OBJECTLIST *object_list);
extern "C" OBJECTLIST * syntClassifyDynamic(void *handle, void *src, int input_width, int input_height, OBJECTLIST *object_list);
extern "C" OBJECTLIST * syntSegmention(void* handle, void* src, int input_width, int input_height);
extern "C" OBJECTLIST * syntOrientedBoundingBox(void* handle, void* src, int input_width, int input_height);

// 卸载
extern "C" int syntDetectionUnInit(void* handle);
extern "C" int syntDetectionUnInitV7(void* handle);
extern "C" int syntClassifyUnInit(void* handle);
extern "C" int syntClassifyDynamicUnInit(void* handle);
extern "C" int syntSegmentionUnInit(void* handle);
extern "C" int syntHandleUnInit(void* handle);

#endif
