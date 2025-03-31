#include "synt_trt.h"

class YOLO11Obb : public Trt
{
public:
    YOLO11Obb(const char* engine_name, int gpu_id, int mode, int& ret);
    ~YOLO11Obb();
    int preprocess_image(void *data, int data_width, int data_height, int index);
    void decodeDetections(const int image_index);

    OBJECTLIST *m_object_list = NULL;                                   // 报警信息.
    
private:

    float m_conf_thres = 0.8;                                           // 得分阈值
    float m_nms_thres = 0.3;                                            // nms阈值
};
