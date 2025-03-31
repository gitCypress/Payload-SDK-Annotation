#include "synt_trt_seg.h"
#include "src/cuda/postprocess.h"
#include "src/cuda/preprocess.h"

class YOLO11Seg : public TrtSeg
{
public:
    YOLO11Seg(const char* engine_name, int gpu_id, int mode, int& ret);
    ~YOLO11Seg();
    void process_mask(std::vector<SSBoxInfo> &res_box_info_list, int height, int width);
    void cut_mask(std::vector<SSBoxInfo> &res_box_info_list, int data_height, int data_width);
    int preprocess_image(void *data, int data_width, int data_height, int index);
    void decodeDetections(const int image_index);

    OBJECTLIST *m_object_list = NULL;                                   // 报警信息.
    
private:

    float m_conf_thres = 0.3;                                           // 得分阈值
    float m_nms_thres = 0.5;                                            // nms阈值                                   
};
