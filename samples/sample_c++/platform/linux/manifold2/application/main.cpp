/**
 ********************************************************************
 * @file    main.cpp
 * @brief   PSDK示例应用程序的主入口文件，展示了如何使用PSDK的各种功能模块
 *
 * @copyright (c) 2021 DJI. All rights reserved.
 *
 * All information contained herein is, and remains, the property of DJI.
 * The intellectual and technical concepts contained herein are proprietary
 * to DJI and may be covered by U.S. and foreign patents, patents in process,
 * and protected by trade secret or copyright law.  Dissemination of this
 * information, including but not limited to data and other proprietary
 * material(s) incorporated within the information, in any form, is strictly
 * prohibited without the express written consent of DJI.
 *
 * If you receive this source code without DJI's authorization, you may not
 * further disseminate the information, and you must immediately remove the
 * source code and notify DJI of its removal. DJI reserves the right to pursue
 * legal actions against you for any loss(es) or damage(s) caused by your
 * failure to do so.
 *
 *********************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <liveview/test_liveview_entry.hpp>           // 实时视图功能模块头文件
#include <perception/test_perception_entry.hpp>       // 感知功能模块头文件
#include <flight_control/test_flight_control.h>       // 飞行控制功能模块头文件
#include <gimbal/test_gimbal_entry.hpp>               // 云台控制功能模块头文件
#include "application.hpp"                            // 应用程序基础类头文件
#include "fc_subscription/test_fc_subscription.h"     // 飞控数据订阅功能模块头文件
#include <gimbal_emu/test_payload_gimbal_emu.h>       // 云台模拟器功能模块头文件
#include <camera_emu/test_payload_cam_emu_media.h>    // 相机媒体模拟器功能模块头文件
#include <camera_emu/test_payload_cam_emu_base.h>     // 相机基础模拟器功能模块头文件
#include <dji_logger.h>                               // PSDK日志管理模块头文件
#include "widget/test_widget.h"                       // 自定义控件功能模块头文件
#include "widget/test_widget_speaker.h"               // 扬声器控件功能模块头文件
#include <power_management/test_power_management.h>   // 电源管理功能模块头文件
#include "data_transmission/test_data_transmission.h" // 数据传输功能模块头文件
#include <flight_controller/test_flight_controller_entry.h> // 飞行控制器功能模块头文件
#include <positioning/test_positioning.h>             // 定位功能模块头文件
#include <hms_manager/hms_manager_entry.h>            // 健康管理系统功能模块头文件
#include "camera_manager/test_camera_manager_entry.h" // 相机管理功能模块头文件

/* Private constants ---------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private values -------------------------------------------------------------*/

/* Private functions declaration ---------------------------------------------*/

/* Exported functions definition ---------------------------------------------*/
/**
 * @brief 主函数 - PSDK示例应用程序的入口点
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return 程序执行结果
 */
int main(int argc, char **argv)
{
    // 初始化应用程序对象，传入命令行参数
    Application application(argc, argv);
    char inputChar;
    
    // 获取PSDK操作系统抽象层处理程序，用于访问操作系统功能
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    T_DjiReturnCode returnCode;
    T_DjiTestApplyHighPowerHandler applyHighPowerHandler;

start:
    // 显示可用的示例命令菜单
    std::cout
        << "\n"
        << "| Available commands:                                                                              |\n"
        << "| [0] Fc subscribe sample - subscribe quaternion and gps data                                      |\n"
        << "| [1] Flight controller sample - you can control flying by PSDK                                    |\n"
        << "| [2] Hms info manager sample - get health manger system info by language                          |\n"
        << "| [a] Gimbal manager sample - you can control gimbal by PSDK                                       |\n"
        << "| [c] Camera stream view sample - display the camera video stream                                  |\n"
        << "| [d] Stereo vision view sample - display the stereo image                                         |\n"
        << "| [e] Run camera manager sample - you can test camera's functions interactively                    |\n"
        << "| [f] Start rtk positioning sample - you can receive rtk rtcm data when rtk signal is ok           |\n"
        << std::endl;

    // 获取用户输入的命令
    std::cin >> inputChar;
    switch (inputChar) {
        case '0':
            // 运行飞控数据订阅示例 - 订阅四元数和GPS数据
            // 参考文档: doc/usage_docs/cn/40.function-set/10.basic-function/70.flight-control.md
            DjiTest_FcSubscriptionRunSample();
            break;
        case '1':
            // 运行飞行控制器示例 - 通过PSDK控制飞行
            // 参考文档: doc/usage_docs/cn/40.function-set/10.basic-function/70.flight-control.md
            DjiUser_RunFlightControllerSample();
            break;
        case '2':
            // 运行健康管理系统示例 - 获取HMS信息
            // 参考文档: doc/usage_docs/cn/40.function-set/10.basic-function/90.hms-function.md
            DjiUser_RunHmsManagerSample();
            break;
        case 'a':
            // 运行云台管理示例 - 通过PSDK控制云台
            // 参考文档: doc/usage_docs/cn/40.function-set/10.basic-function/40.gimbal-function.md
            // 参考文档: doc/usage_docs/cn/40.function-set/10.basic-function/50.gimbal-management.md
            DjiUser_RunGimbalManagerSample();
            break;
        case 'c':
            // 运行相机视频流示例 - 显示相机视频流
            // 参考文档: doc/usage_docs/cn/40.function-set/20.advanced-function/10.camera-video-stream-transmission.md
            DjiUser_RunCameraStreamViewSample();
            break;
        case 'd':
            // 运行立体视觉示例 - 显示立体图像
            // 参考文档: doc/usage_docs/cn/40.function-set/20.advanced-function/100.perception-image.md
            DjiUser_RunStereoVisionViewSample();
            break;
        case 'e':
            // 运行相机管理示例 - 交互式测试相机功能
            // 参考文档: doc/usage_docs/cn/40.function-set/10.basic-function/20.basic-camera-function.md
            // 参考文档: doc/usage_docs/cn/40.function-set/10.basic-function/30.basic-camera-management.md
            DjiUser_RunCameraManagerSample();
            break;
        case 'f':
            // 启动RTK定位服务示例 - 当RTK信号良好时接收RTCM数据
            // 参考文档: doc/usage_docs/cn/40.function-set/20.advanced-function/60.positioning.md
            returnCode = DjiTest_PositioningStartService();
            if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                // 如果RTK定位服务初始化失败，记录错误日志
                USER_LOG_ERROR("rtk positioning sample init error");
                break;
            }

            // 记录RTK定位服务启动成功的日志
            USER_LOG_INFO("Start rtk positioning sample successfully");
            break;
        default:
            // 处理无效输入
            break;
    }

    // 暂停2秒，让用户查看示例执行结果
    osalHandler->TaskSleepMs(2000);

    // 返回菜单开始处，等待用户下一个命令
    goto start;
}

/* Private functions definition-----------------------------------------------*/

/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/
