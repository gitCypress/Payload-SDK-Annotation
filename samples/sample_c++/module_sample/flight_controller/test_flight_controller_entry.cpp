/**
 ********************************************************************
 * @file    test_flight_controller_entry.cpp
 * @brief
 *
 * @copyright (c) 2018 DJI. All rights reserved.
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
#include <iostream>
#include <flight_control/test_flight_control.h>
#include "test_flight_controller_entry.h"
#include "dji_logger.h"
#include "test_flight_controller_command_flying.h"
#include <waypoint_v2/test_waypoint_v2.h>
#include <waypoint_v3/test_waypoint_v3.h>
#include <interest_point/test_interest_point.h>

/* Private constants ---------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private values -------------------------------------------------------------*/

/* Private functions declaration ---------------------------------------------*/

/* Exported functions definition ---------------------------------------------*/

/**
 * @brief 飞行控制器示例程序入口函数
 * 
 * 该函数提供了一个交互式菜单，允许用户选择并运行不同的飞行控制示例。
 * 包括基本飞行控制、航点任务、兴趣点任务和紧急降落系统等功能。
 * 
 * 功能列表：
 * - 键盘控制飞行
 * - 起飞和降落
 * - 位置控制飞行
 * - 返航和强制降落
 * - 速度控制飞行
 * - 紧急停止飞行
 * - 设置和获取飞行参数
 * - 航点2.0任务（仅支持M300 RTK）
 * - 航点3.0任务（不支持M300 RTK）
 * - 兴趣点任务（仅支持M3E/M3T）
 * - EU-C6 FTS触发示例（仅支持M3D/M3DT）
 * 
 * @note 不同的示例功能支持不同的飞行器型号，使用前请确认兼容性
 * @return void
 */
void DjiUser_RunFlightControllerSample(void)
{
    // 获取PSDK操作系统抽象层处理程序，用于访问操作系统功能（如延时）
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    char inputSelectSample;

start:
    // 短暂延时，确保系统稳定
    osalHandler->TaskSleepMs(100);

    // 显示飞行控制器示例菜单，列出所有可用的示例选项
    std::cout
        << "\n"
        << "| Available commands:                                                                                            |\n"
        << "| [0] Flight controller sample - control flying with keyboard                                                    |\n"
        << "| [1] Flight controller sample - take off landing                                                                |\n"
        << "| [2] Flight controller sample - take off position ctrl landing                                                  |\n"
        << "| [3] Flight controller sample - take off go home force landing                                                  |\n"
        << "| [4] Flight controller sample - take off velocity ctrl landing                                                  |\n"
        << "| [5] Flight controller sample - arrest flying                                                                   |\n"
        << "| [6] Flight controller sample - set get parameters                                                              |\n"
        << "| [7] Waypoint 2.0 sample - run airline mission by settings (only support on M300 RTK)                           |\n"
        << "| [8] Waypoint 3.0 sample - run airline mission by kmz file (not support on M300 RTK)                            |\n"
        << "| [9] Interest point sample - run interest point mission by settings (only support on M3E/M3T)                   |\n"
        << "| [a] EU-C6 FTS trigger sample - receive fts callback to trigger parachute function (only support on M3D/M3DT)   |\n"
        << "| [b] 自定义示例：正圆飞行   |\n"
        << std::endl;

    // 获取用户输入的命令
    std::cin >> inputSelectSample;
    switch (inputSelectSample) {
        case '0':
            // 运行键盘控制飞行示例 - 通过键盘输入控制飞行器移动，优先参考该示例，实现上最为全面
            // TODO: 避障功能的关闭；配套安全保护；飞行控制权获取问题
            // 参考：test_flight_controller_command_flying.cpp
            DjiUser_RunFlightControllerCommandFlyingSample();
            goto start;
        case '1':
            // 运行基本起飞降落示例 - 演示简单的起飞和降落过程
            // 参考：test_flight_control.cpp中的相应函数
            DjiTest_FlightControlRunSample(E_DJI_TEST_FLIGHT_CTRL_SAMPLE_SELECT_TAKE_OFF_LANDING);
            goto start;
        case '2':
            // 运行位置控制降落示例 - 起飞后通过位置控制移动，然后降落
            // 位置控制是指通过指定目标位置坐标来控制飞行器移动
            DjiTest_FlightControlRunSample(E_DJI_TEST_FLIGHT_CTRL_SAMPLE_SELECT_TAKE_OFF_POSITION_CTRL_LANDING);
            goto start;
        case '3':
            // 运行返航强制降落示例 - 起飞后执行返航，然后强制降落
            // 返航是指飞行器自动飞回预设的返航点
            DjiTest_FlightControlRunSample(E_DJI_TEST_FLIGHT_CTRL_SAMPLE_SELECT_TAKE_OFF_GO_HOME_FORCE_LANDING);
            goto start;
        case '4':
            // 运行速度控制降落示例 - 起飞后通过速度控制移动，然后降落
            // 速度控制是指通过指定目标速度来控制飞行器移动
            DjiTest_FlightControlRunSample(E_DJI_TEST_FLIGHT_CTRL_SAMPLE_SELECT_TAKE_OFF_VELOCITY_CTRL_LANDING);
            goto start;
        case '5':
            // 运行紧急停止飞行示例 - 演示如何紧急停止飞行器
            // 紧急停止会立即停止电机，飞行器将会坠落，仅用于紧急情况
            DjiTest_FlightControlRunSample(E_DJI_TEST_FLIGHT_CTRL_SAMPLE_SELECT_ARREST_FLYING);
            goto start;
        case '6':
            // 运行参数设置获取示例 - 演示如何设置和获取飞行控制参数
            // 包括RTK开关、避障开关、返航高度等参数
            DjiTest_FlightControlRunSample(E_DJI_TEST_FLIGHT_CTRL_SAMPLE_SELECT_SET_GET_PARAM);
            goto start;
        case '7':
            // 运行航点2.0任务示例 - 通过设置执行航线任务
            // 航点2.0支持更复杂的航线规划和动作，仅支持M300 RTK
            // 参考：waypoint_v2/test_waypoint_v2.cpp
            DjiTest_WaypointV2RunSample();
            break;
        case '8':
            // 运行航点3.0任务示例 - 通过kmz文件执行航线任务
            // 航点3.0是更新的航点任务系统，不支持M300 RTK
            // 参考：waypoint_v3/test_waypoint_v3.cpp
            DjiTest_WaypointV3RunSample();
            break;
        case '9':
            // 运行兴趣点任务示例 - 通过设置执行兴趣点环绕任务
            // 兴趣点任务使飞行器围绕指定点飞行，仅支持M3E/M3T
            // 参考：interest_point/test_interest_point.cpp
            DjiTest_InterestPointRunSample();
            break;
        case 'a':
            // 运行EU-C6 FTS触发示例 - 接收FTS回调触发降落伞功能
            // FTS (Flight Termination System) 是飞行终止系统，用于紧急情况
            // 仅支持M3D/M3DT机型
            DjiTest_FlightControlRunSample(E_DJI_TEST_FLIGHT_CTRL_SAMPLE_SELECT_FTS_TRIGGER);
            break;        
        case 'b':
            // 运行自定义正圆飞行示例
            Custom_FlightAction_Circle();
            break;
        case 'q':
            // 退出示例程序
            break;
        default:
            // 处理无效输入
            USER_LOG_ERROR("Input command is invalid");
            goto start;
    }
}

/* Private functions definition-----------------------------------------------*/

/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/
