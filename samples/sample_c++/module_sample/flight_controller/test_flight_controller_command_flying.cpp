/**
 ********************************************************************
 * @file    test_flight_controller_command_flying.cpp
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
#include <termios.h>
#include <utils/util_misc.h>
#include <utils/util_file.h>
#include <utils/cJSON.h>
#include <dji_aircraft_info.h>
#include "test_flight_controller_command_flying.h"
#include "dji_flight_controller.h"
#include "dji_logger.h"
#include "dji_fc_subscription.h"
#include "cmath"

#ifdef OPEN_CV_INSTALLED

#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace cv;
#endif

/* Private constants ---------------------------------------------------------*/
#define DJI_TEST_COMMAND_FLYING_TASK_STACK_SIZE                          2048
#define DJI_TEST_COMMAND_FLYING_CTRL_FREQ                                50
#define DJI_TEST_COMMAND_FLYING_GO_HOME_ALTITUDE                         50
#define DJI_TEST_COMMAND_FLYING_CONTROL_SPEED_DEFAULT                    5
#define DJI_TEST_COMMAND_FLYING_RC_LOST_ACTION_STR_MAX_LEN               32
#define DJI_TEST_COMMAND_FLYING_CONFIG_DIR_PATH_LEN_MAX                  (256)

/* Private types -------------------------------------------------------------*/

/* Private values -------------------------------------------------------------*/
static T_DjiTaskHandle s_commandFlyingTaskHandle;
static T_DjiTaskHandle s_statusDisplayTaskHandle;
static T_DjiFlightControllerJoystickCommand s_flyingCommand = {0};
static uint16_t s_inputFlag = 0;
static dji_f32_t s_flyingSpeed = DJI_TEST_COMMAND_FLYING_CONTROL_SPEED_DEFAULT;
static uint16_t s_goHomeAltitude = DJI_TEST_COMMAND_FLYING_GO_HOME_ALTITUDE;
static char s_rcLostActionString[DJI_TEST_COMMAND_FLYING_RC_LOST_ACTION_STR_MAX_LEN] = {0};
static T_DjiFlightControllerHomeLocation s_homeLocation = {0};
static T_DjiFcSubscriptionGpsPosition s_gpsPosition = {0};
static bool isFirstUpdateConfig = false;
static bool isCommandFlyingTaskStart = false;
static uint32_t s_statusDisplayTaskCnt = 0;
static T_DjiFcSubscriptionSingleBatteryInfo singleBatteryInfo1 = {0};
static T_DjiFcSubscriptionSingleBatteryInfo singleBatteryInfo2 = {0};

/* Private functions declaration ---------------------------------------------*/
static void *DjiUser_FlightControllerCommandFlyingTask(void *arg);
static void *DjiUser_FlightControllerStatusDisplayTask(void *arg);
static void DjiUser_ShowFlightStatusByOpenCV(void);
static void DjiUser_FlightControllerVelocityAndYawRateCtrl(T_DjiFlightControllerJoystickCommand command);
static int DjiUser_ScanKeyboardInput(void);
static T_DjiReturnCode
DjiUser_FlightCtrlJoystickCtrlAuthSwitchEventCb(T_DjiFlightControllerJoystickCtrlAuthorityEventInfo eventData);
static T_DjiVector3f DjiUser_FlightControlGetValueOfQuaternion(void);
static T_DjiFcSubscriptionGpsPosition DjiUser_FlightControlGetValueOfGpsPosition(void);
static T_DjiFcSubscriptionAltitudeOfHomePoint DjiUser_FlightControlGetValueOfRelativeHeight(void);
static T_DjiFcSubscriptionPositionVO DjiUser_FlightControlGetValueOfPositionVo(void);
static T_DjiFcSubscriptionControlDevice DjiUser_FlightControlGetValueOfControlDevice(void);
static T_DjiFcSubscriptionSingleBatteryInfo DjiUser_FlightControlGetValueOfBattery1(void);
static T_DjiFcSubscriptionSingleBatteryInfo DjiUser_FlightControlGetValueOfBattery2(void);
static T_DjiReturnCode DjiUser_FlightControlUpdateConfig(void);

// 正圆飞行，参考函数 DjiUser_RunFlightControllerCommandFlyingSample
void Custom_FlightAction_Circle(void){
    dji_f32_t custom_yaw = 30; // 顺时针旋转，单位 deg/s

    T_DjiReturnCode returnCode;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();

    // 创建命令飞行任务，用于处理飞行控制命令
    // 该任务会初始化飞行控制器，订阅飞行数据，并循环、有保护地执行用户输入的飞行命令
    returnCode = osalHandler->TaskCreate("command_flying_task", DjiUser_FlightControllerCommandFlyingTask,
                                         DJI_TEST_COMMAND_FLYING_TASK_STACK_SIZE, NULL,
                                         &s_commandFlyingTaskHandle);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("- [custom] Create command flying task failed, errno = 0x%08llX", returnCode);
        return;
    }

    // 创建状态显示任务，用于显示飞行器的实时状态信息
    // 该任务会通过OpenCV创建一个窗口，显示飞行器的姿态、位置、电池等信息
    returnCode = osalHandler->TaskCreate("status_display_task", DjiUser_FlightControllerStatusDisplayTask,
                                         DJI_TEST_COMMAND_FLYING_TASK_STACK_SIZE, NULL,
                                         &s_statusDisplayTaskHandle);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("- [custom] Create status display task failed, errno = 0x%08llX", returnCode);
        return;
    }

    // 等待任务初始化完成
    osalHandler->TaskSleepMs(1000);

    // 获取控制权
    returnCode = DjiFlightController_ObtainJoystickCtrlAuthority();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("- [custom] 飞行控制权获取失败，错误代码: 0x%08X", returnCode);
        return;
    }
    osalHandler->TaskSleepMs(1000);  // 见其他示例

    // 起飞请求
    returnCode = DjiFlightController_StartTakeoff();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("- [custom] 请求起飞失败，错误代码: 0x%08llX", returnCode);
        return;
    }
    USER_LOG_INFO(" - [custom] 起飞。输入 q 返航\r\n");

    // uint16_t clock = 0; 可以添加定时逻辑

    // 简单的画圆逻辑
    while (DjiUser_ScanKeyboardInput() != 'q'){  // 输入 q 退出循环
        osalHandler->TaskSleepMs(10);

        s_flyingCommand.x = s_flyingSpeed;  // 默认速度
        s_flyingCommand.y = 0;
        s_flyingCommand.z = 0;
        s_flyingCommand.yaw = custom_yaw;  // 自定义角速度
        s_inputFlag = 0;
    }

    // 立即清零
    s_flyingCommand = {0, 0, 0, 0};

    // 返航
    returnCode = DjiFlightController_StartGoHome();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("- [custom] 请求返航失败，错误代码: 0x%08llX", returnCode);
        return;
    }
    USER_LOG_INFO(" - Start go home\r\n");
}

/* Exported functions definition ---------------------------------------------*/
/**
 * @brief 飞行控制器命令飞行示例函数
 * 
 * 该函数演示了如何通过键盘输入控制飞行器执行各种飞行动作，包括：
 * - 基本移动控制（前、后、左、右、上、下）
 * - 偏航控制（左转、右转）
 * - 起飞、降落、返航等飞行操作
 * - 电机控制（启动、停止、紧急停止）
 * - 飞行参数配置更新
 * - 紧急制动和避障功能控制
 * 
 * 函数创建两个任务：
 * 1. 命令飞行任务：初始化飞行控制器，订阅飞行数据，执行飞行命令
 * 2. 状态显示任务：显示飞行器的实时状态信息
 * 
 * @return void
 */
void DjiUser_RunFlightControllerCommandFlyingSample(void)
{
    T_DjiReturnCode returnCode;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();

    // 创建命令飞行任务，用于处理飞行控制命令
    // 该任务会初始化飞行控制器，订阅飞行数据，并循环、有保护地执行用户输入的飞行命令
    returnCode = osalHandler->TaskCreate("command_flying_task", DjiUser_FlightControllerCommandFlyingTask,
                                         DJI_TEST_COMMAND_FLYING_TASK_STACK_SIZE, NULL,
                                         &s_commandFlyingTaskHandle);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Create command flying task failed, errno = 0x%08llX", returnCode);
        return;
    }

    // 创建状态显示任务，用于显示飞行器的实时状态信息
    // 该任务会通过OpenCV创建一个窗口，显示飞行器的姿态、位置、电池等信息
    returnCode = osalHandler->TaskCreate("status_display_task", DjiUser_FlightControllerStatusDisplayTask,
                                         DJI_TEST_COMMAND_FLYING_TASK_STACK_SIZE, NULL,
                                         &s_statusDisplayTaskHandle);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Create status display task failed, errno = 0x%08llX", returnCode);
        return;
    }

    // 等待任务初始化完成
    osalHandler->TaskSleepMs(1000);

    // 主循环，处理用户键盘输入
    while (1) {
        osalHandler->TaskSleepMs(1);
        switch (DjiUser_ScanKeyboardInput()) {
            case 'W':
            case 'w':
                // 向前飞行 - 设置X轴速度为正值，表示向前移动
                // 参考文档: https://developer.dji.com/doc/payload-sdk-tutorial/en/function-set/basic-function/flight-control.html
                s_flyingCommand.x = s_flyingSpeed;
                s_flyingCommand.y = 0;
                s_flyingCommand.z = 0;
                s_flyingCommand.yaw = 0;
                s_inputFlag = 0;
                USER_LOG_INFO(" - Front\r\n");
                break;
            case 'S':
            case 's':
                // 向后飞行 - 设置X轴速度为负值，表示向后移动
                s_flyingCommand.x = -s_flyingSpeed;
                s_flyingCommand.y = 0;
                s_flyingCommand.z = 0;
                s_flyingCommand.yaw = 0;
                s_inputFlag = 0;
                USER_LOG_INFO(" - Near\r\n");
                break;
            case 'A':
            case 'a':
                // 向左飞行 - 设置Y轴速度为负值，表示向左移动
                s_flyingCommand.x = 0;
                s_flyingCommand.y = -s_flyingSpeed;
                s_flyingCommand.z = 0;
                s_flyingCommand.yaw = 0;
                s_inputFlag = 0;
                USER_LOG_INFO(" - Left\r\n");
                break;
            case 'D':
            case 'd':
                // 向右飞行 - 设置Y轴速度为正值，表示向右移动
                s_flyingCommand.x = 0;
                s_flyingCommand.y = s_flyingSpeed;
                s_flyingCommand.z = 0;
                s_flyingCommand.yaw = 0;
                s_inputFlag = 0;
                USER_LOG_INFO(" - Right\r\n");
                break;
            case 'Q':
            case 'q':
                // 向上飞行 - 设置Z轴速度为正值，表示向上移动
                s_flyingCommand.x = 0;
                s_flyingCommand.y = 0;
                s_flyingCommand.z = s_flyingSpeed;
                s_flyingCommand.yaw = 0;
                s_inputFlag = 0;
                USER_LOG_INFO(" - Up\r\n");
                break;
            case 'E':
            case 'e':
                // 向下飞行 - 设置Z轴速度为负值，表示向下移动
                s_flyingCommand.x = 0;
                s_flyingCommand.y = 0;
                s_flyingCommand.z = -s_flyingSpeed;
                s_flyingCommand.yaw = 0;
                s_inputFlag = 0;
                USER_LOG_INFO(" - Down\r\n");
                break;
            case 'Z':
            case 'z':
                // 左转 - 设置偏航角速度为负值，表示逆时针旋转
                s_flyingCommand.x = 0;
                s_flyingCommand.y = 0;
                s_flyingCommand.z = 0;
                s_flyingCommand.yaw = -30;
                s_inputFlag = 0;
                USER_LOG_INFO(" - Yaw--\r\n");
                break;
            case 'C':
            case 'c':
                // 右转 - 设置偏航角速度为正值，表示顺时针旋转
                s_flyingCommand.x = 0;
                s_flyingCommand.y = 0;
                s_flyingCommand.z = 0;
                s_flyingCommand.yaw = 30;
                s_inputFlag = 0;
                USER_LOG_INFO(" - Yaw++\r\n");
                break;
            case 'R':
            case 'r':
                // 起飞 - 首先获取控制权限，然后执行起飞动作
                // 参考API: DjiFlightController_ObtainJoystickCtrlAuthority, DjiFlightController_StartTakeoff
                DjiFlightController_ObtainJoystickCtrlAuthority();
                DjiFlightController_StartTakeoff();
                USER_LOG_INFO(" - Take off\r\n");
                break;
            case 'F':
            case 'f':
                // 强制降落 - 忽略智能降落功能，直接降落
                // 参考API: DjiFlightController_StartForceLanding
                // 注意：此功能可能导致飞行器坠毁，仅在紧急情况下使用
                DjiFlightController_StartForceLanding();
                USER_LOG_INFO(" - Force landing\r\n");
                break;
            case 'H':
            case 'h':
                // 开始返航 - 飞行器自动飞回预设的返航点
                // 参考API: DjiFlightController_StartGoHome
                DjiFlightController_StartGoHome();
                USER_LOG_INFO(" - Start go home\r\n");
                break;
            case 'Y':
            case 'y':
                // 取消返航 - 取消当前的返航动作
                // 参考API: DjiFlightController_CancelGoHome
                DjiFlightController_CancelGoHome();
                USER_LOG_INFO(" - Cancel go home\r\n");
                break;
            case 'G':
            case 'g':
                // 开始降落 - 在当前位置开始降落
                // 参考API: DjiFlightController_StartLanding
                DjiFlightController_StartLanding();
                USER_LOG_INFO(" - Start landing\r\n");
                break;
            case 'T':
            case 't':
                // 取消降落 - 取消当前的降落动作
                // 参考API: DjiFlightController_CancelLanding
                DjiFlightController_CancelLanding();
                USER_LOG_INFO(" - Cancel landing\r\n");
                break;
            case 'V':
            case 'v':
                // 确认降落 - 当飞行器距地面0.7米时，确认继续降落
                // 参考API: DjiFlightController_StartConfirmLanding
                DjiFlightController_StartConfirmLanding();
                USER_LOG_INFO(" - Confirm landing\r\n");
                break;
            case 'X':
            case 'x':
                // 设置返航点 - 将当前飞行器位置设为返航点
                // 参考API: DjiFlightController_SetHomeLocationUsingCurrentAircraftLocation
                s_homeLocation.longitude = (dji_f64_t) s_gpsPosition.x / 10000000;
                s_homeLocation.latitude = (dji_f64_t) s_gpsPosition.y / 10000000;
                DjiFlightController_SetHomeLocationUsingCurrentAircraftLocation();
                USER_LOG_INFO(" - Set home location\r\n");
                break;
            case 'P':
            case 'p':
                // 紧急停止电机 - 立即停止所有电机，飞行器将会坠落
                // 参考API: DjiFlightController_EmergencyStopMotor
                // 注意：此功能仅用于紧急情况，会导致飞行器坠落
                DjiFlightController_EmergencyStopMotor(DJI_FLIGHT_CONTROLLER_ENABLE_EMERGENCY_STOP_MOTOR,
                                                       (char *) "Test is ok");
                USER_LOG_INFO(" - Emergency stop motor\r\n");
                break;
            case 'B':
            case 'b':
                // 启动电机 - 当飞行器在地面时启动电机
                // 参考API: DjiFlightController_TurnOnMotors
                DjiFlightController_TurnOnMotors();
                USER_LOG_INFO(" - Turn on motors\r\n");
                break;
            case 'N':
            case 'n':
                // 关闭电机 - 当飞行器在地面时关闭电机
                // 参考API: DjiFlightController_TurnOffMotors
                DjiFlightController_TurnOffMotors();
                USER_LOG_INFO(" - Turn off motors\r\n");
                break;
            case 'J':
            case 'j':
                // 更新配置 - 从配置文件中读取并更新飞行参数
                // 包括返航高度、失控动作、RTK开关、避障开关等
                DjiUser_FlightControlUpdateConfig();
                USER_LOG_INFO(" - Update config\r\n");
                break;
            case 'I':
            case 'i':
                // 启用紧急停止飞行 - 飞行器将悬停在当前位置
                // 参考API: DjiFlightController_ArrestFlying
                DjiFlightController_ArrestFlying();
                USER_LOG_INFO(" - Enable arrest flying\r\n");
                break;
            case 'O':
            case 'o':
                // 取消紧急停止飞行 - 恢复飞行器的控制
                // 参考API: DjiFlightController_CancelArrestFlying
                DjiFlightController_CancelArrestFlying();
                USER_LOG_INFO(" - Disable arrest flying\r\n");
                break;
            case 'K':
            case 'k':
                // 执行紧急制动 - 飞行器将立即停止并悬停
                // 参考API: DjiFlightController_ExecuteEmergencyBrakeAction
                DjiFlightController_ExecuteEmergencyBrakeAction();
                USER_LOG_INFO(" - Brake\r\n");
                break;
            case 'L':
            case 'l':
                // 取消紧急制动 - 恢复飞行器的控制
                // 参考API: DjiFlightController_CancelEmergencyBrakeAction
                DjiFlightController_CancelEmergencyBrakeAction();
                USER_LOG_INFO(" - Disable Brake\r\n");
                break;
            case 'M':
            case 'm':
                // 获取摇杆控制权限 - 允许PSDK控制飞行器
                // 参考API: DjiFlightController_ObtainJoystickCtrlAuthority
                // 注意：遥控器必须处于P模式才能获取控制权限
                DjiFlightController_ObtainJoystickCtrlAuthority();
                USER_LOG_INFO(" - Obtain joystick ctrl authority\r\n");
                break;
        }
    }
}

/* Private functions definition-----------------------------------------------*/
/**
 * @brief 飞行控制器命令飞行任务函数，使用
 * 
 * 该函数是命令飞行任务的主体，负责：
 * 1. 初始化飞行控制器
 * 2. 初始化数据订阅模块
 * 3. 订阅飞行器状态数据（四元数、GPS位置、高度等）
 * 4. 更新飞行配置
 * 5. 注册摇杆控制权限事件回调
 * 6. 循环执行用户输入的飞行命令，三维坐标值结构 s_flyingCommand 在这里得到维护，同时有保护机制
 * 
 * @param arg 任务参数（未使用）
 * @return 任务返回值（未使用）
 */
static void *DjiUser_FlightControllerCommandFlyingTask(void *arg)
{
    T_DjiReturnCode returnCode;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    T_DjiFlightControllerRidInfo ridInfo = {0};
    T_DjiFlightControllerGeneralInfo generalInfo = {0};

    // 设置RID信息（Remote ID，远程识别信息）
    // 在某些国家/地区，必须报告正确的RID信息才能使用PSDK控制飞行器
    ridInfo.latitude = 22.542812;
    ridInfo.longitude = 113.958902;
    ridInfo.altitude = 0;

    // 初始化飞行控制器模块
    // 参考API: DjiFlightController_Init
    returnCode = DjiFlightController_Init(ridInfo);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Init flight controller failed, errno = 0x%08llX", returnCode);
        return NULL;
    }

    // 初始化数据订阅模块
    // 参考API: DjiFcSubscription_Init
    returnCode = DjiFcSubscription_Init();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Init data subscription module failed, error code:0x%08llX", returnCode);
        return NULL;
    }

    // 订阅四元数数据，用于获取飞行器姿态信息
    // 频率：50Hz
    // 参考API: DjiFcSubscription_SubscribeTopic
    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION,
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                                  NULL);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic flight status failed, error code:0x%08llX", returnCode);
        return NULL;
    }

    // 订阅GPS位置数据，用于获取飞行器位置信息
    // 频率：5Hz
    // 参考API: DjiFcSubscription_SubscribeTopic
    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION,
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ,
                                                  NULL);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic gps failed, error code:0x%08llX", returnCode);
        return NULL;
    }

    // 订阅融合高度数据，用于获取飞行器相对于起飞点的高度
    // 频率：10Hz
    // 参考API: DjiFcSubscription_SubscribeTopic
    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_HEIGHT_FUSION,
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ,
                                                  NULL);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic altitude failed, error code:0x%08llX", returnCode);
        return NULL;
    }

    // 订阅视觉定位数据，用于获取飞行器在视觉坐标系中的位置
    // 频率：10Hz
    // 参考API: DjiFcSubscription_SubscribeTopic
    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_POSITION_VO,
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ,
                                                  NULL);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic altitude failed, error code:0x%08llX", returnCode);
        return NULL;
    }

    // 订阅控制设备数据，用于获取当前控制飞行器的设备信息
    // 频率：5Hz
    // 参考API: DjiFcSubscription_SubscribeTopic
    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_CONTROL_DEVICE,
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ,
                                                  NULL);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic altitude failed, error code:0x%08llX", returnCode);
        return NULL;
    }

    // 等待数据订阅初始化完成
    osalHandler->TaskSleepMs(1000);

    // 从配置文件更新飞行参数
    // 包括返航高度、失控动作、RTK开关、避障开关等
    returnCode = DjiUser_FlightControlUpdateConfig();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Update config failed, error code:0x%08llX", returnCode);
    }

    // 获取飞行器基本信息，包括序列号
    // 参考API: DjiFlightController_GetGeneralInfo
    returnCode = DjiFlightController_GetGeneralInfo(&generalInfo);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get general info failed, error code:0x%08llX", returnCode);
    }
    USER_LOG_INFO("Get aircraft serial number is: %s", generalInfo.serialNum);

    // 注册摇杆控制权限事件回调函数
    // 当控制权限发生变化时（如从遥控器切换到PSDK控制），会触发该回调
    // 参考API: DjiFlightController_RegJoystickCtrlAuthorityEventCallback
    returnCode = DjiFlightController_RegJoystickCtrlAuthorityEventCallback(
        DjiUser_FlightCtrlJoystickCtrlAuthSwitchEventCb);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS && returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT) {
        USER_LOG_ERROR("Register joystick control authority event callback failed, errno = 0x%08llX", returnCode);
        return NULL;
    }

    // 标记命令飞行任务已启动，状态显示任务可以开始工作
    isCommandFlyingTaskStart = true;

    // 主循环，执行飞行命令
    while (true) {
        // 计数器递增，用于自动清除长时间未更新的飞行命令
        s_inputFlag++;
        if (s_inputFlag > 25) {
            // 如果超过25个周期没有新的输入，则清除当前命令，按照下面的频率算约为 0.5s
            // 这是一个安全机制，防止飞行器持续执行旧命令
            s_flyingCommand.x = 0;
            s_flyingCommand.y = 0;
            s_flyingCommand.z = 0;
            s_flyingCommand.yaw = 0;
            s_inputFlag = 0;
        }

        // 执行速度和偏航角速度控制
        // 将用户输入的命令转换为飞行控制指令并发送给飞行器
        DjiUser_FlightControllerVelocityAndYawRateCtrl(s_flyingCommand);

        // 控制频率为 DJI_TEST_COMMAND_FLYING_CTRL_FREQ（50Hz）
        osalHandler->TaskSleepMs(1000 / DJI_TEST_COMMAND_FLYING_CTRL_FREQ);
    }
}

/**
 * @brief 飞行控制器状态显示任务函数
 * 
 * 该函数是状态显示任务的主体，负责：
 * 1. 等待命令飞行任务启动
 * 2. 定期调用OpenCV显示函数，展示飞行器的实时状态
 * 
 * @param arg 任务参数（未使用）
 * @return 任务返回值（未使用）
 */
static void *DjiUser_FlightControllerStatusDisplayTask(void *arg)
{
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();

    while (1) {
        // 等待命令飞行任务启动
        if (isCommandFlyingTaskStart == false) {
            continue;
        }
#ifdef OPEN_CV_INSTALLED
        // 使用OpenCV显示飞行器状态信息
        DjiUser_ShowFlightStatusByOpenCV();
#endif
        // 控制显示频率为DJI_TEST_COMMAND_FLYING_CTRL_FREQ（50Hz）
        osalHandler->TaskSleepMs(1000 / DJI_TEST_COMMAND_FLYING_CTRL_FREQ);
    }
}

/**
 * @brief 使用OpenCV显示飞行器状态信息
 * 
 * 该函数使用OpenCV创建一个窗口，显示飞行器的实时状态信息，包括：
 * 1. 飞行器姿态（横滚角、俯仰角、偏航角）
 * 2. 飞行器位置（世界坐标、GPS坐标）
 * 3. 飞行器高度
 * 4. 电池信息
 * 5. 飞行配置信息（返航高度、RTK状态、避障状态等）
 * 6. 键盘控制指令提示
 * 
 * 注意：此函数仅在定义了OPEN_CV_INSTALLED宏时有效
 */
static void DjiUser_ShowFlightStatusByOpenCV(void)
{
#ifdef OPEN_CV_INSTALLED
    E_DjiFlightControllerGoHomeAltitude goHomeAltitude = 0;
    T_DjiVector3f aircraftAngles = {0};
    T_DjiFcSubscriptionAltitudeOfHomePoint altitudeOfHomePoint = {0};
    E_DjiFlightControllerRtkPositionEnableStatus rtkPositionEnableStatus;
    E_DjiFlightControllerRCLostAction rcLostAction = DJI_FLIGHT_CONTROLLER_RC_LOST_ACTION_HOVER;
    E_DjiFlightControllerObstacleAvoidanceEnableStatus downwardsVisEnable;
    E_DjiFlightControllerObstacleAvoidanceEnableStatus upwardsVisEnable;
    E_DjiFlightControllerObstacleAvoidanceEnableStatus horizontalVisEnable;
//    E_DjiFlightControllerObstacleAvoidanceEnableStatus upwardsRadarEnable;
//    E_DjiFlightControllerObstacleAvoidanceEnableStatus horizontalRadarEnable;
    T_DjiFcSubscriptionControlDevice controlDevice;
    T_DjiFcSubscriptionPositionVO positionVo;
    T_DjiAircraftInfoBaseInfo aircraftInfoBaseInfo;
    T_DjiReturnCode returnCode;

    // 获取飞行器基本信息，用于判断飞行器型号
    returnCode = DjiAircraftInfo_GetBaseInfo(&aircraftInfoBaseInfo);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("get aircraft base info error");
    }

    // 创建一个黑色背景的图像，用于显示状态信息
    Mat img(480, 1000, CV_8UC1, cv::Scalar(0));

    // 获取最新的飞行状态信息
    // 根据飞行器型号获取不同的信息
    if (aircraftInfoBaseInfo.aircraftSeries != DJI_AIRCRAFT_SERIES_M300) {
        // 获取遥控器失控动作（悬停、降落或返航）
        DjiFlightController_GetRCLostAction(&rcLostAction);
    }
    // 获取返航高度
    DjiFlightController_GetGoHomeAltitude(&s_goHomeAltitude);
    // 获取RTK定位使能状态
    DjiFlightController_GetRtkPositionEnableStatus(&rtkPositionEnableStatus);
    // 获取向下视觉避障使能状态
    DjiFlightController_GetDownwardsVisualObstacleAvoidanceEnableStatus(&downwardsVisEnable);
    // 获取向上视觉避障使能状态
    DjiFlightController_GetUpwardsVisualObstacleAvoidanceEnableStatus(&upwardsVisEnable);
    // 获取水平视觉避障使能状态
    DjiFlightController_GetHorizontalVisualObstacleAvoidanceEnableStatus(&horizontalVisEnable);

    // 获取控制设备信息
    controlDevice = DjiUser_FlightControlGetValueOfControlDevice();
    // 获取飞行器姿态角（横滚角、俯仰角、偏航角）
    aircraftAngles = DjiUser_FlightControlGetValueOfQuaternion();
    // 获取GPS位置
    s_gpsPosition = DjiUser_FlightControlGetValueOfGpsPosition();
    // 获取相对高度
    altitudeOfHomePoint = DjiUser_FlightControlGetValueOfRelativeHeight();
    // 获取视觉定位信息
    positionVo = DjiUser_FlightControlGetValueOfPositionVo();

    // 每20个周期更新一次电池信息（降低更新频率，减少系统负担）
    if (s_statusDisplayTaskCnt++ % 20 == 0) {
        singleBatteryInfo1 = DjiUser_FlightControlGetValueOfBattery1();
        singleBatteryInfo2 = DjiUser_FlightControlGetValueOfBattery2();
    }

    // 在图像上显示飞行状态信息
    // 1. 显示标题
    cv::putText(img, "Status: ", cv::Point(30, 20), FONT_HERSHEY_SIMPLEX, 0.6,
                cv::Scalar(255, 0, 0));

    // 2. 显示飞行器姿态和位置信息
    cv::putText(img, "Roll: " + cv::format("%.4f", aircraftAngles.y), cv::Point(50, 50), FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(200, 0, 0));
    cv::putText(img, "Pitch: " + cv::format("%.4f", aircraftAngles.x), cv::Point(50, 80), FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(200, 0, 0));
    cv::putText(img, "Yaw: " + cv::format("%.4f", aircraftAngles.z), cv::Point(50, 110), FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(200, 0, 0));
    cv::putText(img, "WorldX: " + cv::format("%.4f", positionVo.x), cv::Point(50, 140), FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(200, 0, 0));
    cv::putText(img, "WorldY: " + cv::format("%.4f", positionVo.y), cv::Point(50, 170), FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(200, 0, 0));
    cv::putText(img, "WorldZ: " + cv::format("%.4f", altitudeOfHomePoint), cv::Point(50, 200), FONT_HERSHEY_SIMPLEX,
                0.5, cv::Scalar(200, 0, 0));
    cv::putText(img, "Latitude: " + cv::format("%.4f", (dji_f64_t) s_gpsPosition.y / 10000000), cv::Point(50, 230),
                FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 0, 0));
    cv::putText(img, "Longitude: " + cv::format("%.4f", (dji_f64_t) s_gpsPosition.x / 10000000), cv::Point(50, 260),
                FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 0, 0));
    cv::putText(img, "Battery1: " + cv::format("%d%%", singleBatteryInfo1.batteryCapacityPercent), cv::Point(50, 290),
                FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 0, 0));
    cv::putText(img, "Battery2: " + cv::format("%d%%", singleBatteryInfo2.batteryCapacityPercent), cv::Point(50, 320),
                FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 0, 0));

    // 3. 显示飞行配置信息
    cv::putText(img, "Config: ", cv::Point(300, 20), FONT_HERSHEY_SIMPLEX, 0.6,
                cv::Scalar(255, 0, 0));
    cv::putText(img, "-> RcLostAction(Sync APP): " + cv::format("%d  (0-hover 1-landing 2-gohome)", rcLostAction),
                cv::Point(320, 50),
                FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 0, 0));
    cv::putText(img, "-> GoHomeAltitude(Sync APP): " + cv::format("%d", s_goHomeAltitude), cv::Point(320, 80),
                FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 0, 0));
    cv::putText(img, "-> RTK-Enable(Sync APP): " + cv::format("%d", rtkPositionEnableStatus), cv::Point(320, 110),
                FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 0, 0));
    cv::putText(img, "-> HomePointLatitude: " + cv::format("%.4f", s_homeLocation.latitude), cv::Point(320, 140),
                FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 0, 0));
    cv::putText(img, "-> HomePointLongitude: " + cv::format("%.4f", s_homeLocation.longitude), cv::Point(320, 170),
                FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 0, 0));
    cv::putText(img, "-> FlyingSpeed: " + cv::format("%.2f", s_flyingSpeed), cv::Point(320, 200),
                FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 0, 0));
    cv::putText(img, "-> downwardsVisEnable(Sync APP): " + cv::format("%d", downwardsVisEnable), cv::Point(320, 230),
                FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 0, 0));
    cv::putText(img, "-> upwardsVisEnable(Sync APP): " + cv::format("%d", upwardsVisEnable), cv::Point(320, 260),
                FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 0, 0));
    cv::putText(img, "-> horizontalVisEnable(Sync APP): " + cv::format("%d", horizontalVisEnable), cv::Point(320, 290),
                FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 0, 0));
    cv::putText(img, "-> ControlDevice: " + cv::format("%d", controlDevice.deviceStatus), cv::Point(320, 320),
                FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 0, 0));

    // 4. 显示键盘控制指令提示
    cv::putText(img,
                "[Q]-Up    [W]-Front  [E]-Down   [R]-TakeOff  [T]-CancelLanding  [Y]-CancelGoHome  [I]-ArrestFly  [O]-CancelArrestFly  [P]-EmgStopMotor",
                cv::Point(30, 400), FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(128, 0, 0));
    cv::putText(img,
                "[A]-Left   [S]-Near   [D]-Right   [F]-ForceLand   [G]-Landing   [H]-GoHome  [J]-UpdateConfig  [K]-Brake  [L]-CancelBrakeI",
                cv::Point(30, 430), FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(128, 0, 0));
    cv::putText(img,
                "[Z]-Yaw-  [X]-RefreshHomePoint   [C]-Yaw+  [V]-ConfirmLanding   [B]-TurnOn  [N]-TurnOff  [M]-ObtainCtrlAuth",
                cv::Point(30, 460), FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(128, 0, 0));

    // 5. 显示图像窗口
    cv::imshow("Payload SDK Command Flying Data Observation Window", img);
    cv::waitKey(1);
#endif
}

/**
 * @brief 飞行控制器速度和偏航角速度控制函数
 * 
 * 该函数设置飞行控制模式并执行摇杆控制动作，实现飞行器的速度和偏航角速度控制。
 * 控制模式包括：
 * - 水平控制模式：速度控制
 * - 垂直控制模式：速度控制
 * - 偏航控制模式：角速度控制
 * - 水平坐标系：机体坐标系
 * - 稳定控制模式：启用
 * 
 * @param command 摇杆控制命令，包含x、y、z方向的速度和偏航角速度
 * @return void
 */
static void DjiUser_FlightControllerVelocityAndYawRateCtrl(T_DjiFlightControllerJoystickCommand command)
{
    // 设置摇杆控制模式
    // DJI_FLIGHT_CONTROLLER_HORIZONTAL_VELOCITY_CONTROL_MODE: 水平方向使用速度控制
    // DJI_FLIGHT_CONTROLLER_VERTICAL_VELOCITY_CONTROL_MODE: 垂直方向使用速度控制
    // DJI_FLIGHT_CONTROLLER_YAW_ANGLE_RATE_CONTROL_MODE: 偏航使用角速度控制
    // DJI_FLIGHT_CONTROLLER_HORIZONTAL_BODY_COORDINATE: 使用机体坐标系
    // DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE: 启用稳定控制模式
    T_DjiFlightControllerJoystickMode joystickMode = {
        DJI_FLIGHT_CONTROLLER_HORIZONTAL_VELOCITY_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_VERTICAL_VELOCITY_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_YAW_ANGLE_RATE_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_HORIZONTAL_BODY_COORDINATE,
        DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE,
    };
    T_DjiReturnCode returnCode;

    // 设置摇杆控制模式
    // 参考API: DjiFlightController_SetJoystickMode
    DjiFlightController_SetJoystickMode(joystickMode);

    // 打印摇杆命令信息
    USER_LOG_DEBUG("Joystick command: %.2f %.2f %.2f", command.x, command.y, command.z);
    
    // 执行摇杆控制动作
    // 参考API: DjiFlightController_ExecuteJoystickAction
    returnCode = DjiFlightController_ExecuteJoystickAction(command);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Execute joystick command failed, errno = 0x%08llX", returnCode);
        return;
    }
}

/**
 * @brief 扫描键盘输入函数
 * 
 * 该函数通过修改终端设置，实现非阻塞式的键盘输入扫描。
 * 它将终端设置为非规范模式（不需要按回车键确认输入），
 * 并设置为非阻塞模式（不等待用户输入）。
 * 
 * @return int 返回键盘输入的ASCII码
 */
static int DjiUser_ScanKeyboardInput(void)
{
    int input;
    struct termios new_settings;
    struct termios stored_settings;

    // 获取当前终端设置
    tcgetattr(0, &stored_settings);
    new_settings = stored_settings;
    
    // 设置为非规范模式（不需要按回车键确认输入）
    new_settings.c_lflag &= (~ICANON);
    // 设置为非阻塞模式（不等待用户输入）
    new_settings.c_cc[VTIME] = 0;
    
    tcgetattr(0, &stored_settings);
    // 设置最小输入字符数为1
    new_settings.c_cc[VMIN] = 1;
    // 应用新的终端设置
    tcsetattr(0, TCSANOW, &new_settings);

    // 获取键盘输入
    input = getchar();
    
    // 恢复原来的终端设置
    tcsetattr(0, TCSANOW, &stored_settings);

    return input;
}

/**
 * @brief 飞行控制器摇杆控制权限切换事件回调函数，函数本身其实只是一个日志函数
 * 
 * 该函数处理飞行控制器摇杆控制权限变化的事件，包括：
 * - MSDK（移动端SDK）获取或释放控制权限
 * - 内部系统获取或释放控制权限
 * - OSDK（机载SDK，即PSDK）获取或释放控制权限
 * - 遥控器失控导致的控制权限重置
 * - 遥控器模式切换导致的控制权限重置
 * - 低电量返航或降落导致的控制权限重置
 * - 其他各种导致控制权限变化的事件
 * 
 * @param eventData 控制权限事件信息，包含事件类型和当前控制权限状态
 * @return T_DjiReturnCode 返回处理结果
 */
static T_DjiReturnCode
DjiUser_FlightCtrlJoystickCtrlAuthSwitchEventCb(T_DjiFlightControllerJoystickCtrlAuthorityEventInfo eventData)
{
    // 根据事件类型处理不同的控制权限变化事件
    switch (eventData.joystickCtrlAuthoritySwitchEvent) {
        case DJI_FLIGHT_CONTROLLER_MSDK_GET_JOYSTICK_CTRL_AUTH_EVENT: {
            // 移动端SDK（MSDK）获取或释放控制权限
            if (eventData.curJoystickCtrlAuthority == DJI_FLIGHT_CONTROLLER_JOYSTICK_CTRL_AUTHORITY_MSDK) {
                // MSDK获取控制权限
                USER_LOG_INFO("[Event] Msdk request to obtain joystick ctrl authority\r\n");
            } else {
                // MSDK释放控制权限
                USER_LOG_INFO("[Event] Msdk request to release joystick ctrl authority\r\n");
            }
            break;
        }
        case DJI_FLIGHT_CONTROLLER_INTERNAL_GET_JOYSTICK_CTRL_AUTH_EVENT: {
            // 内部系统获取或释放控制权限
            if (eventData.curJoystickCtrlAuthority == DJI_FLIGHT_CONTROLLER_JOYSTICK_CTRL_AUTHORITY_INTERNAL) {
                // 内部系统获取控制权限
                USER_LOG_INFO("[Event] Internal request to obtain joystick ctrl authority\r\n");
            } else {
                // 内部系统释放控制权限
                USER_LOG_INFO("[Event] Internal request to release joystick ctrl authority\r\n");
            }
            break;
        }
        case DJI_FLIGHT_CONTROLLER_OSDK_GET_JOYSTICK_CTRL_AUTH_EVENT: {
            // 机载SDK（OSDK，即PSDK）获取或释放控制权限
            if (eventData.curJoystickCtrlAuthority == DJI_FLIGHT_CONTROLLER_JOYSTICK_CTRL_AUTHORITY_OSDK) {
                // OSDK获取控制权限
                USER_LOG_INFO("[Event] Request to obtain joystick ctrl authority\r\n");
            } else {
                // OSDK释放控制权限
                USER_LOG_INFO("[Event] Request to release joystick ctrl authority\r\n");
            }
            break;
        }
        case DJI_FLIGHT_CONTROLLER_RC_LOST_GET_JOYSTICK_CTRL_AUTH_EVENT :
            // 遥控器失控导致控制权限重置为遥控器
            USER_LOG_INFO("[Event] Current joystick ctrl authority is reset to rc due to rc lost\r\n");
            break;
        case DJI_FLIGHT_CONTROLLER_RC_NOT_P_MODE_RESET_JOYSTICK_CTRL_AUTH_EVENT :
            // 遥控器不在P模式导致控制权限重置为遥控器
            // 注意：只有在P模式（定位模式）下，SDK才能获取控制权限
            USER_LOG_INFO("[Event] Current joystick ctrl authority is reset to rc for rc is not in P mode\r\n");
            break;
        case DJI_FLIGHT_CONTROLLER_RC_SWITCH_MODE_GET_JOYSTICK_CTRL_AUTH_EVENT :
            // 遥控器切换模式导致控制权限重置为遥控器
            USER_LOG_INFO("[Event] Current joystick ctrl authority is reset to rc due to rc switching mode\r\n");
            break;
        case DJI_FLIGHT_CONTROLLER_RC_PAUSE_GET_JOYSTICK_CTRL_AUTH_EVENT :
            // 遥控器暂停导致控制权限重置为遥控器
            USER_LOG_INFO("[Event] Current joystick ctrl authority is reset to rc due to rc pausing\r\n");
            break;
        case DJI_FLIGHT_CONTROLLER_RC_REQUEST_GO_HOME_GET_JOYSTICK_CTRL_AUTH_EVENT :
            // 遥控器请求返航导致控制权限重置为遥控器
            USER_LOG_INFO("[Event] Current joystick ctrl authority is reset to rc due to rc request for return\r\n");
            break;
        case DJI_FLIGHT_CONTROLLER_LOW_BATTERY_GO_HOME_RESET_JOYSTICK_CTRL_AUTH_EVENT :
            // 低电量返航导致控制权限重置为遥控器
            USER_LOG_INFO("[Event] Current joystick ctrl authority is reset to rc for low battery return\r\n");
            break;
        case DJI_FLIGHT_CONTROLLER_LOW_BATTERY_LANDING_RESET_JOYSTICK_CTRL_AUTH_EVENT :
            // 低电量降落导致控制权限重置为遥控器
            USER_LOG_INFO("[Event] Current joystick ctrl authority is reset to rc for low battery land\r\n");
            break;
        case DJI_FLIGHT_CONTROLLER_OSDK_LOST_GET_JOYSTICK_CTRL_AUTH_EVENT:
            // SDK连接丢失导致控制权限重置为遥控器
            USER_LOG_INFO("[Event] Current joystick ctrl authority is reset to rc due to sdk lost\r\n");
            break;
        case DJI_FLIGHT_CONTROLLER_NERA_FLIGHT_BOUNDARY_RESET_JOYSTICK_CTRL_AUTH_EVENT :
            // 接近飞行边界导致控制权限重置为遥控器
            USER_LOG_INFO("[Event] Current joystick ctrl authority is reset to rc due to near boundary\r\n");
            break;
        default:
            // 未知的控制权限事件
            USER_LOG_INFO("[Event] Unknown joystick ctrl authority event\r\n");
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiVector3f DjiUser_FlightControlGetValueOfQuaternion(void)
{
    T_DjiReturnCode djiStat;
    T_DjiFcSubscriptionQuaternion quaternion = {0};
    T_DjiDataTimestamp quaternionTimestamp = {0};
    dji_f64_t pitch, yaw, roll;
    T_DjiVector3f vector3F;

    djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION,
                                                      (uint8_t *) &quaternion,
                                                      sizeof(T_DjiFcSubscriptionQuaternion),
                                                      &quaternionTimestamp);

    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get value of topic quaternion error, error code: 0x%08X", djiStat);
    } else {
        USER_LOG_DEBUG("Timestamp: millisecond %u microsecond %u.", quaternionTimestamp.millisecond,
                       quaternionTimestamp.microsecond);
        USER_LOG_DEBUG("Quaternion: %f %f %f %f.", quaternion.q0, quaternion.q1, quaternion.q2, quaternion.q3);
    }

    pitch = (dji_f64_t) asinf(-2 * quaternion.q1 * quaternion.q3 + 2 * quaternion.q0 * quaternion.q2) * 57.3;
    roll = (dji_f64_t) atan2f(2 * quaternion.q2 * quaternion.q3 + 2 * quaternion.q0 * quaternion.q1,
                              -2 * quaternion.q1 * quaternion.q1 - 2 * quaternion.q2 * quaternion.q2 + 1) * 57.3;
    yaw = (dji_f64_t) atan2f(2 * quaternion.q1 * quaternion.q2 + 2 * quaternion.q0 * quaternion.q3,
                             -2 * quaternion.q2 * quaternion.q2 - 2 * quaternion.q3 * quaternion.q3 + 1) *
          57.3;

    vector3F.x = pitch;
    vector3F.y = roll;
    vector3F.z = yaw;

    return vector3F;
}

static T_DjiFcSubscriptionGpsPosition DjiUser_FlightControlGetValueOfGpsPosition(void)
{
    T_DjiReturnCode djiStat;
    T_DjiDataTimestamp timestamp = {0};
    T_DjiFcSubscriptionGpsPosition gpsPosition;

    djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION,
                                                      (uint8_t *) &gpsPosition,
                                                      sizeof(T_DjiFcSubscriptionGpsPosition),
                                                      &timestamp);

    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get value of topic quaternion error, error code: 0x%08X", djiStat);
    } else {
        USER_LOG_DEBUG("Timestamp: millisecond %u microsecond %u.", timestamp.millisecond,
                       timestamp.microsecond);
    }

    return gpsPosition;
}

static T_DjiFcSubscriptionAltitudeOfHomePoint DjiUser_FlightControlGetValueOfRelativeHeight(void)
{
    T_DjiReturnCode djiStat;
    T_DjiDataTimestamp timestamp = {0};
    T_DjiFcSubscriptionAltitudeOfHomePoint altitudeOfHomePoint;

    djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_HEIGHT_FUSION,
                                                      (uint8_t *) &altitudeOfHomePoint,
                                                      sizeof(T_DjiFcSubscriptionAltitudeOfHomePoint),
                                                      &timestamp);

    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get value of topic quaternion error, error code: 0x%08X", djiStat);
    } else {
        USER_LOG_DEBUG("Timestamp: millisecond %u microsecond %u.", timestamp.millisecond,
                       timestamp.microsecond);
    }

    return altitudeOfHomePoint;
}

static T_DjiFcSubscriptionPositionVO DjiUser_FlightControlGetValueOfPositionVo(void)
{
    T_DjiReturnCode djiStat;
    T_DjiDataTimestamp timestamp = {0};
    T_DjiFcSubscriptionPositionVO positionVo;

    djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_POSITION_VO,
                                                      (uint8_t *) &positionVo,
                                                      sizeof(T_DjiFcSubscriptionPositionVO),
                                                      &timestamp);

    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get value of topic quaternion error, error code: 0x%08X", djiStat);
    } else {
        USER_LOG_DEBUG("Timestamp: millisecond %u microsecond %u.", timestamp.millisecond,
                       timestamp.microsecond);
    }

    return positionVo;
}

static T_DjiFcSubscriptionControlDevice DjiUser_FlightControlGetValueOfControlDevice(void)
{
    T_DjiReturnCode djiStat;
    T_DjiDataTimestamp timestamp = {0};
    T_DjiFcSubscriptionControlDevice controlDevice;

    djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_CONTROL_DEVICE,
                                                      (uint8_t *) &controlDevice,
                                                      sizeof(T_DjiFcSubscriptionControlDevice),
                                                      &timestamp);

    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get value of topic quaternion error, error code: 0x%08X", djiStat);
    } else {
        USER_LOG_DEBUG("Timestamp: millisecond %u microsecond %u.", timestamp.millisecond,
                       timestamp.microsecond);
    }

    return controlDevice;
}

static T_DjiFcSubscriptionSingleBatteryInfo DjiUser_FlightControlGetValueOfBattery1(void)
{
    T_DjiReturnCode djiStat;
    T_DjiDataTimestamp timestamp = {0};
    T_DjiFcSubscriptionSingleBatteryInfo singleBatteryInfo;

    djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_BATTERY_SINGLE_INFO_INDEX1,
                                                      (uint8_t *) &singleBatteryInfo,
                                                      sizeof(T_DjiFcSubscriptionSingleBatteryInfo),
                                                      &timestamp);

    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get value of topic battery1 error, error code: 0x%08X", djiStat);
    } else {
        USER_LOG_DEBUG("Timestamp: millisecond %u microsecond %u.", timestamp.millisecond,
                       timestamp.microsecond);
    }

    return singleBatteryInfo;
}

static T_DjiFcSubscriptionSingleBatteryInfo DjiUser_FlightControlGetValueOfBattery2(void)
{
    T_DjiReturnCode djiStat;
    T_DjiDataTimestamp timestamp = {0};
    T_DjiFcSubscriptionSingleBatteryInfo singleBatteryInfo;

    djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_BATTERY_SINGLE_INFO_INDEX2,
                                                      (uint8_t *) &singleBatteryInfo,
                                                      sizeof(T_DjiFcSubscriptionSingleBatteryInfo),
                                                      &timestamp);

    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get value of topic battery2 error, error code: 0x%08X", djiStat);
    } else {
        USER_LOG_DEBUG("Timestamp: millisecond %u microsecond %u.", timestamp.millisecond,
                       timestamp.microsecond);
    }

    return singleBatteryInfo;
}

static T_DjiReturnCode DjiUser_FlightControlUpdateConfig(void)
{
    T_DjiReturnCode returnCode;
    char curFileDirPath[DJI_TEST_COMMAND_FLYING_CONFIG_DIR_PATH_LEN_MAX];
    char tempFileDirPath[DJI_TEST_COMMAND_FLYING_CONFIG_DIR_PATH_LEN_MAX];
    uint32_t fileSize = 0;
    uint32_t readRealSize = 0;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    uint8_t *jsonData = nullptr;
    cJSON *jsonRoot = nullptr;
    cJSON *jsonItem = nullptr;
    cJSON *jsonValue = nullptr;
    T_DjiAircraftInfoBaseInfo aircraftInfoBaseInfo;

    returnCode = DjiAircraftInfo_GetBaseInfo(&aircraftInfoBaseInfo);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("get aircraft base info error");
    }

#ifdef SYSTEM_ARCH_LINUX
    returnCode = DjiUserUtil_GetCurrentFileDirPath(__FILE__, DJI_TEST_COMMAND_FLYING_CONFIG_DIR_PATH_LEN_MAX,
                                                   curFileDirPath);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get file current path error, stat = 0x%08llX", returnCode);
        return returnCode;
    }

    snprintf(tempFileDirPath, DJI_TEST_COMMAND_FLYING_CONFIG_DIR_PATH_LEN_MAX, "%s/config/flying_config.json",
             curFileDirPath);

    returnCode = UtilFile_GetFileSizeByPath(tempFileDirPath, &fileSize);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get file size by path failed, stat = 0x%08llX", returnCode);
        return returnCode;
    }

    USER_LOG_DEBUG("Get config json file size is %d", fileSize);

    jsonData = static_cast<uint8_t *>(osalHandler->Malloc(fileSize + 1));
    if (jsonData == nullptr) {
        USER_LOG_ERROR("Malloc failed.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }

    memset(jsonData, 0, fileSize);

    UtilFile_GetFileDataByPath(tempFileDirPath, 0, fileSize, jsonData, &readRealSize);

    jsonData[readRealSize] = '\0';

    jsonRoot = cJSON_Parse((char *) jsonData);
    if (jsonRoot == nullptr) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    jsonItem = cJSON_GetObjectItem(jsonRoot, "go_home_altitude");
    if (jsonItem != nullptr) {
        jsonValue = cJSON_GetObjectItem(jsonItem, "value");
        if (jsonValue != nullptr) {
            USER_LOG_INFO("Get go home altitude is [%.2f]", jsonValue->valuedouble);
            s_goHomeAltitude = (uint16_t) jsonValue->valuedouble;
            returnCode = DjiFlightController_SetGoHomeAltitude((uint16_t) jsonValue->valuedouble);
            if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                USER_LOG_ERROR("Set go home altitude failed, errno = 0x%08llX", returnCode);
                return returnCode;
            }
        }
    }

    if (aircraftInfoBaseInfo.aircraftSeries != DJI_AIRCRAFT_SERIES_M300) {
        jsonItem = cJSON_GetObjectItem(jsonRoot, "rc_lost_action");
        if (jsonItem != nullptr) {
            jsonValue = cJSON_GetObjectItem(jsonItem, "value");
            if (jsonValue != nullptr) {
                USER_LOG_INFO("Get rc lost action is [%s]", jsonValue->valuestring);
                strcpy(s_rcLostActionString, jsonValue->valuestring);
                if (strcmp(jsonValue->valuestring, "go_home") == 0) {
                    returnCode = DjiFlightController_SetRCLostAction(DJI_FLIGHT_CONTROLLER_RC_LOST_ACTION_GOHOME);
                    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                        USER_LOG_ERROR("Set rc lost action failed, errno = 0x%08llX", returnCode);
                        return returnCode;
                    }
                } else if (strcmp(jsonValue->valuestring, "hover") == 0) {
                    returnCode = DjiFlightController_SetRCLostAction(DJI_FLIGHT_CONTROLLER_RC_LOST_ACTION_HOVER);
                    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                        USER_LOG_ERROR("Set rc lost action failed, errno = 0x%08llX", returnCode);
                        return returnCode;
                    }
                } else if (strcmp(jsonValue->valuestring, "landing") == 0) {
                    returnCode = DjiFlightController_SetRCLostAction(DJI_FLIGHT_CONTROLLER_RC_LOST_ACTION_LANDING);
                    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                        USER_LOG_ERROR("Set rc lost action failed, errno = 0x%08llX", returnCode);
                        return returnCode;
                    }
                } else {
                    USER_LOG_ERROR("Invalid value: %s", jsonValue->valuestring);
                }
            }
        }
    }

    jsonItem = cJSON_GetObjectItem(jsonRoot, "flying_speed");
    if (jsonItem != nullptr) {
        jsonValue = cJSON_GetObjectItem(jsonItem, "value");
        if (jsonValue != nullptr) {
            USER_LOG_INFO("Get flying speed is [%.2f]", jsonValue->valuedouble);
            s_flyingSpeed = jsonValue->valuedouble;
        }
    }

    jsonItem = cJSON_GetObjectItem(jsonRoot, "rtk_enable");
    if (jsonItem != nullptr) {
        jsonValue = cJSON_GetObjectItem(jsonItem, "value");
        if (jsonValue != nullptr) {
            USER_LOG_INFO("Get rtk enable is [%s]", jsonValue->valuestring);
            if (strcmp(jsonValue->valuestring, "true") == 0) {
                returnCode = DjiFlightController_SetRtkPositionEnableStatus(DJI_FLIGHT_CONTROLLER_ENABLE_RTK_POSITION);
                if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    USER_LOG_ERROR("Set rtk enable failed, errno = 0x%08llX", returnCode);
                    return returnCode;
                }
            } else if (strcmp(jsonValue->valuestring, "false") == 0) {
                returnCode = DjiFlightController_SetRtkPositionEnableStatus(DJI_FLIGHT_CONTROLLER_DISABLE_RTK_POSITION);
                if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    USER_LOG_ERROR("Set rtk enable failed, errno = 0x%08llX", returnCode);
                    return returnCode;
                }
            } else {
                USER_LOG_ERROR("Invalid value: %s", jsonValue->valuestring);
            }
        }
    }

    jsonItem = cJSON_GetObjectItem(jsonRoot, "home_point_latitude");
    if (jsonItem != nullptr) {
        jsonValue = cJSON_GetObjectItem(jsonItem, "value");
        if (jsonValue != nullptr) {
            USER_LOG_INFO("Get home_point_latitude  is [%.2f]", jsonValue->valuedouble);
            s_homeLocation.latitude = jsonValue->valuedouble;
        }
    }
    jsonItem = cJSON_GetObjectItem(jsonRoot, "home_point_longitude");
    if (jsonItem != nullptr) {
        jsonValue = cJSON_GetObjectItem(jsonItem, "value");
        if (jsonValue != nullptr) {
            USER_LOG_INFO("Get home_point_longitude is [%.2f]", jsonValue->valuedouble);
            s_homeLocation.longitude = jsonValue->valuedouble;
        }
    }

    if (isFirstUpdateConfig == false) {
        USER_LOG_INFO("Using current aircraft location, not use config home location.");
        s_gpsPosition = DjiUser_FlightControlGetValueOfGpsPosition();
        s_homeLocation.latitude = (dji_f64_t) s_gpsPosition.y / 10000000;
        s_homeLocation.longitude = (dji_f64_t) s_gpsPosition.x / 10000000;

        returnCode = DjiFlightController_SetHomeLocationUsingCurrentAircraftLocation();
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("Set home location failed, errno = 0x%08llX", returnCode);
        }
        isFirstUpdateConfig = true;
    } else {
        T_DjiFlightControllerHomeLocation homeLocation;
        homeLocation.latitude = s_homeLocation.latitude * DJI_PI / 180;
        homeLocation.longitude = s_homeLocation.longitude * DJI_PI / 180;

        returnCode = DjiFlightController_SetHomeLocationUsingGPSCoordinates(homeLocation);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("Set home location failed, errno = 0x%08llX", returnCode);
            return returnCode;
        }
    }

    jsonItem = cJSON_GetObjectItem(jsonRoot, "HorizontalVisualObstacleAvoidanceEnable");
    if (jsonItem != nullptr) {
        jsonValue = cJSON_GetObjectItem(jsonItem, "value");
        if (jsonValue != nullptr) {
            USER_LOG_INFO("Get HorizontalVisualObstacleAvoidanceEnable is [%s]", jsonValue->valuestring);
            if (strcmp(jsonValue->valuestring, "true") == 0) {
                returnCode = DjiFlightController_SetHorizontalVisualObstacleAvoidanceEnableStatus(
                    DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE);
                if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    USER_LOG_ERROR("Set HorizontalVisualObstacleAvoidanceEnable failed, errno = 0x%08llX", returnCode);
                    return returnCode;
                }
            } else if (strcmp(jsonValue->valuestring, "false") == 0) {
                returnCode = DjiFlightController_SetHorizontalVisualObstacleAvoidanceEnableStatus(
                    DJI_FLIGHT_CONTROLLER_DISABLE_OBSTACLE_AVOIDANCE);
                if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    USER_LOG_ERROR("Set HorizontalVisualObstacleAvoidanceEnable failed, errno = 0x%08llX", returnCode);
                    return returnCode;
                }
            } else {
                USER_LOG_ERROR("Invalid value: %s", jsonValue->valuestring);
            }
        }
    }

//    jsonItem = cJSON_GetObjectItem(jsonRoot, "HorizontalRadarObstacleAvoidanceEnable");
//    if (jsonItem != nullptr) {
//        if (aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M300_RTK ||
//            aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M30 ||
//            aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M30T ||
//            aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M350_RTK) {
//            jsonValue = cJSON_GetObjectItem(jsonItem, "value");
//            if (jsonValue != nullptr) {
//                USER_LOG_INFO("Get HorizontalRadarObstacleAvoidanceEnable is [%s]", jsonValue->valuestring);
//                if (strcmp(jsonValue->valuestring, "true") == 0) {
//                    returnCode = DjiFlightController_SetHorizontalRadarObstacleAvoidanceEnableStatus(
//                        DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE);
//                    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
//                        USER_LOG_ERROR("Set HorizontalRadarObstacleAvoidanceEnable failed, errno = 0x%08llX",
//                                       returnCode);
//                        return returnCode;
//                    }
//                } else if (strcmp(jsonValue->valuestring, "false") == 0) {
//                    returnCode = DjiFlightController_SetHorizontalRadarObstacleAvoidanceEnableStatus(
//                        DJI_FLIGHT_CONTROLLER_DISABLE_OBSTACLE_AVOIDANCE);
//                    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
//                        USER_LOG_ERROR("Set HorizontalRadarObstacleAvoidanceEnable failed, errno = 0x%08llX",
//                                       returnCode);
//                        return returnCode;
//                    }
//                } else {
//                    USER_LOG_ERROR("Invalid value: %s", jsonValue->valuestring);
//                }
//            }
//        }
//    }

    jsonItem = cJSON_GetObjectItem(jsonRoot, "UpwardsVisualObstacleAvoidanceEnable");
    if (jsonItem != nullptr) {
        jsonValue = cJSON_GetObjectItem(jsonItem, "value");
        if (jsonValue != nullptr) {
            USER_LOG_INFO("Get UpwardsVisualObstacleAvoidanceEnable is [%s]", jsonValue->valuestring);
            if (strcmp(jsonValue->valuestring, "true") == 0) {
                returnCode = DjiFlightController_SetUpwardsVisualObstacleAvoidanceEnableStatus(
                    DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE);
                if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    USER_LOG_ERROR("Set UpwardsVisualObstacleAvoidanceEnable failed, errno = 0x%08llX", returnCode);
                    return returnCode;
                }
            } else if (strcmp(jsonValue->valuestring, "false") == 0) {
                returnCode = DjiFlightController_SetUpwardsVisualObstacleAvoidanceEnableStatus(
                    DJI_FLIGHT_CONTROLLER_DISABLE_OBSTACLE_AVOIDANCE);
                if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    USER_LOG_ERROR("Set UpwardsVisualObstacleAvoidanceEnable failed, errno = 0x%08llX", returnCode);
                    return returnCode;
                }
            } else {
                USER_LOG_ERROR("Invalid value: %s", jsonValue->valuestring);
            }
        }
    }

//    jsonItem = cJSON_GetObjectItem(jsonRoot, "UpwardsRadarObstacleAvoidanceEnable");
//    if (jsonItem != nullptr) {
//        if (aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M300_RTK ||
//            aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M30 ||
//            aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M30T ||
//            aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M350_RTK) {
//            jsonValue = cJSON_GetObjectItem(jsonItem, "value");
//            if (jsonValue != nullptr) {
//                USER_LOG_INFO("Get UpwardsRadarObstacleAvoidanceEnable is [%s]", jsonValue->valuestring);
//                if (strcmp(jsonValue->valuestring, "true") == 0) {
//                    returnCode = DjiFlightController_SetUpwardsRadarObstacleAvoidanceEnableStatus(
//                        DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE);
//                    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
//                        USER_LOG_ERROR("Set UpwardsRadarObstacleAvoidanceEnable failed, errno = 0x%08llX", returnCode);
//                        return returnCode;
//                    }
//                } else if (strcmp(jsonValue->valuestring, "false") == 0) {
//                    returnCode = DjiFlightController_SetUpwardsRadarObstacleAvoidanceEnableStatus(
//                        DJI_FLIGHT_CONTROLLER_DISABLE_OBSTACLE_AVOIDANCE);
//                    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
//                        USER_LOG_ERROR("Set UpwardsRadarObstacleAvoidanceEnable failed, errno = 0x%08llX", returnCode);
//                        return returnCode;
//                    }
//                } else {
//                    USER_LOG_ERROR("Invalid value: %s", jsonValue->valuestring);
//                }
//            }
//        }
//    }

    jsonItem = cJSON_GetObjectItem(jsonRoot, "DownwardsVisualObstacleAvoidanceEnable");
    if (jsonItem != nullptr) {
        jsonValue = cJSON_GetObjectItem(jsonItem, "value");
        if (jsonValue != nullptr) {
            USER_LOG_INFO("Get DownwardsVisualObstacleAvoidanceEnable is [%s]", jsonValue->valuestring);
            if (strcmp(jsonValue->valuestring, "true") == 0) {
                returnCode = DjiFlightController_SetDownwardsVisualObstacleAvoidanceEnableStatus(
                    DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE);
                if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    USER_LOG_ERROR("Set DownwardsVisualObstacleAvoidanceEnable failed, errno = 0x%08llX", returnCode);
                    return returnCode;
                }
            } else if (strcmp(jsonValue->valuestring, "false") == 0) {
                returnCode = DjiFlightController_SetDownwardsVisualObstacleAvoidanceEnableStatus(
                    DJI_FLIGHT_CONTROLLER_DISABLE_OBSTACLE_AVOIDANCE);
                if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    USER_LOG_ERROR("Set DownwardsVisualObstacleAvoidanceEnable failed, errno = 0x%08llX", returnCode);
                    return returnCode;
                }
            } else {
                USER_LOG_ERROR("Invalid value: %s", jsonValue->valuestring);
            }
        }
    }

    osalHandler->Free(jsonData);

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
#endif
}

/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/
