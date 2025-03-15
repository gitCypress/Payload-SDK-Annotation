/**
 ********************************************************************
 * @file    dji_flight_controller.h
 * @brief   This is the header file for "dji_flight_controller.c", defining the structure and
 * (exported) function prototypes.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DJI_FLIGHT_CONTROLLER_H
#define DJI_FLIGHT_CONTROLLER_H

/* Includes ------------------------------------------------------------------*/
#include "dji_typedef.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Exported constants --------------------------------------------------------*/
#define EMERGENCY_STOP_MOTOR_MSG_MAX_LENGTH 10 /*!<  Max length of emergency stop motor message. */

/* Exported types ------------------------------------------------------------*/
typedef uint16_t E_DjiFlightControllerGoHomeAltitude; /*!< Unit:meter, range 20~500 */

/**
 * @brief The aircraft's actions when RC is lost.
 */
typedef enum {
    DJI_FLIGHT_CONTROLLER_RC_LOST_ACTION_HOVER = 0,  /*!< Aircraft will execute hover action when RC is lost. */
    DJI_FLIGHT_CONTROLLER_RC_LOST_ACTION_LANDING = 1,  /*!< Aircraft will execute land action when RC is lost. */
    DJI_FLIGHT_CONTROLLER_RC_LOST_ACTION_GOHOME = 2,  /*!< Aircraft will execute go-home action when RC is lost. */
} E_DjiFlightControllerRCLostAction;

/**
 * @brief Enable/Disable RTK position enum
 */
typedef enum {
    DJI_FLIGHT_CONTROLLER_DISABLE_RTK_POSITION = 0, /*!< 0: The aircraft will use GPS data instead of RTK data to execute
                                                     * actions which requires location information(waypoint, go home...)
                                                     */
    DJI_FLIGHT_CONTROLLER_ENABLE_RTK_POSITION = 1, /*!< 1:The aircraft will use RTK data instead of GPS data to execute
                                                    * actions which requires location information(waypoint, go home...)*/
} E_DjiFlightControllerRtkPositionEnableStatus;

/**
 * @brief Enable/Disable obstacle sensing enum
 */
typedef enum {
    DJI_FLIGHT_CONTROLLER_DISABLE_OBSTACLE_AVOIDANCE = 0, /*!< 0: The aircraft will not perform obstacle sensing in
                                                           * the specified direction */
    DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE = 1, /*!< 0: The aircraft will perform obstacle sensing in the
                                                          * specified direction */
} E_DjiFlightControllerObstacleAvoidanceEnableStatus;

/**
 * @brief Enable/Disable emergency stop motor function enum
 * @note Enable emergency-stop-motor function is very dangerous in the air. It will make the aircraft crash!!!
 */
typedef enum {
    DJI_FLIGHT_CONTROLLER_ENABLE_EMERGENCY_STOP_MOTOR = 0x01, /*!< Execute emergency-stop-motor action */
} E_DjiFlightControllerEmergencyStopMotor;

/**
 * @brief Obtain/Release joystick control permission command enum
 * @note You have obtained joystick control permission successfully before using joystick.
 */
typedef enum {
    DJI_FLIGHT_CONTROLLER_RELEASE_JOYSTICK_CTRL_AUTHORITY = 0, /*!< Obtain joystick permission */
    DJI_FLIGHT_CONTROLLER_OBTAIN_JOYSTICK_CTRL_AUTHORITY = 1, /*!< Release joystick permission */
} E_DjiFlightControllerJoystickCtrlAuthorityAction;

/**
 * @brief The aircraft's joystick control permission owner enum
 */
typedef enum {
    DJI_FLIGHT_CONTROLLER_JOYSTICK_CTRL_AUTHORITY_RC = 0,  /*!< RC could control aircraft with joystick. */
    DJI_FLIGHT_CONTROLLER_JOYSTICK_CTRL_AUTHORITY_MSDK = 1, /*!< MSDK could control aircraft with joystick. */
    DJI_FLIGHT_CONTROLLER_JOYSTICK_CTRL_AUTHORITY_INTERNAL = 2, /*!< Special Internal modules could control aircraft
                                                                 * with joystick. */
    DJI_FLIGHT_CONTROLLER_JOYSTICK_CTRL_AUTHORITY_OSDK = 4, /*!< PSDK could control aircraft with joystick. */
} E_DjiFlightControllerJoystickCtrlAuthority;

/**
 * @brief The aircraft's joystick control permission switch reason enum
 */
typedef enum {
    DJI_FLIGHT_CONTROLLER_MSDK_GET_JOYSTICK_CTRL_AUTH_EVENT = 1, /*!< MSDK gets the joystick control permission. */
    DJI_FLIGHT_CONTROLLER_INTERNAL_GET_JOYSTICK_CTRL_AUTH_EVENT = 2,  /*!< A specific internal modules gets the joystick control permission. */
    DJI_FLIGHT_CONTROLLER_OSDK_GET_JOYSTICK_CTRL_AUTH_EVENT = 3, /*!< PSDK gets the joystick control permission. */
    DJI_FLIGHT_CONTROLLER_RC_LOST_GET_JOYSTICK_CTRL_AUTH_EVENT = 4, /*!< Reset the joystick control permission to RC when executing RC lost action */
    DJI_FLIGHT_CONTROLLER_RC_NOT_P_MODE_RESET_JOYSTICK_CTRL_AUTH_EVENT = 5,  /*!< Reset the joystick control permission to RC when RC is not in P mode */
    DJI_FLIGHT_CONTROLLER_RC_SWITCH_MODE_GET_JOYSTICK_CTRL_AUTH_EVENT = 6,  /*!< Set the joystick control permission to RC when RC switches control mode(T/APS) */
    DJI_FLIGHT_CONTROLLER_RC_PAUSE_GET_JOYSTICK_CTRL_AUTH_EVENT = 7, /*!< Reset the joystick control permission to RC when RC pauses */
    DJI_FLIGHT_CONTROLLER_RC_REQUEST_GO_HOME_GET_JOYSTICK_CTRL_AUTH_EVENT = 8, /*!< Reset the joystick control permission to RC when RC requests to go home*/
    DJI_FLIGHT_CONTROLLER_LOW_BATTERY_GO_HOME_RESET_JOYSTICK_CTRL_AUTH_EVENT = 9, /*!< Reset the joystick control permission to RC when aircraft is executing low-battery-go-home*/
    DJI_FLIGHT_CONTROLLER_LOW_BATTERY_LANDING_RESET_JOYSTICK_CTRL_AUTH_EVENT = 10, /*!< Reset the joystick control permission to RC when aircraft is executing low-battery-landing*/
    DJI_FLIGHT_CONTROLLER_OSDK_LOST_GET_JOYSTICK_CTRL_AUTH_EVENT = 11, /*!< Reset the joystick control permission to RC when PSDK is lost*/
    DJI_FLIGHT_CONTROLLER_NERA_FLIGHT_BOUNDARY_RESET_JOYSTICK_CTRL_AUTH_EVENT = 12, /*!< Reset the joystick control permission to RC when aircraft is near boundary.*/
} E_DjiFlightControllerJoystickCtrlAuthoritySwitchEvent;

/**
 * @brief The aircraft's joystick control permission switch event info enum
 */
typedef struct {
    E_DjiFlightControllerJoystickCtrlAuthority curJoystickCtrlAuthority; /*!< The aircraft's joystick control permission owner */
    E_DjiFlightControllerJoystickCtrlAuthoritySwitchEvent joystickCtrlAuthoritySwitchEvent; /*!< The aircraft's joystick control permission switch reason */
} T_DjiFlightControllerJoystickCtrlAuthorityEventInfo;

/**
 * @brief Prototype of callback function used to get joystick control permission switch event info.
 * @return Execution result.
 */
typedef T_DjiReturnCode (*JoystickCtrlAuthorityEventCbFunc)(
    T_DjiFlightControllerJoystickCtrlAuthorityEventInfo eventData);

/**
 * @brief Prototype of callback function used to get the trigger FTS event.
 * @return Execution result.
 */
typedef T_DjiReturnCode (*TriggerFtsEventCallback)(void);

/**
 * @brief Horizon control mode enum in joystick mode
 * @note Need to be referenced to either the ground or body frame by E_DjiFlightControllerHorizontalCoordinate setting Limit: -150deg/s to 150.0 deg/s
 */
typedef enum {
    /**
     * @brief 姿态角控制模式
     * @details 在此模式下，控制飞行器的横滚和俯仰角度
     * @note 需要通过E_DjiFlightControllerHorizontalCoordinate设置参考坐标系
     * @note 限制范围: -35度至35度
     * @note 在机体坐标系(FRU)下，正/负横滚角分别对应飞行器向右/左倾斜
     * @note 在机体坐标系(FRU)下，正/负俯仰角分别对应飞行器向前/后倾斜
     */
    DJI_FLIGHT_CONTROLLER_HORIZONTAL_ANGLE_CONTROL_MODE = 0,
    /**
     * @brief 速度控制模式
     * @details 在此模式下，控制飞行器的水平速度
     * @note 需要通过E_DjiFlightControllerHorizontalCoordinate设置参考坐标系
     * @note 限制范围: -30m/s至30m/s
     * @note 只有当GPS信号良好或高级感知系统正常工作时，才能使用此模式
     * @note 在地面坐标系(NEU)下，正/负x值分别对应飞行器向北/南飞行
     * @note 在地面坐标系(NEU)下，正/负y值分别对应飞行器向东/西飞行
     * @note 在机体坐标系(FRU)下，正/负x值分别对应飞行器向前/后飞行
     * @note 在机体坐标系(FRU)下，正/负y值分别对应飞行器向右/左飞行
     */
    DJI_FLIGHT_CONTROLLER_HORIZONTAL_VELOCITY_CONTROL_MODE = 1,
    /**
     * @brief 位置控制模式
     * @details 在此模式下，控制飞行器的水平位置偏移量
     * @note 需要通过E_DjiFlightControllerHorizontalCoordinate设置参考坐标系
     * @note 只有当GPS信号良好(health_flag >= 3)时，才能使用此模式
     * @note 当位置命令的模不为0时，飞行器将以指定速度向前飞行
     * @note 当位置命令的模为0时，飞行器将悬停在指定位置
     * @note 在地面坐标系(NEU)下，正/负x值分别对应飞行器向北/南移动
     * @note 在地面坐标系(NEU)下，正/负y值分别对应飞行器向东/西移动
     * @note 在机体坐标系(FRU)下，正/负x值分别对应飞行器向前/后移动
     * @note 在机体坐标系(FRU)下，正/负y值分别对应飞行器向右/左移动
     */
    DJI_FLIGHT_CONTROLLER_HORIZONTAL_POSITION_CONTROL_MODE = 2,
    /**
     * @brief 角速度控制模式
     * @details 在此模式下，控制飞行器姿态的变化率
     * @note 需要通过E_DjiFlightControllerHorizontalCoordinate设置参考坐标系
     * @note 限制范围: -150度/秒至150度/秒
     * @note 在机体坐标系(FRU)下，正/负横滚角速度分别对应飞行器向右/左旋转
     * @note 在机体坐标系(FRU)下，正/负俯仰角速度分别对应飞行器向前/后旋转
     */
    DJI_FLIGHT_CONTROLLER_HORIZONTAL_ANGULAR_RATE_CONTROL_MODE = 3
} E_DjiFlightControllerHorizontalControlMode;

/**
 * @brief Vertical control mode enum in joystick mode
 * @note We don not recommend using VERTICAL_POSITION control mode indoors when the aircraft's flying height exceeds 3 meters.
 * This is because barometer can be inaccurate indoors, and the vertical controller may fail to keep the height of the aircraft.
 */
typedef enum {
    /**
     * @brief 垂直速度控制模式
     * @details 在此模式下，控制飞行器的垂直速度，向上为正
     * @note 限制范围: -5m/s至5m/s
     * @note 正/负值分别对应飞行器向上/下飞行
     */
    DJI_FLIGHT_CONTROLLER_VERTICAL_VELOCITY_CONTROL_MODE = 0,

    /**
     * @brief 垂直位置控制模式
     * @details 在此模式下，控制飞行器的垂直高度
     * @note 限制范围: 0m至120m
     * @note 此高度为相对于起飞点的绝对高度
     * @note 当飞行高度超过3米时，不建议在室内使用此模式，因为气压计在室内可能不准确
     */
    DJI_FLIGHT_CONTROLLER_VERTICAL_POSITION_CONTROL_MODE = 1,

    /**
     * @brief 油门控制模式
     * @details 在此模式下，直接控制飞行器的推力
     * @note 范围: 0%至100%
     * @note 此模式下需要飞行员具有较高的操作技巧，不建议新手使用
     */
    DJI_FLIGHT_CONTROLLER_VERTICAL_THRUST_CONTROL_MODE = 2,
} E_DjiFlightControllerVerticalControlMode;

/**
 * @brief Yaw control mode enum in joystick mode
 */
typedef enum {
    /**
     * @brief 偏航角控制模式
     * @details 在此模式下，控制飞行器的偏航角
     * @note 偏航角是参考地面坐标系的。在此控制模式下，自动驾驶仪强制使用地面坐标系
     * @note 在地面坐标系下，0度表示飞行器机头指向正北
     * @note 正/负角度分别表示飞行器相对于北方顺时针/逆时针旋转
     * @note 范围: -180度至180度
     */
    DJI_FLIGHT_CONTROLLER_YAW_ANGLE_CONTROL_MODE = 0x00,

    /**
     * @brief 偏航角速度控制模式
     * @details 在此模式下，控制飞行器偏航的角速度
     * @note 与YAW_ANGLE使用相同的参考坐标系
     * @note 限制范围: -150度/秒至150度/秒
     * @note 正/负值分别表示飞行器顺时针/逆时针旋转
     */
    DJI_FLIGHT_CONTROLLER_YAW_ANGLE_RATE_CONTROL_MODE = 1
} E_DjiFlightControllerYawControlMode;

/**
 * @brief Horizontal coordinate enum in joystick mode
 */
typedef enum {
    /**
     * @brief 地面坐标系
     * @details 设置地面坐标系作为水平坐标系(NEU)
     * @note NEU: 北-东-上坐标系
     * @note 在此坐标系中，X轴指向地球正北方向，Y轴指向正东方向，Z轴垂直向上
     * @note 此坐标系也称为世界坐标系或本地水平坐标系
     * @note 当使用位置或速度控制模式时，地面坐标系更适合执行定点悬停或沿固定方向飞行等任务
     */
    DJI_FLIGHT_CONTROLLER_HORIZONTAL_GROUND_COORDINATE = 0, /*!< Set the x-y of ground frame as the horizontal frame (NEU) */
    /**
     * @brief 机体坐标系
     * @details 设置机体坐标系作为水平坐标系(FRU)
     * @note FRU: 前-右-上坐标系
     * @note 在此坐标系中，X轴指向飞行器机头方向，Y轴指向机头右侧，Z轴垂直向上
     * @note 机体坐标系以飞行器重心为原点，随飞行器姿态变化而变化
     * @note 当需要相对于飞行器当前朝向进行控制时，机体坐标系更为直观
     */
    DJI_FLIGHT_CONTROLLER_HORIZONTAL_BODY_COORDINATE = 1 /*!< Set the x-y of body frame as the horizontal frame (FRU) */
} E_DjiFlightControllerHorizontalCoordinate;

/*!
 * @brief Stable mode enum in joystick mode
 * @note Only works in horizontal velocity control mode. In velocity stable mode, aircraft will brake and hover at one position once
 * the input command is zero. In velocity non-stable mode, aircraft will follow the velocity command and not hover when the command is zero.
 * That means aircraft will drift with the wind.
 */
typedef enum {
    /**
     * @brief 禁用稳定模式
     * @details 在此模式下，当速度命令为零时，飞行器不会悬停
     * @note 仅在水平速度控制模式(DJI_FLIGHT_CONTROLLER_HORIZONTAL_VELOCITY_CONTROL_MODE)下有效
     * @note 在非稳定模式下，飞行器将遵循速度命令，当命令为零时不会悬停
     * @note 这意味着飞行器可能会随风漂移
     * @note 此模式适用于需要飞行器随环境漂移的场景，如跟随移动物体
     */
    DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_DISABLE = 0, /*!< Disable the stable mode */
    /**
     * @brief 启用稳定模式
     * @details 在此模式下，当速度命令为零时，飞行器将刹车并悬停在一个位置
     * @note 仅在水平速度控制模式(DJI_FLIGHT_CONTROLLER_HORIZONTAL_VELOCITY_CONTROL_MODE)下有效
     * @note 在稳定模式下，当输入命令为零时，飞行器将刹车并悬停在一个位置
     * @note 此模式适用于需要飞行器保持位置的场景，如定点拍摄
     */
    DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE = 1   /*!< Enable the stable mode */
} E_DjiFlightControllerStableControlMode;

typedef enum {
    DJI_FLIGHT_CONTROLLER_ENABLE_RC_LOST_ACTION = 0,
    DJI_FLIGHT_CONTROLLER_DISABLE_RC_LOST_ACTION = 1,
} E_DjiFlightControllerRCLostActionEnableStatus;

/**
 * @brief Joystick mode.
 * @note You need to set joystick mode first before start to send joystick command to aircraft.
 */
typedef struct {
    /**
     * @brief 水平控制模式
     * @note 可用选项:
     * - DJI_FLIGHT_CONTROLLER_HORIZONTAL_ANGLE_CONTROL_MODE: 姿态角控制模式，控制飞行器的横滚和俯仰角
     *   (需要通过horizontalCoordinate设置参考坐标系，范围: -35度至35度)
     * - DJI_FLIGHT_CONTROLLER_HORIZONTAL_VELOCITY_CONTROL_MODE: 速度控制模式，控制飞行器的水平速度
     *   (需要通过horizontalCoordinate设置参考坐标系，范围: -30m/s至30m/s)
     * - DJI_FLIGHT_CONTROLLER_HORIZONTAL_POSITION_CONTROL_MODE: 位置控制模式，控制飞行器的水平位置
     *   (需要通过horizontalCoordinate设置参考坐标系)
     * - DJI_FLIGHT_CONTROLLER_HORIZONTAL_ANGULAR_RATE_CONTROL_MODE: 角速度控制模式，控制飞行器的旋转角速度
     *   (需要通过horizontalCoordinate设置参考坐标系，范围: -150度/秒至150度/秒)
     * @note 只有当GPS信号良好(health_flag >= 3)时，才能使用水平位置控制模式；
     *       只有当GPS信号良好或高级感知系统正常工作时，才能使用水平速度控制模式
     */
    E_DjiFlightControllerHorizontalControlMode horizontalControlMode; 

    /**
     * @brief 垂直控制模式
     * @note 可用选项:
     * - DJI_FLIGHT_CONTROLLER_VERTICAL_VELOCITY_CONTROL_MODE: 速度控制模式，控制飞行器的垂直速度
     *   (向上为正，范围: -5m/s至5m/s)
     * - DJI_FLIGHT_CONTROLLER_VERTICAL_POSITION_CONTROL_MODE: 位置控制模式，控制飞行器的垂直高度
     *   (相对于起飞点的绝对高度，范围: 0m至120m)
     * - DJI_FLIGHT_CONTROLLER_VERTICAL_THRUST_CONTROL_MODE: 油门控制模式，直接控制飞行器的推力
     *   (范围: 0%至100%)
     * @note 当飞行高度超过3米时，不建议在室内使用垂直位置控制模式，因为气压计在室内可能不准确
     */
    E_DjiFlightControllerVerticalControlMode verticalControlMode; 

    /**
     * @brief 偏航控制模式
     * @note 可用选项:
     * - DJI_FLIGHT_CONTROLLER_YAW_ANGLE_CONTROL_MODE: 角度控制模式，控制飞行器的偏航角
     *   (参考地面坐标系，在此控制模式下，自动驾驶仪强制使用地面坐标系)
     * - DJI_FLIGHT_CONTROLLER_YAW_ANGLE_RATE_CONTROL_MODE: 角速度控制模式，控制飞行器偏航的角速度
     *   (与YAW_ANGLE使用相同的参考坐标系，范围: -150度/秒至150度/秒)
     */
    E_DjiFlightControllerYawControlMode yawControlMode; 

    /**
     * @brief 水平坐标系
     * @note 可用选项:
     * - DJI_FLIGHT_CONTROLLER_HORIZONTAL_GROUND_COORDINATE: 地面坐标系(NEU)，以地面为参考，北-东-上作为坐标轴
     * - DJI_FLIGHT_CONTROLLER_HORIZONTAL_BODY_COORDINATE: 机体坐标系(FRU)，以飞行器为参考，前-右-上作为坐标轴
     */
    E_DjiFlightControllerHorizontalCoordinate horizontalCoordinate; 

    /**
     * @brief 稳定控制模式
     * @note 可用选项:
     * - DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_DISABLE: 禁用稳定模式，飞行器将随风漂移
     * - DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE: 启用稳定模式，当输入命令为零时，飞行器将刹车并悬停在一个位置
     * @note 仅在水平速度控制模式(DJI_FLIGHT_CONTROLLER_HORIZONTAL_VELOCITY_CONTROL_MODE)下有效
     */
    E_DjiFlightControllerStableControlMode stableControlMode; 
} T_DjiFlightControllerJoystickMode;

#pragma pack(1)
typedef struct {
    dji_f32_t x;   /*!< Controls the x axis.*/
    dji_f32_t y;   /*!< Controls the y axis.*/
    dji_f32_t z;   /*!< Controls the z axis, setting upward as positive. */
    dji_f32_t yaw; /*!< Yaw position/velocity control w.r.t. the ground frame.*/
} T_DjiFlightControllerJoystickCommand;// pack(1)

typedef struct {
    dji_f64_t latitude;  /*!< unit: rad */
    dji_f64_t longitude; /*!< unit: rad */
} T_DjiFlightControllerHomeLocation; // pack(1)

typedef struct {
    char serialNum[32];
} T_DjiFlightControllerGeneralInfo;

typedef struct {
    dji_f64_t latitude;  /*!< unit: rad */
    dji_f64_t longitude; /*!< unit: rad */
    uint16_t altitude;
} T_DjiFlightControllerRidInfo;

#pragma pack()

/* Exported functions --------------------------------------------------------*/
/**
 * @brief Initialise flight controller module
 * @param ridInfo: Must report the correct RID information before using PSDK to control the aircraft.
 * @return Execution result.
 */
T_DjiReturnCode DjiFlightController_Init(T_DjiFlightControllerRidInfo ridInfo);

/**
 * @brief DeInitialise flight controller module.
 * @return Execution result.
 */
T_DjiReturnCode DjiFlightController_DeInit(void);

/**
 * @brief Enable/Disable RTK position function.
 * @details Enabling RTK means that RTK data will be used instead of GPS during flight.
 * @param rtkEnableStatus: refer to "E_DjiFlightControllerRtkPositionEnableStatus", inheriting from Pilot.
 * @return Execution result.
 */
T_DjiReturnCode
DjiFlightController_SetRtkPositionEnableStatus(E_DjiFlightControllerRtkPositionEnableStatus rtkEnableStatus);

/**
 * @brief Get RTK enable status.
 * @note Enabling RTK means that RTK data will be used during intelligent flight.
 * @param rtkEnableStatus: refer to "E_DjiFlightControllerRtkPositionEnableStatus", inheriting from Pilot.
 * @return Execution result.
 */
T_DjiReturnCode
DjiFlightController_GetRtkPositionEnableStatus(E_DjiFlightControllerRtkPositionEnableStatus *rtkEnableStatus);

/**
 * @brief Set RC lost action.
 * @note Valid when RC and PSDK are both lost. It only supports M30.
 * @param rcLostAction: actions when RC is lost.(hover/landing/go home).It inherits from Pilot's param.
 * @return Execution result.
 */
T_DjiReturnCode DjiFlightController_SetRCLostAction(E_DjiFlightControllerRCLostAction rcLostAction);

/**
 * @brief Get RC lost action(hover/landing/gohome).
 * @note Valid when RC and PSDK are both lost. It only supports M30.
 * @param rcLostAction: see reference of E_DjiFlightControllerRCLostAction.It inherits from Pilot's param.
 * @return Execution result.
 */
T_DjiReturnCode DjiFlightController_GetRCLostAction(E_DjiFlightControllerRCLostAction *rcLostAction);

/**
 * @brief Enable/Disable horizontal visual(forwards,backwards,left,right) obstacle sensing.
 * @note For detailed parameters of obstacle sensing, it is recommended to read the official user manual in
 * https://www.dji.com.
 * @param horizontalObstacleAvoidanceEnableStatus: see reference of E_DjiFlightControllerObstacleAvoidanceEnableStatus.
 * It inherits from Pilot's param.
 * @return Execution result.
 */
T_DjiReturnCode DjiFlightController_SetHorizontalVisualObstacleAvoidanceEnableStatus(
    E_DjiFlightControllerObstacleAvoidanceEnableStatus horizontalObstacleAvoidanceEnableStatus);

/**
 * @brief Get the switch status of horizontal visual(forwards,backwards,left,right) obstacle sensing.
 * @note For detailed parameters of obstacle sensing, it is recommended to read the official user manual in
 * https://www.dji.com.
 * @param horizontalObstacleAvoidanceEnableStatus: see reference of E_DjiFlightControllerObstacleAvoidanceEnableStatus.
 * It inherits from Pilot's param.
 * @return Execution result.
 */
T_DjiReturnCode DjiFlightController_GetHorizontalVisualObstacleAvoidanceEnableStatus(
    E_DjiFlightControllerObstacleAvoidanceEnableStatus *horizontalObstacleAvoidanceEnableStatus);

/**
 * @brief Enable/Disable horizontal radar obstacle sensing.
 * @note It will be valid only if you install CSM radar successfully.For detailed parameters of obstacle sensing,
 * it is recommended to read the official user manual in https://www.dji.com.
 * @param horizontalObstacleAvoidanceEnableStatus: see reference of E_DjiFlightControllerObstacleAvoidanceEnableStatus.
 * It inherits from Pilot's param.
 * @return Execution result.
 */
T_DjiReturnCode DjiFlightController_SetHorizontalRadarObstacleAvoidanceEnableStatus(
    E_DjiFlightControllerObstacleAvoidanceEnableStatus horizontalObstacleAvoidanceEnableStatus);

/**
 * @brief Get the switch status of horizontal radar obstacle sensing.
 * @note It will be valid only if you install CSM radar successfully.For detailed parameters of obstacle sensing,
 * it is recommended to read the official user manual in https://www.dji.com/uk/matrice-300/downloads.
 * @param horizontalObstacleAvoidanceEnableStatus: see reference of E_DjiFlightControllerObstacleAvoidanceEnableStatus.
 * It inherits from Pilot's param.
 * @return Execution result.
 */
T_DjiReturnCode DjiFlightController_GetHorizontalRadarObstacleAvoidanceEnableStatus(
    E_DjiFlightControllerObstacleAvoidanceEnableStatus *horizontalObstacleAvoidanceEnableStatus);

/**
 * @brief Enable/Disable upwards visual obstacle sensing.
 * @note For detailed parameters of obstacle sensing, it is recommended to read the official user manual in
 * https://www.dji.com.
 * @param upwardsObstacleAvoidanceEnableStatus: see reference of E_DjiFlightControllerObstacleAvoidanceEnableStatus.
 * It inherits from Pilot's param.
 * @return Execution result.
 */
T_DjiReturnCode DjiFlightController_SetUpwardsVisualObstacleAvoidanceEnableStatus(
    E_DjiFlightControllerObstacleAvoidanceEnableStatus upwardsObstacleAvoidanceEnableStatus);

/**
 * @brief Get the switch status of upwards visual obstacle sensing.
 * @note For detailed parameters of obstacle sensing, it is recommended to read the official user manual in
 * https://www.dji.com.
 * @param upwardsObstacleAvoidanceEnableStatus: see reference of E_DjiFlightControllerObstacleAvoidanceEnableStatus.
 * It inherits from Pilot's param.
 * @return Execution result.
 */
T_DjiReturnCode DjiFlightController_GetUpwardsVisualObstacleAvoidanceEnableStatus(
    E_DjiFlightControllerObstacleAvoidanceEnableStatus *upwardsObstacleAvoidanceEnableStatus);

/**
 * @brief Enable/Disable upwards radar obstacle sensing.
 * @note It will be valid only if you install CSM radar successfully.For detailed parameters of obstacle sensing,
 * it is recommended to read the official user manual in https://www.dji.com.
 * @param upwardsObstacleAvoidanceEnableStatus: see reference of E_DjiFlightControllerObstacleAvoidanceEnableStatus.
 * It inherits from Pilot's param.
 * @return Execution result.
 */
T_DjiReturnCode DjiFlightController_SetUpwardsRadarObstacleAvoidanceEnableStatus(
    E_DjiFlightControllerObstacleAvoidanceEnableStatus upwardsObstacleAvoidanceEnableStatus);

/**
 * @brief Get the switch status of upwards radar obstacle sensing.
 * @note For detailed parameters of obstacle sensing, it is recommended to read the official user manual in
 * https://www.dji.com.
 * @param upwardsObstacleAvoidanceEnableStatus: see reference of E_DjiFlightControllerObstacleAvoidanceEnableStatus.
 * It inherits from Pilot's param.
 * @return Execution result.
 */
T_DjiReturnCode DjiFlightController_GetUpwardsRadarObstacleAvoidanceEnableStatus(
    E_DjiFlightControllerObstacleAvoidanceEnableStatus *upwardsObstacleAvoidanceEnableStatus);

/**
 * @brief Enable/Disable downwards visual obstacle sensing.
 * @note For detailed parameters of obstacle sensing, it is recommended to read the official user manual in
 * https://www.dji.com.
 * @param downwardsObstacleAvoidanceEnableStatus: see reference of E_DjiFlightControllerObstacleAvoidanceEnableStatus.
 * It inherits from Pilot's param.
 * @return Execution result.
 */
T_DjiReturnCode DjiFlightController_SetDownwardsVisualObstacleAvoidanceEnableStatus(
    E_DjiFlightControllerObstacleAvoidanceEnableStatus downwardsObstacleAvoidanceEnableStatus);

/**
 * @brief Get the switch status of downwards visual obstacle sensing.
 * @note For detailed parameters of obstacle sensing, it is recommended to read the official user manual in
 * https://www.dji.com.
 * @param downwardsObstacleAvoidanceEnableStatus: see reference of E_DjiFlightControllerObstacleAvoidanceEnableStatus.
 * It inherits from Pilot's param.
 * @return Execution result.
 */
T_DjiReturnCode DjiFlightController_GetDownwardsVisualObstacleAvoidanceEnableStatus(
    E_DjiFlightControllerObstacleAvoidanceEnableStatus *downwardsObstacleAvoidanceEnableStatus);

/**
 * @brief Arrest flying means emergency braking
 * @note When the aircraft is on the ground, it will stop motors and display "hms description" on App. when the aircraft is
 * in the air, it will continue flying and display "hms description" on App only.
 * If you use this interface, you need to use "DjiFlightController_CancelArrestFlying" to quit arrest-flying status, then
 * then the aircraft can fly again.
 * @return Execution result.
 */
T_DjiReturnCode DjiFlightController_ArrestFlying(void);

/**
 * @brief Quit status of arrest-flying.
 * @note The aircraft need to quit status of arrest-flying to continue flying after arresting flying.
 * @return Execution result.
 */
T_DjiReturnCode DjiFlightController_CancelArrestFlying(void);

/**
 * @brief Turn on motors when the aircraft is on the ground.
 * @return Execution result.
 */
T_DjiReturnCode DjiFlightController_TurnOnMotors(void);

/**
 * @brief Turn off motors when the aircraft is on the ground.
 * @return Execution result.
 */
T_DjiReturnCode DjiFlightController_TurnOffMotors(void);

/**
 * @brief Emergency stop motor in any case.
 * @note If you want to turn on motor after emergency stopping motor, you need to use the interface to send disable
 * command to quit lock-motor status.
 * @param cmd: see reference of E_DjiFlightControllerEmergencyStopMotor
 * @param debugMsg:inject debug message to flight control FW for logging, size limit: 10 bytes
 * @return Execution result.
 */
T_DjiReturnCode DjiFlightController_EmergencyStopMotor(E_DjiFlightControllerEmergencyStopMotor cmd,
                                                       char debugMsg[EMERGENCY_STOP_MOTOR_MSG_MAX_LENGTH]);
/**
 * @brief Request taking off action when the aircraft is on the ground.
 * @return Execution result.
 */
T_DjiReturnCode DjiFlightController_StartTakeoff(void);

/**
 * @brief Request landing action when the aircraft is in the air.
 * @return Execution result.
 */
T_DjiReturnCode DjiFlightController_StartLanding(void);

/**
 * @brief Request cancelling landing action when the aircraft is landing
 * @return Execution result.
 */
T_DjiReturnCode DjiFlightController_CancelLanding(void);

/**
 * @brief Confirm the landing when the aircraft is 0.7 m above the ground.
 * @note When the clearance between the aircraft and the ground is less than 0.7m, the aircraft will pause landing and
 * wait for user's confirmation. This API is for confirm landing. If the ground is not suitable for landing, user
 * must use RC to control it landing manually or force landing.
 * @return Execution result.
 */
T_DjiReturnCode DjiFlightController_StartConfirmLanding(void);

/**
 * @brief Force landing in any case.
 * @note This API will ignore the smart landing function. When using this API, it will landing directly (would not stop
 * at 0.7m and wait user's command). Attention: it may make the aircraft crash!!!
 * @return Execution result.
 */
T_DjiReturnCode DjiFlightController_StartForceLanding(void);

/**
 * @brief Set customized GPS(not RTK) home location.
 * @note Set customized home location failed reason may as follows:
 * 1. The distance between new home location and last home location is larger than MAX_FLY_RADIUS(20 km).
 * 2. Record initial home location failed after start aircraft.
 * @param homeLocation: homeLocation include latitude and longitude
 * @return Execution result.
 */
T_DjiReturnCode DjiFlightController_SetHomeLocationUsingGPSCoordinates(T_DjiFlightControllerHomeLocation homeLocation);

/**
 * @brief Set home location using current aircraft GPS (not RTK) location.
 * @note Set home location failed reasons may as follows:
 * 1. Aircraft's gps level can't reach the condition of recording home location.
 * 2. Record initial home location failed after start aircraft.
 * @return Execution result.
 */
T_DjiReturnCode DjiFlightController_SetHomeLocationUsingCurrentAircraftLocation(void);

/**
 * @brief Set go home altitude.
 * @note If aircraft's current altitude is higher than the setting value of go home altitude, aircraft will go home
 * using current altitude. Otherwise, it will climb to setting of go home altitude ,and then execute go home action.
 * Go home altitude setting is 20-1500 m.
 * @param altitude: go home altitude, unit: meter
 * @return Execution result.
 */
T_DjiReturnCode DjiFlightController_SetGoHomeAltitude(E_DjiFlightControllerGoHomeAltitude altitude);

/**
 * @brief Get go home altitude.
 * @param altitude: go home altitude, unit: meter
 * @return Execution result.
 */
T_DjiReturnCode DjiFlightController_GetGoHomeAltitude(E_DjiFlightControllerGoHomeAltitude *altitude);

/**
 * @brief Get country code.
 * @param countryCode: Pointer of buffer to return country code. The country code indicates the current country or
 * region where the aircraft is located. Please refer to the ISO 3166-1 code table for the specific meaning of the
 * country code.
 * @return Execution result.
 */
T_DjiReturnCode DjiFlightController_GetCountryCode(uint16_t *countryCode);

/**
 * @brief Request go home action when the aircraft is in the air
 * @return Execution result.
 */
T_DjiReturnCode DjiFlightController_StartGoHome(void);

/**
 * @brief Request cancel go home action when the aircraft is going home
 * @return Execution result.
 */
T_DjiReturnCode DjiFlightController_CancelGoHome(void);

/**
 * @brief Obtain aircraft's joystick control permission.
 * @note 1.You have to obtain joystick control permission successfully before you using joystick to control aircraft.
 * 2. RC must be in p-mode.
 * @return Execution result.
 */
T_DjiReturnCode DjiFlightController_ObtainJoystickCtrlAuthority(void);

/**
 * @brief Release aircraft's joystick control permission.
 * @note RC must be in p-mode.
 * @return Execution result.
 */
T_DjiReturnCode DjiFlightController_ReleaseJoystickCtrlAuthority(void);

/**
 * @brief Subscribe to joystick control permission switch event with a callback function.
 * @note it will be triggered once the joystick control permission switch event occurs.
 * @return Execution result.
 */
T_DjiReturnCode DjiFlightController_RegJoystickCtrlAuthorityEventCallback(JoystickCtrlAuthorityEventCbFunc callback);

/**
 * @brief Set expected joystick mode before requesting joystick.
 * @param joystickMode: include horizontal/vertical/yaw control mode, stable control mode.
 */
void DjiFlightController_SetJoystickMode(T_DjiFlightControllerJoystickMode joystickMode);

/**
 * @brief Request execute joystick action.
 * @param joystickCommand: include x/y/z/yaw.
 * @return Execution result.
 */
T_DjiReturnCode DjiFlightController_ExecuteJoystickAction(T_DjiFlightControllerJoystickCommand joystickCommand);

/**
 * @brief Request emergency brake action.
 * @return Execution result.
 */
T_DjiReturnCode DjiFlightController_ExecuteEmergencyBrakeAction(void);

/**
 * @brief Request cancel emergency brake action.
 * @note It is only support on M30.If you use DjiFlightController_ExecuteEmergencyBrakeAction(), you need to use
 * "DjiFlightController_CancelEmergencyBrakeAction()" to allow aircraft to execute aircraft action again.
 * @return Execution result.
 */
T_DjiReturnCode DjiFlightController_CancelEmergencyBrakeAction(void);

/**
 * @brief Get general info of the aircraft.
 * @param generalInfo: the struct stored the serial num which contains a array of chars var in case the user gives an
 * illegal length character pointer
 * @return Execution result.
 */
T_DjiReturnCode DjiFlightController_GetGeneralInfo(T_DjiFlightControllerGeneralInfo *generalInfo);

/*! @brief The command decides whether execute RC lost action or not when PSDK is running
  * @note  This setting only affects the behavior of the aircraft when the RC lost and the PSDK is connected.
  *         if the command is enable, the aircraft will not execute RC lost action when RC is lost but PSDK is running;
  *         if the command is disable, the aircraft will execute RC lost action when RC is lost but PSDK is running
  *         the aircraft will execute RC lost action when RC is lost and PSDK is lost whatever the command is.
  *         default command is disable.
  * @param executeRCLostActionOrNotWhenOnboardOn  enable:1;disable:0
  * @return T_DjiReturnCode error code
   */
T_DjiReturnCode
DjiFlightController_SetRCLostActionEnableStatus(E_DjiFlightControllerRCLostActionEnableStatus command);

/*! @brief get RC lost action enable status(enable or disable)
 *  @param command executeRCLostActionOrNotWhenOnboardOn, enable:1;disable:0
 *  @return  T_DjiReturnCode error code
 */
T_DjiReturnCode
DjiFlightController_GetEnableRCLostActionStatus(E_DjiFlightControllerRCLostActionEnableStatus *command);

/**
 * @brief Register callback function for the trigger FTS event.
 * @note The timing of the trigger of the callback function of the FTS is determined by the aircraft, and the trigger
 *       execution action of the FTS needs to be implemented in the callback function and the correct return value
*        must be returned, otherwise the aircraft will always be triggered.
 * @param callback: the callback for the trigger FTS event.
 * @return Execution result.
 */
T_DjiReturnCode DjiFlightController_RegTriggerFtsEventCallback(TriggerFtsEventCallback callback);

#ifdef __cplusplus
}
#endif

#endif // DJI_FLIGHT_CONTROLLER_H
/************************ (C) COPYRIGHT DJI Innovations *******END OF FILE******/
