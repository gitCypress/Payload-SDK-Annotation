/**
 ********************************************************************
 * @file    dji_platform.h
 * @brief   This is the header file for "dji_platform.c", defining the structure and
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
 * If you receive this source code without DJI’s authorization, you may not
 * further disseminate the information, and you must immediately remove the
 * source code and notify DJI of its removal. DJI reserves the right to pursue
 * legal actions against you for any loss(es) or damage(s) caused by your
 * failure to do so.
 *
 *********************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DJI_PLATFORM_H
#define DJI_PLATFORM_H

/* Includes ------------------------------------------------------------------*/
#include "dji_typedef.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Exported constants --------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/**
* @brief Platform handle of uart operation.
*/
typedef void *T_DjiUartHandle;

/**
* @brief Platform handle of usb bulk operation.
*/
typedef void *T_DjiUsbBulkHandle;

/**
* @brief Platform handle of network operation.
*/
typedef void *T_DjiNetworkHandle;

/**
* @brief Platform handle of i2c device operation.
*/
typedef void *T_DjiI2cHandle;

/**
* @brief Platform handle of thread task operation.
*/
typedef void *T_DjiTaskHandle;

/**
* @brief Platform handle of mutex operation.
*/
typedef void *T_DjiMutexHandle;

/**
* @brief Platform handle of semaphore operation.
*/
typedef void *T_DjiSemaHandle;

/**
* @brief Platform handle of file operation.
*/
typedef void *T_DjiFileHandle;

/**
* @brief Platform handle of dir operation.
*/
typedef void *T_DjiDirHandle;

/**
* @brief Platform handle of socket operation.
*/
typedef void *T_DjiSocketHandle;

typedef enum {
    /**
    * All aircraft type support，users can connect via chip serial port or USB to TTL serial port.
    * Baud rate support list on M300 RTK Payload Port: 115200, 230400, 460800, 921600.
    * Baud rate support list on M300 RTK Extension Port: 115200, 230400, 460800, 921600, 1000000.
    * Baud rate support list on M30/M30T: 115200, 230400, 460800, 921600, 1000000.
    * Baud rate support list on M3E/M3T: 921600.
    * Baud rate support list on M350 RTK Payload Port: 115200, 230400, 460800, 921600.
    * Baud rate support list on M350 RTK Extension Port: 115200, 230400, 460800, 921600, 1000000.
    * */
    DJI_HAL_UART_NUM_0,
    /**
    * Only support on M300/M350 RTK Extension Port by USB virtual serial port, such as /dev/ttyACM0.
    * Baud rate support list on M300 RTK Extension Port: 921600.
    * Baud rate support list on M350 RTK Extension Port: 921600.
    * */
    DJI_HAL_UART_NUM_1,
} E_DjiHalUartNum;

typedef enum {
    DJI_HAL_USB_BULK_NUM_0 = 0,
    DJI_HAL_USB_BULK_NUM_1,
    DJI_HAL_USB_BULK_NUM_MAX,
} E_DjiHalUsbBulkNum;

typedef enum {
    DJI_SOCKET_MODE_UDP,
    DJI_SOCKET_MODE_TCP,
} E_DjiSocketMode;

typedef struct {
    bool isConnect;
} T_DjiUartStatus;

typedef struct {
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
} T_DjiTime;

typedef struct {
    uint32_t size;
    T_DjiTime createTime;
    T_DjiTime modifyTime;
    char path[DJI_FILE_PATH_SIZE_MAX];
    bool isDir;
} T_DjiFileInfo;

typedef struct {
    T_DjiReturnCode (*UartInit)(E_DjiHalUartNum uartNum, uint32_t baudRate, T_DjiUartHandle *uartHandle);

    T_DjiReturnCode (*UartDeInit)(T_DjiUartHandle uartHandle);

    T_DjiReturnCode (*UartWriteData)(T_DjiUartHandle uartHandle, const uint8_t *buf, uint32_t len, uint32_t *realLen);

    T_DjiReturnCode (*UartReadData)(T_DjiUartHandle uartHandle, uint8_t *buf, uint32_t len, uint32_t *realLen);

    T_DjiReturnCode (*UartGetStatus)(E_DjiHalUartNum uartNum, T_DjiUartStatus *status);
} T_DjiHalUartHandler;

typedef struct {
    uint16_t interfaceNum;
    uint16_t endPointIn;
    uint16_t endPointOut;
} T_DjiHalUsbBulkChannelInfo;

typedef struct {
    bool isUsbHost;
    // attention: if 'isUsbHost' equals false, the following parameters is not valid.
    uint16_t pid;
    uint16_t vid;
    T_DjiHalUsbBulkChannelInfo channelInfo;
} T_DjiHalUsbBulkInfo;

typedef struct {
    uint16_t pid;
    uint16_t vid;
    T_DjiHalUsbBulkChannelInfo channelInfo[DJI_HAL_USB_BULK_NUM_MAX];
} T_DjiHalUsbBulkDeviceInfo;

typedef struct {
    struct {
        uint16_t vid;
        uint16_t pid;
    } usbNetAdapter;
} T_DjiHalNetworkDeviceInfo;

typedef struct {
    uint32_t i2cSpeed;
    uint16_t devAddress;
} T_DjiHalI2cConfig;

typedef struct {
    T_DjiReturnCode (*UsbBulkInit)(T_DjiHalUsbBulkInfo usbBulkInfo, T_DjiUsbBulkHandle *usbBulkHandle);

    T_DjiReturnCode (*UsbBulkDeInit)(T_DjiUsbBulkHandle usbBulkHandle);

    T_DjiReturnCode (*UsbBulkWriteData)(T_DjiUsbBulkHandle usbBulkHandle, const uint8_t *buf,
                                        uint32_t len, uint32_t *realLen);

    T_DjiReturnCode (*UsbBulkReadData)(T_DjiUsbBulkHandle usbBulkHandle, uint8_t *buf,
                                       uint32_t len, uint32_t *realLen);

    T_DjiReturnCode (*UsbBulkGetDeviceInfo)(T_DjiHalUsbBulkDeviceInfo *deviceInfo);
} T_DjiHalUsbBulkHandler;

typedef struct {
    T_DjiReturnCode (*NetworkInit)(const char *ipAddr, const char *netMask, T_DjiNetworkHandle *networkHandle);

    T_DjiReturnCode (*NetworkDeInit)(T_DjiNetworkHandle networkHandle);

    T_DjiReturnCode (*NetworkGetDeviceInfo)(T_DjiHalNetworkDeviceInfo *deviceInfo);
} T_DjiHalNetworkHandler;

typedef struct {
    T_DjiReturnCode (*I2cInit)(T_DjiHalI2cConfig i2cConfig, T_DjiI2cHandle *i2cHandle);

    T_DjiReturnCode (*I2cDeInit)(T_DjiI2cHandle i2cHandle);

    T_DjiReturnCode (*I2cWriteData)(T_DjiI2cHandle i2cHandle, uint16_t devAddress, const uint8_t *buf,
                                    uint32_t len, uint32_t *realLen);

    T_DjiReturnCode (*I2cReadData)(T_DjiI2cHandle i2cHandle, uint16_t devAddress, uint8_t *buf,
                                   uint32_t len, uint32_t *realLen);
} T_DjiHalI2cHandler;

typedef struct {
    /**
     * @brief 创建任务（线程）
     * @details 创建一个新的线程来执行指定的任务函数
     * @param name 任务名称，用于调试和识别
     * @param taskFunc 任务函数指针，指向线程要执行的函数
     * @param stackSize 线程栈大小，单位为字节
     * @param arg 传递给任务函数的参数
     * @param task 输出参数，用于存储创建的任务句柄
     * @return 执行结果
     */
    T_DjiReturnCode (*TaskCreate)(const char *name, void *(*taskFunc)(void *),
                                  uint32_t stackSize, void *arg, T_DjiTaskHandle *task);

    /**
     * @brief 销毁任务（线程）
     * @details 终止并释放指定任务的资源
     * @param task 要销毁的任务句柄
     * @return 执行结果
     */
    T_DjiReturnCode (*TaskDestroy)(T_DjiTaskHandle task);

    /**
     * @brief 任务休眠
     * @details 使当前任务休眠指定的毫秒数
     * @param timeMs 休眠时间，单位为毫秒
     * @return 执行结果
     */
    T_DjiReturnCode (*TaskSleepMs)(uint32_t timeMs);

    /**
     * @brief 创建互斥锁
     * @details 创建一个互斥锁用于线程同步
     * @param mutex 输出参数，用于存储创建的互斥锁句柄
     * @return 执行结果
     */
    T_DjiReturnCode (*MutexCreate)(T_DjiMutexHandle *mutex);

    /**
     * @brief 销毁互斥锁
     * @details 释放互斥锁占用的资源
     * @param mutex 要销毁的互斥锁句柄
     * @return 执行结果
     */
    T_DjiReturnCode (*MutexDestroy)(T_DjiMutexHandle mutex);

    /**
     * @brief 获取互斥锁（加锁）
     * @details 尝试获取互斥锁，如果锁被占用则阻塞等待
     * @param mutex 要获取的互斥锁句柄
     * @return 执行结果
     */
    T_DjiReturnCode (*MutexLock)(T_DjiMutexHandle mutex);

    /**
     * @brief 释放互斥锁（解锁）
     * @details 释放已获取的互斥锁，使其他线程可以获取
     * @param mutex 要释放的互斥锁句柄
     * @return 执行结果
     */
    T_DjiReturnCode (*MutexUnlock)(T_DjiMutexHandle mutex);

    /**
     * @brief 创建信号量
     * @details 创建一个具有初始值的信号量
     * @param initValue 信号量的初始值
     * @param semaphore 输出参数，用于存储创建的信号量句柄
     * @return 执行结果
     */
    T_DjiReturnCode (*SemaphoreCreate)(uint32_t initValue, T_DjiSemaHandle *semaphore);

    /**
     * @brief 销毁信号量
     * @details 释放信号量占用的资源
     * @param semaphore 要销毁的信号量句柄
     * @return 执行结果
     */
    T_DjiReturnCode (*SemaphoreDestroy)(T_DjiSemaHandle semaphore);

    /**
     * @brief 等待信号量
     * @details 尝试获取信号量，如果信号量值为0则阻塞等待
     * @param semaphore 要等待的信号量句柄
     * @return 执行结果
     */
    T_DjiReturnCode (*SemaphoreWait)(T_DjiSemaHandle semaphore);

    /**
     * @brief 带超时的等待信号量
     * @details 尝试在指定时间内获取信号量，超时则返回错误
     * @param semaphore 要等待的信号量句柄
     * @param waitTimeMs 最大等待时间，单位为毫秒
     * @return 执行结果
     */
    T_DjiReturnCode (*SemaphoreTimedWait)(T_DjiSemaHandle semaphore, uint32_t waitTimeMs);

    /**
     * @brief 释放信号量
     * @details 增加信号量的值，可能唤醒等待该信号量的线程
     * @param semaphore 要释放的信号量句柄
     * @return 执行结果
     */
    T_DjiReturnCode (*SemaphorePost)(T_DjiSemaHandle semaphore);

    /**
     * @brief 获取当前系统时间（毫秒）
     * @details 获取自系统启动以来的毫秒数
     * @param ms 输出参数，用于存储当前时间（毫秒）
     * @return 执行结果
     */
    T_DjiReturnCode (*GetTimeMs)(uint32_t *ms);

    /**
     * @brief 获取当前系统时间（微秒）
     * @details 获取自系统启动以来的微秒数
     * @param us 输出参数，用于存储当前时间（微秒）
     * @return 执行结果
     */
    T_DjiReturnCode (*GetTimeUs)(uint64_t *us);

    /**
     * @brief 获取随机数
     * @details 生成一个16位随机数
     * @param randomNum 输出参数，用于存储生成的随机数
     * @return 执行结果
     */
    T_DjiReturnCode (*GetRandomNum)(uint16_t *randomNum);

    /**
     * @brief 内存分配
     * @details 分配指定大小的内存块
     * @param size 要分配的内存大小，单位为字节
     * @return 分配的内存块指针，失败则返回NULL
     */
    void *(*Malloc)(uint32_t size);

    /**
     * @brief 内存释放
     * @details 释放之前分配的内存块
     * @param ptr 要释放的内存块指针
     */
    void (*Free)(void *ptr);
} T_DjiOsalHandler;

typedef struct {
    T_DjiReturnCode (*FileOpen)(const char *fileName, const char *fileMode, T_DjiFileHandle *fileObj);

    T_DjiReturnCode (*FileClose)(T_DjiFileHandle fileObj);

    T_DjiReturnCode (*FileWrite)(T_DjiFileHandle fileObj, const uint8_t *buf, uint32_t len, uint32_t *realLen);

    T_DjiReturnCode (*FileRead)(T_DjiFileHandle fileObj, uint8_t *buf, uint32_t len, uint32_t *realLen);

    T_DjiReturnCode (*FileSeek)(T_DjiFileHandle fileObj, uint32_t offset);

    T_DjiReturnCode (*FileSync)(T_DjiFileHandle fileObj);

    T_DjiReturnCode (*DirOpen)(const char *filePath, T_DjiDirHandle *dirObj);

    T_DjiReturnCode (*DirClose)(T_DjiDirHandle dirObj);

    T_DjiReturnCode (*DirRead)(T_DjiDirHandle dirObj, T_DjiFileInfo *fileInfo);

    T_DjiReturnCode (*Mkdir)(const char *filePath);

    T_DjiReturnCode (*Unlink)(const char *filePath);

    T_DjiReturnCode (*Rename)(const char *oldFilePath, const char *newFilePath);

    T_DjiReturnCode (*Stat)(const char *filePath, T_DjiFileInfo *fileInfo);
} T_DjiFileSystemHandler;

typedef struct {
    T_DjiReturnCode (*Socket)(E_DjiSocketMode mode, T_DjiSocketHandle *socketHandle);

    T_DjiReturnCode (*Close)(T_DjiSocketHandle socketHandle);

    T_DjiReturnCode (*Bind)(T_DjiSocketHandle socketHandle, const char *ipAddr, uint32_t port);

    T_DjiReturnCode (*UdpSendData)(T_DjiSocketHandle socketHandle, const char *ipAddr, uint32_t port,
                                   const uint8_t *buf, uint32_t len, uint32_t *realLen);

    T_DjiReturnCode (*UdpRecvData)(T_DjiSocketHandle socketHandle, char *ipAddr, uint32_t *port,
                                   uint8_t *buf, uint32_t len, uint32_t *realLen);

    T_DjiReturnCode (*TcpListen)(T_DjiSocketHandle socketHandle);

    T_DjiReturnCode (*TcpAccept)(T_DjiSocketHandle socketHandle, char *ipAddr, uint32_t *port,
                                 T_DjiSocketHandle *outSocketHandle);

    T_DjiReturnCode (*TcpConnect)(T_DjiSocketHandle socketHandle, const char *ipAddr, uint32_t port);

    T_DjiReturnCode (*TcpSendData)(T_DjiSocketHandle socketHandle,
                                   const uint8_t *buf, uint32_t len, uint32_t *realLen);

    T_DjiReturnCode (*TcpRecvData)(T_DjiSocketHandle socketHandle,
                                   uint8_t *buf, uint32_t len, uint32_t *realLen);
} T_DjiSocketHandler;

/* Exported functions --------------------------------------------------------*/
/**
 * @brief Register the handler for hal uart interfaces by your platform.
 * @note It should be noted that the interface in hal is written and tested well. Users need to implement all the
 * interfaces. Otherwise, the user interface cannot be successfully registered, and then the user interface is registered
 * through the interface. If the registration fails, it needs to be based on the return code. To judge the problem. Make
 * sure that the feature is available after a successful registration. The interface needs to be called at the beginning of
 * the application for registration, otherwise, the subsequent functions will not work properly.
 * @param halUartHandler: pointer to the handler for hal uart interfaces by your platform.
 * @return Execution result.
 */
T_DjiReturnCode DjiPlatform_RegHalUartHandler(const T_DjiHalUartHandler *halUartHandler);

/**
 * @brief Register the handler for usb bulk interfaces by your platform.
 * @param fileSystemHandler: pointer to the handler for usb bulk interfaces by your platform.
 * @return Execution result.
 */
T_DjiReturnCode DjiPlatform_RegHalUsbBulkHandler(const T_DjiHalUsbBulkHandler *halUsbBulkHandler);

/**
 * @brief Register the handler for hal network interfaces by your platform.
 * @note It should be noted that the interface in hal is written and tested well. Users need to implement all the
 * interfaces. Otherwise, the user interface cannot be successfully registered, and then the user interface is registered
 * through the interface. If the registration fails, it needs to be based on the return code. To judge the problem. Make
 * sure that the feature is available after a successful registration.
 * @attention The interface needs to be called at the beginning of the application for registration, otherwise, the
 * subsequent functions will not work properly.
 * @param halNetworkHandler: pointer to the handler for network handler interfaces by your platform.
 * @return Execution result.
 */
T_DjiReturnCode DjiPlatform_RegHalNetworkHandler(const T_DjiHalNetworkHandler *halNetworkHandler);

/**
 * @brief Register the handler for hal i2c master mode interfaces by your platform.
 * @note It should be noted that the interface in hal is written and tested well. Users need to implement all the
 * interfaces. Otherwise, the user interface cannot be successfully registered, and then the user interface is registered
 * through the interface. If the registration fails, it needs to be based on the return code. To judge the problem. Make
 * sure that the feature is available after a successful registration.
 * @attention The interface needs to be called at the beginning of the application for registration, otherwise, the
 * subsequent functions will not work properly.
 * @param halI2cHandler: pointer to the handler for hal i2c handler interfaces by your platform.
 * @return Execution result.
 */
T_DjiReturnCode DjiPlatform_RegHalI2cHandler(const T_DjiHalI2cHandler *halI2cHandler);

/**
 * @brief Register the handler for osal interfaces by your platform.
 * @note It should be noted that the interface in osal is written and tested well. Users need to implement all the
 * interfaces. Otherwise, the user interface cannot be successfully registered, and then the user interface is registered
 * through the interface. If the registration fails, it needs to be based on the return code. To judge the problem. Make
 * sure that the feature is available after a successful registration. The interface needs to be called at the beginning of
 * the application for registration, otherwise, the subsequent functions will not work properly.
 * @param osalHandler: pointer to the handler for osal interfaces by your platform.
 * @return Execution result.
 */
T_DjiReturnCode DjiPlatform_RegOsalHandler(const T_DjiOsalHandler *osalHandler);

/**
 * @brief Register the handler for file-system interfaces by your platform.
 * @param fileSystemHandler: pointer to the handler for file-system interfaces by your platform.
 * @return Execution result.
 */
T_DjiReturnCode DjiPlatform_RegFileSystemHandler(const T_DjiFileSystemHandler *fileSystemHandler);

/**
 * @brief Register the handler for socket interfaces by your platform.
 * @param fileSystemHandler: pointer to the handler for socket interfaces by your platform.
 * @return Execution result.
 */
T_DjiReturnCode DjiPlatform_RegSocketHandler(const T_DjiSocketHandler *socketHandler);

/**
 * @brief Get the handler of osal interfaces.
 * @return Pointer to osal handler.
 */
T_DjiOsalHandler *DjiPlatform_GetOsalHandler(void);

/**
 * @brief Get the handler of usb bulk interfaces.
 * @return Pointer to usb bulk handler.
 */
T_DjiHalUsbBulkHandler *DjiPlatform_GetHalUsbBulkHandler(void);

/**
 * @brief Get the handler of network interfaces.
 * @return Pointer to network handler.
 */
T_DjiHalNetworkHandler *DjiPlatform_GetHalNetworkHandler(void);

/**
 * @brief Get the handler of i2c interfaces.
 * @return Pointer to i2c handler.
 */
T_DjiHalI2cHandler *DjiPlatform_GetHalI2cHandler(void);

/**
 * @brief Get the handler of file-system interfaces.
 * @return Pointer to file-system handler.
 */
T_DjiFileSystemHandler *DjiPlatform_GetFileSystemHandler(void);

/**
 * @brief Get the handler of socket interfaces.
 * @return Pointer to socket handler.
 */
T_DjiSocketHandler *DjiPlatform_GetSocketHandler(void);

#ifdef __cplusplus
}
#endif

#endif // DJI_PLATFORM_H
/************************ (C) COPYRIGHT DJI Innovations *******END OF FILE******/
