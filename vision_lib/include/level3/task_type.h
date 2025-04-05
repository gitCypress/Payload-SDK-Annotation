#ifndef TASK_TYPE_H // 头文件保护宏
#define TASK_TYPE_H

#include <stdio.h>

namespace stage_2
{
    /**
     * @brief 用于存储旋转齿轮相关信息的结构体。
     */
    struct RotatedGear
    {
        /** @brief 时间戳 (单位可能为秒或其他)。*/
        double time;
        /** @brief 目标点或瞄准点的三维坐标 [x, y, z]。*/
        float aimpoint[3];
        /** @brief 信息的更新时间戳 (可能是系统时间或帧计数)。*/
        long update_time;

        /**
         * @brief 默认构造函数。
         *        初始化 time 和 update_time 为 0.0，aimpoint 为 [0.0f, 0.0f, 0.0f]。
         */
        RotatedGear() : time(0.0), update_time(0)
        {
            // 初始化 aimpoint 数组为全零
            for (int i = 0; i < 3; ++i)
            {
                aimpoint[i] = 0.0f;
            }
        }

        /**
         * @brief 带参数的构造函数。
         * @param t 时间戳。
         * @param ap 目标点三维坐标数组。
         * @param ut 更新时间戳。
         */
        RotatedGear(double t, const float ap[3], long ut) : time(t), update_time(ut)
        {
            // 复制 aimpoint 数组的值
            for (int i = 0; i < 3; ++i)
            {
                aimpoint[i] = ap[i];
            }
        }
    };
}

#endif // TASK_TYPE_H