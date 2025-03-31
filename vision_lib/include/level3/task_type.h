#ifndef TASK_TYPE_H // 头文件保护宏
#define TASK_TYPE_H

#include <stdio.h>

namespace stage_2
{
    struct RotatedGear
    {
        double time;
        float aimpoint[3];
        long update_time;

        // 构造函数
        RotatedGear() : time(0.0), update_time(0.0)
        {
            // 初始化 aimpoint 数组为全零
            for (int i = 0; i < 3; ++i)
            {
                aimpoint[i] = 0.0f;
            }
        }

        // 带参数的构造函数
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