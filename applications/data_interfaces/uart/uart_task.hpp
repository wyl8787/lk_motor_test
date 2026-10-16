#ifndef _UART_TASK_HPP
#define _UART_TASK_HPP

#include "io/dbus/dbus.hpp"

// ============================ 遥控器 ============================
// 达妙板: 遥控器接收机接 UART5(100000bps, 8E1), 不用DMA
// 接收回调见 uart_task.cpp, 解析在 sp::DBus 里
inline sp::DBus remote(&huart5, false);

// 遥控器摇杆 -> 电机指令的映射参数
namespace remote_ctrl
{
// 摇杆满量程对应的输出端速度, 单位: rad/s
constexpr float MHF9025_MAX_SPEED = 6.0f;  // 左摇杆垂直, 慢速便于台架观察
constexpr float MG4005_MAX_SPEED = 6.0f;   // 右摇杆垂直
// 摇杆死区
constexpr float DEAD_ZONE = 0.05f;
}  // namespace remote_ctrl

#endif  // _UART_TASK_HPP
