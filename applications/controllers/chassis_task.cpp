#include "cmsis_os.h"
#include "controllers/chassis_para.hpp"
#include "data_interfaces/can/can.hpp"
#include "data_interfaces/uart/uart_task.hpp"
#include "tools/math_tools/math_tools.hpp"

namespace
{
// 遥控器拨杆 -> 控制状态
//   左拨杆 上   = 一次性转圈(优先)
//   右拨杆 上   = 匀速
//   右拨杆 中/下 = 停转
// 没插遥控器 / 遥控器掉线, 都按停转处理(保护)
Mode read_mode()
{
  if (!remote.is_open()) return Mode::stop_mode;
  if (!remote.is_alive(osKernelSysTick())) return Mode::stop_mode;

  if (remote.sw_l == sp::DBusSwitchMode::UP) return Mode::turn_mode;

  return (remote.sw_r == sp::DBusSwitchMode::UP) ? Mode::run_mode : Mode::stop_mode;
}

// 拨杆打到下档(停转)时, 自动清零这些"最大值保持", 方便"停一下 -> 再打上去跑":
//   interval_max_us 反馈帧最大间隔
//   encoder_raw_max 编码器原始值最大值(转一整圈后看它就知道编码器是多少位)
// 用边沿触发: 只在"进入停转的那一拍"清一次; 否则没接遥控器时 mode 恒为停转, 会每拍清零
void clear_max_values()
{
  timing_mhf9025.interval_max_us = 0.0f;
  timing_mg4005.interval_max_us = 0.0f;
  lk_mhf9025.encoder_raw_max = 0;
  lk_mg4005.encoder_raw_max = 0;
}

// 按控制状态更新两台电机的指令缓存(真正的组帧与发送在 can_task 里)
// 只有两种状态: 匀速 -> 下发设定值; 停转 -> 下发 0
void apply_ctrl()
{
  const bool run = (mode == Mode::run_mode);

  if (lk_ctrl::USE_TORQUE_CONTROL) {
    // 力矩环: 匀速 = 设定力矩(N·m), 停转 = 0
    const float tq_mhf9025 = run ? lk_ctrl::torque_set_mhf9025 : 0.0f;
    const float tq_mg4005 = run ? lk_ctrl::torque_set_mg4005 : 0.0f;

    lk_mhf9025.cmd_torque(tq_mhf9025);
    lk_mg4005.cmd_torque(tq_mg4005);

    telemetry.cmd_mhf9025 = tq_mhf9025;
    telemetry.cmd_mg4005 = tq_mg4005;
  }
  else {
    // 速度环: 匀速 = 设定转速(rad/s), 停转 = 0(主动刹住, 电机不会自由滑行)
    const float v_mhf9025 = run ? lk_ctrl::RUN_SPEED_MHF9025 : 0.0f;
    const float v_mg4005 = run ? lk_ctrl::RUN_SPEED_MG4005 : 0.0f;

    lk_mhf9025.cmd_velocity(v_mhf9025);
    lk_mg4005.cmd_velocity(v_mg4005);

    telemetry.cmd_mhf9025 = v_mhf9025;
    telemetry.cmd_mg4005 = v_mg4005;
  }
}
}  // namespace

// ============================ LK 电机控制 ============================
// MHF9025(ID=1, 直驱) 与 MG4005(ID=2, 自带 10:1 行星减速箱) 都挂在 CAN3(FDCAN3) 上
//
// 控制区只有两种状态, 由遥控器右拨杆(sw_r)切换:
//   sw_r 上   -> 匀速: 按 lk_ctrl::RUN_SPEED_xxx 的转速运行
//   sw_r 中/下 -> 停转: 目标转速 0
// 力矩设置接口见 chassis_para.hpp 的 lk_ctrl::torque_set_xxx + USE_TORQUE_CONTROL
//
// 本任务只负责"算什么", 真正的组帧与发送在 can_task 里(见 data_interfaces/can/can_send.cpp)
// 下发值与反馈值通过 plot_task(huart7) 观察, 文本日志由 plot_task 打印
extern "C" void control_task()
{
  // 与 can_task 的启动延时对齐
  osDelay(2000);

  constexpr float dt = lk_ctrl::CTRL_PERIOD_MS * 0.001f;

  // 一次性转圈的输出端角度(度) = 圈数 × 360 / 总减速比(这样电机侧正好是 TURN_TURNS 圈)
  const float turn_mhf9025 = lk_ctrl::TURN_TURNS * 360.0f / lk_mhf9025.ratio();
  const float turn_mg4005 = lk_ctrl::TURN_TURNS * 360.0f / lk_mg4005.ratio();

  uint32_t tick = 0;
  Mode last_mode = mode;  // 初值取当前值, 避免上电就打一次清零

  for (;;) {
    mode = read_mode();

    // 左拨杆打到上档(进入转圈模式)的瞬间 -> 锁存一帧增量位置命令, can_task 只发这一次,
    // 发完安静 one_shot.hold_us 让电机把这一圈走完, 之后 can_task 会继续发"停转"帧
    if (mode == Mode::turn_mode && last_mode != Mode::turn_mode) {
      lk_mhf9025.cmd_angle_increment(turn_mhf9025);
      one_shot_arm_angle_increment(one_shot_mhf9025, lk_mhf9025);

      lk_mg4005.cmd_angle_increment(turn_mg4005);
      one_shot_arm_angle_increment(one_shot_mg4005, lk_mg4005);
    }

    // 拨杆打到下档(进入停转)的瞬间 -> 清零各种最大值
    if (mode == Mode::stop_mode && last_mode != Mode::stop_mode) clear_max_values();
    last_mode = mode;

    apply_ctrl();

    const uint32_t now_ms = osKernelSysTick();
    telemetry.time_s = tick * dt;
    telemetry.online_mhf9025 = lk_mhf9025.is_alive(now_ms);
    telemetry.online_mg4005 = lk_mg4005.is_alive(now_ms);
    telemetry.frame_mhf9025 = lk_mhf9025_rx_count;
    telemetry.frame_mg4005 = lk_mg4005_rx_count;

    tick++;
    osDelay(lk_ctrl::CTRL_PERIOD_MS);
  }
}
