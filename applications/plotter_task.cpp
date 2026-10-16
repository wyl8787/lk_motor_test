#include "cmsis_os.h"
#include "controllers/chassis_para.hpp"
#include "data_interfaces/can/can.hpp"
#include "data_interfaces/uart/uart_task.hpp"
#include "io/plotter/plotter.hpp"

sp::Plotter plotter(&huart7, false);

extern "C" void plot_task()
{
  while (true) {
    plotter.plot(
      //遥控器: 别的通道
      // remote.ch_lh, remote.ch_rh, remote.ch_lu,
      // remote.mouse.vx, remote.mouse.vy, remote.mouse.vs,
      // remote.mouse.left, remote.mouse.right,
      // remote.keys.w, remote.keys.s, remote.keys.a, remote.keys.d,
      // remote.keys.shift, remote.keys.ctrl, remote.keys.q, remote.keys.e,
      // static_cast<float>(remote.keyboard_value),

      //电机反馈: 转速 / 角度 / 力矩 / 电流
      // lk_mhf9025.speed, lk_mg4005.speed,
      // lk_mhf9025.angle, lk_mg4005.angle,
      // lk_mhf9025.multicycle_angle, lk_mg4005.multicycle_angle,
      lk_mhf9025.torque, lk_mg4005.torque,
      lk_mhf9025.current, lk_mg4005.current
      // lk_mhf9025.temp, lk_mg4005.temp,
      // lk_mhf9025.motor_state, lk_mg4005.motor_state,
      // lk_mhf9025.error_state, lk_mg4005.error_state,

      //下发指令 / 电机在线 / 反馈帧率
      // telemetry.cmd_mhf9025, telemetry.cmd_mg4005,
      // telemetry.online_mhf9025, telemetry.online_mg4005,
      // telemetry.frame_mhf9025, telemetry.frame_mg4005,
      // telemetry.time_s,

      //编码器原始值(核对位宽): 左拨杆打到上档会让电机转 TURN_TURNS 圈
      // 拨杆下档会自动把 encoder_raw_max 清零 -> 再打上档转一圈 -> 看 max 就知道位宽:
      // 0~65535 = 16bit, 0~32767 = 15bit, 0~16383 = 14bit
      // static_cast<float>(lk_mhf9025.encoder_raw), static_cast<float>(lk_mg4005.encoder_raw),
      // static_cast<float>(lk_mhf9025.encoder_raw_max), static_cast<float>(lk_mg4005.encoder_raw_max)

      //判断"到底有没有收到CAN帧": 温度(收到就是25~40) + 在线 + 帧计数
      // static_cast<float>(lk_mhf9025.temp), static_cast<float>(lk_mg4005.temp),
      // telemetry.online_mhf9025, telemetry.online_mg4005,
      // static_cast<float>(telemetry.frame_mhf9025), static_cast<float>(telemetry.frame_mg4005),

      //单机调试(例如只接 9025)
      // lk_mhf9025.speed, lk_mhf9025.torque, lk_mhf9025.current,
      // telemetry.cmd_mhf9025, lk_mhf9025.temp,

      //当前: 遥控器读取验证
      // 1 收到过数据(0/1) / 2 正在收(0/1 需100ms内收到帧)
      // 3 右拨杆(0=下 1=中 2=上) / 4 左拨杆(同)
      // 5 左摇杆垂直 / 6 右摇杆垂直
      // 7 控制状态(0=STOP 1=RUN)
      // remote.is_open(), remote.is_alive(osKernelSysTick()),
      // static_cast<float>(remote.sw_r),
      // static_cast<float>(mode),

      //当前: 反馈帧计时(µs)
      // 8/9   帧间隔(1000Hz 指令下正常约 800~1200; 跳 2000/3000 就是丢帧)
      // 10/11 发送->反馈延迟(手册说 0.25ms 内; 负数=这轮没等到回复)
      // 12/13 最大帧间隔(上电以来最坏情况; 拨杆打到下档会自动清零, 见 control_task)
      // timing_mhf9025.interval_us, timing_mg4005.interval_us,
      // timing_mhf9025.latency_us, timing_mg4005.latency_us
      // timing_mhf9025.interval_max_us, timing_mg4005.interval_max_us
      );
    osDelay(10);  // 100Hz
  }
}
