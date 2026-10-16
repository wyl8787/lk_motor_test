#ifndef CAN_HPP
#define CAN_HPP

#include <cstdint>

#include "controllers/chassis_para.hpp"
#include "io/fdcan/fdcan.hpp"
#include "motor/lk_motor/lk_motor.hpp"

// ============================ LK 电机的 CAN 接口 ============================
// 两个电机挂在同一根 CAN(FDCAN3, PD12=RX / PD13=TX)上:
//   MHF9025 电机ID = 1 (控制帧/反馈帧都是 0x141)
//   MG4005  电机ID = 2 (控制帧/反馈帧都是 0x142)
//
// 收发习惯与原工程保持一致:
//   接收: HAL_FDCAN_RxFifo0Callback 中断回调(见 can_recv.cpp), 按 rx_id 分发到电机
//   发送: can_task 里周期调用 send_motor_xxx()
// 被测电机实例与控制参数见 controllers/chassis_para.hpp
//
// 总线速率: fdcan.c 中 NominalPrescaler=24, 位时间 = 1(SYNC) + 2(TSEG1) + 2(TSEG2) = 5 tq
//           FDCAN 内核时钟 120MHz => 120MHz / 24 / 5 = 1Mbps, 与瓴控电机出厂默认一致
//
// 总线负载: 每个电机每周期产生 1 帧指令 + 1 帧反馈, 一帧 8 字节标准帧约 112~135bit
//           1000Hz 时 2 * 2 * 1000 = 4000 帧/s, 1Mbps 下总线负载约 45%~54%
//           若想留更多余量, 可以把 lk_ctrl::CTRL_PERIOD_MS 调大(例如2 -> 500Hz)

inline sp::FDCAN fdcan3(&hfdcan3);

// ============================ 反馈帧计时器 ============================
// 用 DWT 的周期计数器做 µs 级时间戳(480MHz, 分辨率约 2ns, 约 8.9s 回绕,
// 回绕用无符号相减自动处理, 只要两次读取间隔 < 8.9s 就是对的)
// 用途:
//   帧间隔 interval_us    —— 两次反馈帧之间的时间。1000Hz 指令下正常约 800~1200µs;
//                            如果时不时跳到 2000µs/3000µs, 就是中间丢帧了
//   最大间隔 interval_max —— 上电以来的最大间隔(丢帧的最坏情况), 想清零直接赋 0
//   延迟   latency_us     —— 从发出本周期指令到收到该电机回复的时间(手册 P5: 回复在命令后 0.25ms 内),
//                            负数表示这一轮没等到合法回复(丢帧/超时)
struct FrameTiming
{
  uint32_t last_rx_us = 0;    // 上一反馈帧的 µs 时间戳
  uint32_t tx_us = 0;         // 本周期指令帧发出时的 µs 时间戳
  bool tx_pending = false;    // 已发指令, 正在等回复
  float interval_us = 0;      // 最近两次反馈帧的间隔, µs
  float interval_max_us = 0;  // 上电以来最大间隔, µs
  float latency_us = -1;      // 发送 -> 反馈 延迟, µs(负数=无效)
};

inline FrameTiming timing_mhf9025;
inline FrameTiming timing_mg4005;

// µs 级时间戳(由 can_start() 里的 DWT 初始化打开)
uint32_t micros();

// ============================ 单次命令(one-shot) ============================
// 有些命令"重复发会出问题": 比如 0xA7 增量位置, 它每次收到都是"从现在的位置再走这么多",
// 1kHz 连着发就等于让电机一直转; 而紧跟其后的 0xA2 速度帧又会把这个动作打断。
// 所以这类命令要走单次通道: 锁存一帧 -> 只发一次 -> 保持安静 hold_us 等动作走完 -> 再恢复普通指令帧。
struct OneShot
{
  bool armed = false;         // 有锁存的单次帧待发
  bool sent = false;          // 已经发过了(正在安静期)
  uint8_t data[8] = {};       // 锁存的帧数据
  uint32_t sent_us = 0;       // 发出去的时刻
  uint32_t hold_us = 2000000; // 发完后保持安静的时长(等动作走完), 默认 2s, 按需调
};

inline OneShot one_shot_mhf9025;
inline OneShot one_shot_mg4005;

// 把"增量位置"(0xA7)按电机当前的 cmd_angle_increment() 锁存成一帧, can_task 只会发这一次
void one_shot_arm_angle_increment(OneShot & one_shot, const sp::LK_Motor & motor);

// 配置 fdcan3 过滤器(接收全部标准帧到 FIFO0)并启动总线, 在 can_task 里调用
void can_start();

// -------------------- 电机指令帧(由 can_task 周期调用) --------------------
// 数据域由 control_task 缓存的指令决定(匀速: 设定转速 / 停转: 0)
void send_motor_mhf9025();
void send_motor_mg4005();

// -------------------- 单帧发送辅助 --------------------
// 发送一帧指令: 数据域由电机对象按最近一次 cmd_* 的模式填充(默认力矩模式)
void can_send(const sp::LK_Motor & motor);
// 发送使能帧(0x88)
void can_send_open(const sp::LK_Motor & motor);
// 发送停止帧(0x81): 硬停机, 需要用的时候手动调
void can_send_stop(const sp::LK_Motor & motor);
// 调试用: 状态1查询帧(0x9A) / 状态2查询帧(0x9C)
void can_send_state1(const sp::LK_Motor & motor);
void can_send_state2(const sp::LK_Motor & motor);

#endif  // CAN_HPP
