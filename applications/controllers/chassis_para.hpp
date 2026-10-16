#ifndef _CHASSIS_PARA_HPP
#define _CHASSIS_PARA_HPP

#include <cstdint>

#include "motor/lk_motor/lk_motor.hpp"

// ============================ 被测电机 ============================
// MHF9025 与 MG4005 都挂在 CAN3(FDCAN3, PD12=RX / PD13=TX)上同时控制
//   MHF9025: 电机ID = 1
//   MG4005 : 电机ID = 2
//
// 构造参数: LK_Motor(电机ID, 型号, 额外加装的行星减速箱减速比, 是否多圈)
//   电机ID: 1 ~ 32, 必须和电机里配置的 ID 一致(瓴控出厂默认 ID = 1, 可用上位机改)
//   型号:   sp::LK_Motors::MHF9025 / sp::LK_Motors::MG4005
//   减速比: 只填"额外加装"的行星减速箱; 型号自带的减速箱已包含在型号参数里
//           总减速比 = 型号自带减速比 × 该值
//   输出端量: angle[rad] / speed[rad/s] / torque[N·m] / current[A](电机侧)

// MHF9025: 直驱(无减速箱), 19位编码器, 堵转 5.8N·m / 6.9A
inline sp::LK_Motor lk_mhf9025(1, sp::LK_Motors::MHF9025, 1.0f);

// MG4005: 4005 电机 + 自带 10:1 行星减速箱, 输出端堵转 2.5N·m / 4A
inline sp::LK_Motor lk_mg4005(2, sp::LK_Motors::MG4005, 1.0f);

// -------------------- 反馈帧计数 --------------------
// 用于确认两个电机都在同一条总线上正常应答(在 can_recv.cpp 的中断里累加)
inline uint32_t lk_mhf9025_rx_count = 0;
inline uint32_t lk_mg4005_rx_count = 0;

// ============================ 控制参数 ============================
namespace lk_ctrl
{
// 控制/发送周期, 单位: ms
// 1ms -> 1000Hz: 两个电机各 1 帧指令 + 各 1 帧反馈 = 4000 帧/s,
// 一帧 8 字节标准帧约 112~135bit, 1Mbps 下总线负载约 45%~54%
constexpr uint32_t CTRL_PERIOD_MS = 1;

// ---------------------------- 控制量选择 ----------------------------
// false: 速度环 —— 匀速状态下发 RUN_SPEED_xxx, 停转状态下发 0(主动刹住)
// true : 力矩环 —— 匀速状态下发 torque_set_xxx, 停转状态下发 0(自由)
constexpr bool USE_TORQUE_CONTROL = 1;

// ---------------------------- 匀速状态 ----------------------------
// 目标转速(输出端), 单位: rad/s
constexpr float RUN_SPEED_MHF9025 = 13.0f;  // 额定 130rpm = 13.6 rad/s
constexpr float RUN_SPEED_MG4005 = 20.0f;   // 输出端额定 253rpm = 26.5 rad/s

// ---------------------------- 一次性转圈(核对编码器位宽) ----------------------------
// 左拨杆打到上档 -> 电机转这么多圈, 然后保持不动(可以反复打下来再打上去重测)
// 圈数按"电机侧/编码器侧"算:
//   1.0  -> 编码器正好扫过完整量程一次, 看 encoder_raw_max 就知道是多少位编码器
//           (MHF9025 是直驱, 输出轴也正好一圈; MG4005 是 10:1, 输出轴只转 36°)
//   10.0 -> 对 10:1 的 MG4005 才是输出轴整一圈
constexpr float TURN_TURNS = 1.0f;

// ---------------------------- 力矩设置接口 ----------------------------
// 直接填这里的值即可(输出端, N·m):
//   1) 力矩环: 把上面的 USE_TORQUE_CONTROL 改成 true, 匀速状态就会下发 torque_set_xxx
//   2) 也可以运行时改(例如调试器里改): lk_ctrl::torque_set_mhf9025 = 0.5f;
// 上限由 LK_Motor 内部按型号峰值电流限幅(MHF9025 2.79N·m / MG4005 输出端 1N·m)
inline float torque_set_mhf9025 = 0.3f;
inline float torque_set_mg4005 = 0.3f;
}  // namespace lk_ctrl

// ============================ 控制状态 ============================
// 由遥控器拨杆切换, 在 control_task 里更新
//   左拨杆 上      -> 一次性转圈(核对编码器位宽, 见 lk_ctrl::TURN_TURNS)
//   右拨杆 上      -> 匀速
//   右拨杆 中/下   -> 停转
// 没插遥控器 / 遥控器掉线 -> 停转(保护)
enum class Mode
{
  stop_mode,  // 停转
  run_mode,   // 匀速
  turn_mode   // 一次性转圈
};

inline Mode mode = Mode::stop_mode;

inline const char * mode_name(Mode m)
{
  switch (m) {
    case Mode::run_mode:
      return "RUN";
    case Mode::turn_mode:
      return "TURN";
    default:
      return "STOP";
  }
}

// ============================ 对外遥测 ============================
// control_task(1000Hz) 写, plot_task(100Hz) 读, 仅用于观察
struct Telemetry
{
  float time_s = 0.0f;
  float cmd_mhf9025 = 0.0f;  // 当前下发的指令值(速度 rad/s 或 力矩 N·m)
  float cmd_mg4005 = 0.0f;
  bool online_mhf9025 = false;
  bool online_mg4005 = false;
  uint32_t frame_mhf9025 = 0;  // 收到的反馈帧数
  uint32_t frame_mg4005 = 0;
};

inline Telemetry telemetry;

#endif  // _CHASSIS_PARA_HPP
