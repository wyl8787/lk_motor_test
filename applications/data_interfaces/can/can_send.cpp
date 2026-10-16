#include "cmsis_os.h"
#include "data_interfaces/can/can.hpp"

// ============================ fdcan3 发送 ============================
// 沿用原工程的写法: can_send_freq 相位计数器分频 + send_motor_xxx() 组帧发送
// 发送周期 1ms -> 1000Hz(与 control_task 的 CTRL_PERIOD_MS 一致)

uint16_t can_send_freq = 0;

// 上电使能帧(0x88)持续的时长, 单位: ms
constexpr uint16_t ENABLE_TIME_MS = 200;

// DWT 周期计数器: 打开后 DWT->CYCCNT 每个 CPU 周期 +1(480MHz)
static void dwt_init()
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t micros() { return DWT->CYCCNT / (SystemCoreClock / 1000000U); }

void can_start()
{
  dwt_init();

  // 过滤器: 掩码为0 => 全部标准帧都收进 FIFO0
  FDCAN_FilterTypeDef filter = {};
  filter.IdType = FDCAN_STANDARD_ID;
  filter.FilterIndex = 0;
  filter.FilterType = FDCAN_FILTER_MASK;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter.FilterID1 = 0x000;
  filter.FilterID2 = 0x000;
  HAL_FDCAN_ConfigFilter(&hfdcan3, &filter);  // dismiss return

  // 未匹配过滤器的帧也收进 FIFO0(调试时可以看到总线上的其它帧)
  HAL_FDCAN_ConfigGlobalFilter(
    &hfdcan3, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_REJECT_REMOTE,
    FDCAN_REJECT_REMOTE);  // dismiss return

  fdcan3.start();
}

void can_send(const sp::LK_Motor & motor)
{
  motor.write(fdcan3.tx_data);
  fdcan3.send(motor.tx_id);
}

void can_send_open(const sp::LK_Motor & motor)
{
  motor.write_open(fdcan3.tx_data);
  fdcan3.send(motor.tx_id);
}

void can_send_stop(const sp::LK_Motor & motor)
{
  motor.write_stop(fdcan3.tx_data);
  fdcan3.send(motor.tx_id);
}

void can_send_state1(const sp::LK_Motor & motor)
{
  motor.write_state1(fdcan3.tx_data);
  fdcan3.send(motor.tx_id);
}

void can_send_state2(const sp::LK_Motor & motor)
{
  motor.write_state2(fdcan3.tx_data);
  fdcan3.send(motor.tx_id);
}

// 单次命令: 按电机当前缓存的增量把 0xA7 帧锁存下来(只发这一次)
void one_shot_arm_angle_increment(OneShot & one_shot, const sp::LK_Motor & motor)
{
  motor.write_angle_increment(one_shot.data);
  one_shot.armed = true;
  one_shot.sent = false;
}

// 发一台电机的帧: 单次命令优先, 其余情况发 control_task 缓存的指令帧
// 发出去之前记一个 µs 时间戳, 等回复到达时在 can_recv.cpp 里算"发送->反馈"延迟
static void send_frames(const sp::LK_Motor & motor, OneShot & one_shot, FrameTiming & timing)
{
  const uint32_t now_us = micros();

  // 单次命令: 只发一帧, 然后安静 hold_us 等动作走完, 再恢复普通指令帧
  if (one_shot.armed) {
    if (!one_shot.sent) {
      for (uint8_t i = 0; i < 8; i++) fdcan3.tx_data[i] = one_shot.data[i];
      timing.tx_us = now_us;
      timing.tx_pending = true;
      fdcan3.send(motor.tx_id);
      one_shot.sent = true;
      one_shot.sent_us = now_us;
      return;
    }
    if ((now_us - one_shot.sent_us) < one_shot.hold_us) return;  // 安静期, 什么都不发
    one_shot.armed = false;
    one_shot.sent = false;
  }

  timing.tx_us = now_us;
  timing.tx_pending = true;
  can_send(motor);
}

// 两台电机的指令帧: 具体内容由 control_task 缓存的 cmd_* 决定(匀速/停转/转圈)
void send_motor_mhf9025() { send_frames(lk_mhf9025, one_shot_mhf9025, timing_mhf9025); }

void send_motor_mg4005() { send_frames(lk_mg4005, one_shot_mg4005, timing_mg4005); }

// clang-format off
extern "C" void can_task()
{
  // 启动延时
  osDelay(2000);
  can_start();

  // 上电先连续发一段使能帧(0x88), 让电机进入使能状态
  for (uint16_t i = 0; i < ENABLE_TIME_MS; i++) {
    if (i % 5 == 0) {
      can_send_open(lk_mhf9025);
      can_send_open(lk_mg4005);
    }
    osDelay(1);
  }

  while (1) {
    can_send_freq = (can_send_freq + 1) % 700;

    uint32_t phase_100 = can_send_freq % 100;  // 10Hz 周期(计数按1ms一个数)

    // 1000Hz: 两个电机各一帧指令帧
    // (对应原工程 chassis_task 里 send_motor_3508()/send_motor_6020() 的位置)
    send_motor_mhf9025();
    send_motor_mg4005();

    // 10Hz: 状态1查询帧(温度/电压/电机状态/错误标志)
    if (phase_100 == 30) {
      can_send_state1(lk_mhf9025);
      can_send_state1(lk_mg4005);
    }

    osDelay(lk_ctrl::CTRL_PERIOD_MS);
  }
}
// clang-format on
