#include "cmsis_os.h"
#include "data_interfaces/can/can.hpp"

// ============================ fdcan3 接收回调 ============================
// 瓴控电机的反馈帧 CAN ID = 0x140 + 电机ID(手册 P5), 数据域[0]为指令字, 与指令帧同布局
//   MHF9025(ID=1) -> 0x141, MG4005(ID=2) -> 0x142
//
// 每收到一帧就在这里更新该电机的反馈帧计时:
//   interval_us   与上一帧的间隔(丢帧会变大)
//   latency_us    从发出本周期指令到收到回复的时间(超过20ms按无效处理, 记 -1)

namespace
{
// 收到一帧后的计时更新
void update_timing(FrameTiming & t)
{
  const uint32_t now_us = micros();

  // 帧间隔(第一次收到只记时间戳, 不出间隔)
  if (t.last_rx_us != 0) {
    const uint32_t dt = now_us - t.last_rx_us;  // 无符号相减, 自动处理 8.9s 回绕
    t.interval_us = static_cast<float>(dt);
    if (t.interval_us > t.interval_max_us) t.interval_max_us = t.interval_us;
  }
  t.last_rx_us = now_us;

  // 发送 -> 反馈 延迟
  if (t.tx_pending) {
    const uint32_t dt = now_us - t.tx_us;
    t.latency_us = (dt < 20000U) ? static_cast<float>(dt) : -1.0f;
    t.tx_pending = false;
  }
}
}  // namespace

extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef * hfdcan, uint32_t RxFifo0ITs)
{
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == RESET) return;
  if (hfdcan != &hfdcan3) return;

  const uint32_t stamp_ms = osKernelSysTick();

  fdcan3.recv();

  if (fdcan3.rx_id == lk_mhf9025.rx_id) {
    lk_mhf9025.read(fdcan3.rx_data, stamp_ms);
    lk_mhf9025_rx_count++;
    update_timing(timing_mhf9025);
  }
  else if (fdcan3.rx_id == lk_mg4005.rx_id) {
    lk_mg4005.read(fdcan3.rx_data, stamp_ms);
    lk_mg4005_rx_count++;
    update_timing(timing_mg4005);
  }
}

// 总线关闭后清除 CCCR.INIT, 让 FDCAN 重新参与总线
extern "C" void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef * hfdcan, uint32_t ErrorStatusITs)
{
  if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) == RESET) return;

  CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
}
