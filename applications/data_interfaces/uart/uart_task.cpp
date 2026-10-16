#include "cmsis_os.h"
#include "data_interfaces/uart/uart_task.hpp"
#include "usart.h"

// ============================ 遥控器接收 ============================
// 达妙板: UART5 接 DBus 接收机, 用 ReceiveToIdle_IT 收 18 字节定长帧
// 调试信息统一走 plotter(huart7, 见 plotter_task.cpp), 这里不打印

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
  if (huart != &huart5) return;

  remote.update(Size, osKernelSysTick());
  remote.request();
}

// 接收出错后重新挂起接收, 否则遥控器会一直收不到数据
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
  if (huart == &huart5) remote.request();
}

extern "C" void uart_task()
{
  remote.request();

  while (true) {
    osDelay(10);
  }
}
