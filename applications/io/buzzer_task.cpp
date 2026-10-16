#include "cmsis_os.h"
#include "io/buzzer/buzzer.hpp"

namespace
{
// TIM12 的定时器时钟 = 240MHz(APB1 定时器时钟)
sp::Buzzer buzzer(&htim12, TIM_CHANNEL_2, 240e6f);
}  // namespace

// 上电提示音: 两声短鸣, 之后保持静默
extern "C" void buzzer_task()
{
  buzzer.set(4000.0f, 0.5f);

  for (int i = 0; i < 2; i++) {
    buzzer.start();
    osDelay(80);
    buzzer.stop();
    osDelay(80);
  }

  while (true) {
    osDelay(1000);
  }
}
