#include "cmsis_os.h"
#include "io/ws2812/ws2812.hpp"

extern "C" void led_task()
{
  sp::WS2812 ws2812(&hspi6);

  while (true) {
    for (uint8_t g = 0; g < 10; g++) {
      ws2812.set(0, g, 0);
      osDelay(100);
    }

    for (uint8_t g = 10; g > 0; g--) {
      ws2812.set(0, g, 0);
      osDelay(100);
    }
  }

  return;
}
