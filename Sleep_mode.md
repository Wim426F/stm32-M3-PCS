# Sleep Mode Implementation

## Overview
Added low-power Stop mode support with CAN wake-up capability. The MCU enters Stop mode after 5 minutes in MOD_OFF, reducing current draw from ~50mA to ~2-3mA average (due to IWDG waking every 2 seconds).

---

## hwinit.cpp

### Added includes (lines 32-34)
```cpp
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/f1/bkp.h>
```

### Added functions (lines 152-217)
| Function | Purpose |
|----------|---------|
| `enable_backup_domain()` | Enables PWR/BKP clocks and write access to backup registers |
| `set_sleep_flag()` | Writes 0xBEEF to BKP_DR1 |
| `has_sleep_flag()` | Returns true if BKP_DR1 contains 0xBEEF |
| `clear_sleep_flag()` | Clears BKP_DR1 |
| `was_iwdg_reset()` | Returns true if IWDG caused the last reset |
| `clear_reset_flags()` | Clears RCC_CSR reset flags |
| `configure_exti_wakeup()` | Enables GPIOB/AFIO clocks, configures EXTI8 (PB8/CAN_RX) for falling-edge wake |
| `enter_stop_mode()` | Enters Stop mode with low-power regulator |

---

## main.cpp

### Added includes (lines 27-29)
```cpp
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/cm3/scb.h>
```

### Added variables (lines 71, 73)
```cpp
static uint16_t sleepCountdown = 0;
#define SLEEP_TIMEOUT_SEC 300  // 5 minutes before entering sleep
```

### Added EXTI ISR (lines 391-394)
```cpp
extern "C" void exti9_5_isr(void)
{
   exti_reset_request(EXTI8);
}
```

---

Wake sources:
- CAN bus activity (EXTI on PB8) → full wake, normal operation
- IWDG timeout → quick check, back to sleep if flag set
