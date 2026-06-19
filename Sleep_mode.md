# Sleep Mode

## Overview
When the car is parked the PCS controller has nothing to do, but left running it draws
roughly 50 mA from the 12 V battery — enough to matter over days or weeks of standing.
Sleep mode puts the STM32 into **Stop mode** after a period of inactivity, cutting the
average draw to a few mA, and brings it back the moment the car comes alive again on the
CAN bus.

The whole design is shaped by one hard constraint and one hard requirement:

- **Constraint** — the bootloader enables the independent watchdog (IWDG), and once
  enabled it *cannot* be turned off. It is clocked from the LSI and keeps counting even in
  Stop mode, so it **will** reset the chip periodically whether we want it to or not.
- **Requirement** — a controller that runs the 12 V system must never be able to lock
  itself up. A firmware state that leaves it asleep and unresponsive, recoverable only by
  reflashing, is far worse than a slightly flat battery.

Rather than fight the watchdog, sleep mode uses it as a heartbeat, and is built on a single
safety rule: **never enter Stop mode until the firmware has fully booted into a recoverable
state.**

## The core idea
Sleep is only ever entered from the running 100 ms task — **never from the boot path**.
Every reset, whatever caused it (the watchdog firing during sleep, a CAN wake, or a power
glitch), therefore lands in `main()` and runs the *complete* initialisation — clocks, GPIO,
CAN, EXTI, scheduler — *before* any decision about sleeping is made. For about a second
after every boot the module is fully alive: it answers CAN, the serial terminal, and SWD.
Only then, if it is still idle, does it go back to sleep.

The consequence is that the device cannot be trapped. The worst case is that it wakes,
finds nothing to do, and sleeps again — it is always reachable within roughly a second of
any reset.

## Lifecycle

```
        Awake & active  (opmode = RUN / CHARGE)
                │   car parked → opmode = OFF
                ▼
        Idle countdown  (up to 5 min, fully awake)
                │   countdown expires
                ▼
        Set sleep flag → stretch IWDG → enter Stop (WFI)      ← ~2–3 mA
                │
                ├──  CAN edge on PB8 (EXTI8)  ──┐
                ├──  IWDG times out (~26 s)   ──┤
                ▼                               ▼
              Full reset → main() re-initialises everything
                │
                ▼
        Sleep flag set? ── no ──► full 5-min idle window  (fresh power-on)
                │ yes
                ▼
        Short ~1 s re-check window  (fully responsive)
                │
                ├──  RUN / CHARGE command arrives → clear flag → AWAKE
                └──  still idle → back to Stop
```

## Wake sources
| Source | What it is | Result |
|--------|-----------|--------|
| CAN activity | Falling edge on PB8 (CAN_RX) via EXTI8 | Returns from `WFI`, then `scb_reset_system()` for a clean, correct-clock boot |
| Watchdog | IWDG times out (~26 s) while asleep | Hardware reset |

Both paths converge on the same outcome: a full reboot. There is deliberately no
"resume where we left off" path — Stop mode drops the system clock back to the internal
HSI oscillator, so resetting is the clean way to come back at the correct speed.

## The sleep flag
A single magic value (`0xBEEF`) in backup register `BKP_DR1` records "we were asleep." It
lives in the VBAT-backed backup domain, so it survives the resets that punctuate sleep.

Its role is deliberately weak. It does **not** gate initialisation and it does **not**
trigger sleep — it only tells the boot code whether to use the short (~1 s) re-check window
or the full 5-minute idle timeout. It is *set* when the controller decides to sleep, and
*cleared* only when the car is genuinely back in use (`opmode` leaves `OFF`).

Because the flag can only ever *shorten the awake window*, a stale flag is harmless: if a
brown-out leaves `0xBEEF` in the register while the car is actually off, the controller just
runs one short wake cycle and goes back to sleep — which is what it should do anyway. This
is the failure the earlier design suffered from. That version used the flag together with
the reset-cause register to *skip* initialisation and dive straight back into Stop from the
boot path; a wrong reset-cause reading after a glitchy power event could then trap it
re-sleeping forever, recoverable only by reflashing.

## Watchdog stretching
During normal operation the IWDG runs at the bootloader's short (~2 s) period, giving full
protection against a hung firmware. Left at that period a sleeping controller would reset
every couple of seconds, wasting power on constant reboots.

So just before entering Stop, `enter_stop_mode()` stretches the IWDG to its maximum
(~26 s on the F1's LSI). The controller then sleeps in ~26 s blocks instead of ~2 s ones.
The stretch is never undone in software — and it does not need to be: every reset re-runs
the bootloader, which restores the short period. Full watchdog protection is therefore back
the instant the controller is awake and running, and is only ever relaxed while parked.

## Power behaviour
| State | Draw |
|-------|------|
| Awake (normal running) | ~50 mA |
| Asleep (Stop mode) | ~2–3 mA |
| Parked average | ~5 mA — awake ~1 s out of every ~26 s cycle |

That is roughly a 10× reduction in standing drain while keeping the controller fully
recoverable at all times.

## Recovery and bench development
Because every reset produces a ~1 s window of full responsiveness, the module is always
recoverable:

- A firmware update over CAN or SWD is picked up during the wake window.
- For SWD on the bench with no CAN traffic, use *connect-under-reset*, or simply flash
  within the first five minutes after power-up while it is still in its initial idle window.

## Tunables
| Name | Location | Meaning |
|------|----------|---------|
| `SLEEP_TIMEOUT_SEC` | `main.cpp` | Idle time in `OFF` before the first sleep (default 300 s) |
| `RESLEEP_WINDOW_TICKS` | `main.cpp` | Length of the post-wake re-check window, in 100 ms ticks (default 10 ≈ 1 s) |
| IWDG stretch period | `hwinit.cpp`, `enter_stop_mode()` | Maximum sleep interval between watchdog resets (default ~26 s) |

The boot-time flag check and the sleep decision both live in `main.cpp` (the decision in the
100 ms task); Stop-mode entry, EXTI wake configuration, and the watchdog and backup-domain
helpers live in `hwinit.cpp`.
