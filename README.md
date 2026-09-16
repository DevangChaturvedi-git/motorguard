# MotorGuard

**A load-agnostic protection and control engine for STM32, built and validated on an STM32F103C8 (Blue Pill) in Wokwi.**

Submitted by: Devang Chaturvedi — 23BCT0151 — VIT Vellore
AENEXZ Embedded Systems Capstone — Project 2 (Industrial Motor Protection Controller)

---

## What this actually is

Not a motor controller. A **generic protection and control engine**: plug in any electrical
load through a small abstraction layer, and it (1) drives that load toward a requested
setpoint, continuously, without ever asking it to sustain more than its continuous-safe
rating, and (2) watches every relevant channel for conditions that would damage it, cutting
power immediately and safely when they occur — then requires an explicit human approval
before it will try again, rather than guessing.

Built as three layers, deliberately not more:

```
   setpoint
      |
      v
 +-------------+      +-------------+
 |   LAYER 1   |----->|   LAYER 3   |------> PWM (TIM1_CH1)
 |  Maintain   |clamp |  Proactive  |
 |  Operation  |      |   Limiter   |
 +-------------+      +-------------+
                              ^
                              | (shares one descriptor table)
                              |
                      +-------------+
                      |   LAYER 2   |  <-- runs every tick, every state,
                      |  Reactive   |      independent of what 1/3 are doing
                      | Protection  |
                      +-------------+
```

- **Layer 1 — Maintain Operation.** PI + feedforward, fixed-point, running on a fixed 5ms
  tick fed asynchronously from the plant's speed reading (not tied to the raw feedback rate —
  see `control.h` for why that matters for stability, not just energy use).
- **Layer 3 — Proactive Limiter.** Sits between Layer 1's output and the PWM register.
  Clamps commanded duty to whatever the continuous-safe current rating implies, forever —
  not just during a fault. Derives that ceiling from the same channel descriptor table Layer
  2 uses, so the two layers can never quietly disagree about what "safe" means.
- **Layer 2 — Reactive Protection.** Table-driven fusion engine: hysteresis, a
  confidence-counter debounce, an I²t (inverse-time) accumulator with decay, sensor
  plausibility checks, a dedicated stall correlation detector, and lockout escalation after
  repeated trips. Runs every tick regardless of supervisor state.

A **supervisor state machine** ties all three together and owns recovery/approval:

```
POST -> IDLE -> [AWAITING_SETPOINT_APPROVAL ->] RAMPING -> RUNNING
                                                    |
                                                    v
                                                  FAULT -> RECOVERY_PENDING -> RAMPING (soft re-arm)
                                                    |
                                                    v (repeated trips)
                                                 LOCKOUT
```

A power cycle is never an approval: if the last saved state was `FAULT`, `RECOVERY_PENDING`,
`LOCKOUT`, or `SENSOR_FAULT`, the supervisor comes back up latched in that same state, read
from a dedicated flash page, not a fresh `IDLE`.

## Why this project, and where the ideas actually come from

The design borrows deliberately from real protective-relay engineering (ANSI/IEEE C37.2
device numbering — 50 instantaneous overcurrent, 51 inverse-time overcurrent, 49 thermal, 86
lockout), not from novelty. Inverse-time overcurrent protection is over a century old; a
generic multi-load protective relay is an entire mature industry (Omron, SEL, Eaton, ABB all
sell exactly this). The honest framing throughout this project — in code comments and here —
is "applying real protective-relay-grade engineering to a $2 chip," not "inventing something
the market doesn't have." Worth being upfront about that rather than overselling it.

## Real, load-bearing engineering decisions (and where they came from)

- **I²t with decay, not a fixed threshold or dumb delay.** `protection.c`'s `check_i2t()`.
  Tuned against a native (x86, non-firmware) test harness in `test/i2t_tune.c` before any of
  these numbers touched real PWM output — see "Validation" below.
- **Hardware kill switch via TIM1's BKIN pin**, not just a software fault ISR. In a real
  build this is driven by a discrete comparator watching the current-sense shunt — Wokwi's
  part library has no op-amp/comparator primitive, so in this simulation BKIN is driven
  directly by a pushbutton standing in for "the comparator just fired." Electrically
  identical at the pin either way. `TIM1->BDTR` has `AOE=0`: a break event requires an
  explicit `pwm_reenable_output()` call afterwards, never a silent auto-resume.
- **Boot-time state latch.** `flash_persist.c` + `supervisor_init()`. A dedicated last flash
  page (kept out of the linker's `FLASH` region on purpose — see the `.ld` file) holds the
  last supervisor state, so unplugging and replugging the device can't be used to bypass an
  unresolved fault.
- **Conditional-integration anti-windup**, freezing the PI integrator on *any* cause of
  output saturation (Layer 3 clamp, or natural 0%/100% pinning) — not just during a fault
  latch. `control.c`.
- **Feedback sanity clamp ahead of the PI math** (`control.c`) as a fast, lightweight
  first line of defense, distinct from and faster than Layer 2's slower, more careful
  multi-sample sensor plausibility confirmation (`channels.h`'s `implausible_confidence_required`).

## Simulation limitations, stated plainly

Checked directly against Wokwi's own documentation before committing to this architecture,
not discovered by trial and error mid-build:

| What was planned | What's actually true in Wokwi's Blue Pill sim | What this firmware does instead |
|---|---|---|
| ADC1 + DMA continuous sampling | **DMA is not implemented** | Interrupt-driven, one channel at a time, round-robin (`adc_driver.c`) |
| Independent watchdog (IWDG) | **IWDG is not implemented**; WWDG is | WWDG instead — its windowed nature also catches a runaway-fast loop, which IWDG can't (`wwdg_driver.c`) |
| Real DC motor physics | **No such part exists** in Wokwi's library | A first-order plant model living entirely in firmware (`plant_model.c`) — the same technique used for HIL testing and digital twins, not a shortcut passed off as real hardware |
| Discrete comparator driving BKIN | **No op-amp/comparator part exists** | A pushbutton drives the physical BKIN pin directly — same electrical behaviour at that pin, see `pin_map.h` |
| SysTick as the fixed 5ms control tick | **Went silent past the boot line for 18+ real seconds in testing**, with no further output at all | Tried TIM2's update interrupt, then DWT->CYCCNT polling, then a plain busy-wait loop as increasingly hardware-independent tick sources — all three produced the same silence, which correctly proved the tick source was never the actual problem |
| WWDG | Initially suspected as the cause (a reset loop was observed at one point, and the timing was genuinely too tight) | Widening its margin didn't fix the underlying silence, and disabling it entirely didn't either — it's left disabled in this build (see the note in `main.c`). It was a real, separate issue worth fixing, but a red herring for *this* bug specifically |
| The actual root cause | **This simulator's Cortex-M3 core does not correctly execute an indirect function call through a pointer stored in a struct** — found by bisecting `supervisor_tick()` with numbered checkpoint prints on the first tick: every direct function call up to that point worked, and execution died on the exact line calling a per-channel `read_fn()` pointer, with zero fault-handler output, silently | `channels.h`/`channels.c` no longer store a function pointer per channel. `channel_read_value(id)` is a plain `switch` with direct calls instead — same external behavior, re-verified against the native test harness with identical results, no indirect call anywhere in the path |

## Validation

`test/i2t_tune.c` is a native (x86, plain `gcc`) build of `plant_model.c` + `channels.c` +
`protection.c` — no firmware, no ARM toolchain, just the actual protection math run against
six scenarios before any of it touched real PWM output:

| Scenario | Result |
|---|---|
| Normal run, 60% duty, no faults | Never trips, current settles well under the continuous limit |
| Normal run, 100% duty, no faults | I²t on current rises briefly during startup inrush, self-clears by tick 60 as speed builds up — no trip |
| Genuine stall at 100% duty | Trips via the dedicated stall correlation detector in ~200ms — faster than waiting for I²t, appropriately so for a real safety condition |
| Injected overload (not a stall) | Trips via the absolute-limit software backstop within 2 ticks |
| Brief graze (100% duty for 30 ticks, then back to 50%) | Does **not** trip — accumulator rises, then decays back to 0 |
| Heavy sustained non-stalling load | Trips at ~3.6s — via temperature's own absolute limit reaching its ceiling before current's I²t did. This is the same "composite protection profile: whichever curve is lesser wins" principle real multi-curve protective relays use, not a bug |

An early version of the plant model had temperature blowing past its absolute limit within
~15 ticks of *completely normal* startup — a thermal-constant scaling bug caught by this
harness before it ever reached firmware, not by eyeballing the equations.

The full firmware (all 15 source files, real ST CMSIS headers, real startup/linker scripts)
builds clean with `arm-none-eabi-gcc` — zero warnings beyond the expected nano.specs syscall
stub notices — at 7.2KB flash / 2.2KB RAM, comfortably inside the part's 64KB/20KB.

## Building

```
make            # produces motorguard.elf / .bin / .hex
```

Then open this folder in Wokwi (or push to GitHub and open via the Wokwi VS Code extension /
wokwi CI) — `wokwi.toml` points at `motorguard.elf` directly.

## Wiring

| Pin | Function |
|---|---|
| PA0 | Setpoint potentiometer |
| PA1 | Load/disturbance potentiometer |
| PA4 | LOCKOUT LED |
| PA8 | PWM output (TIM1_CH1) |
| PA9 | UART1 TX — diagnostics, 115200 8N1 |
| PB5 | APPROVE button |
| PB6 | INJECT STALL button (hold) |
| PB7 | INJECT OVERLOAD button (hold) |
| PB8 | INJECT SENSOR FAULT button (hold) |
| PB12 | TIM1 BKIN — hardware trip test button (stands in for a real comparator) |
| PB13/14/15 | RUN / WARN / FAULT LEDs |
| PC13 | Onboard heartbeat LED, 1Hz |

## Known limitations / honest roadmap

Same pattern as documenting a "planned, not built" item rather than glossing over it:

- **`plant_model_step()` forces speed to hard-zero whenever commanded duty is zero** (added under deadline pressure) — this simulator was producing an implausible speed reading (pinned near/at its clamp ceiling) from the very first step after a confirmed-zeroed init, with duty verified at exactly 0 throughout. Not reproducible on a native x86 rebuild of the identical file, and not producible by hand-deriving the equations either — root cause genuinely unresolved. This override neutralizes the visible symptom without touching any validated protection logic (confirmed against the native test harness, identical results).
- **The boot-time POST plausibility self-test is disabled** (`supervisor.c`) — it was spuriously failing on every single cold boot in this simulator, landing the system in `SENSOR_FAULT` before it ever had a chance to run, even with the plant model's internal state confirmed (via direct instrumentation) to be genuinely zeroed at that exact point. Root cause not resolved before submission — worth real investigation with proper hardware debugging tools (this simulator's VS Code extension does support GDB) rather than print-statement bisection under deadline pressure. Layers 2 and 3 — the reactive fusion engine and the proactive limiter, which is to say the actual safety architecture — are separate code paths, unaffected, and still fully functional; this only removes the one-shot sanity check that ran before ever leaving `IDLE`.
- **WWDG is currently disabled** (see `main.c`) after ruling it out as the cause of the indirect-call bug above — the timing work done on it (`wwdg_driver.c`) is real and still correct, it just isn't armed in this build. Re-enable it (uncomment the one line in `main.c`) once satisfied it doesn't interact badly with anything else — it wasn't the problem, but it also hasn't been re-validated as armed since.
- **BLE/server connectivity (Layer 5) is architecture-only tonight.** The design calls for a
  companion ESP32 handling BLE notifications and recovery-approval-over-network, talking to
  the STM32 over UART, with the same fault-isolation reasoning as a prior dual-node audio
  project. Not built in this submission — the STM32 side never blocks on it either way.
- **Flash persistence erases and rewrites the whole page on every transition** — fine for one
  evening, not for continuous real service. A real deployment should apply a RAM-accumulation
  + periodic-checkpoint wear-leveling strategy instead, the same pattern used in a prior
  always-on audio appliance project of mine.
- **LOCKOUT is terminal until reset** in this build — a real version would want a distinct,
  harder-to-trigger unlock path (e.g. a long-press or second confirmation), not just an
  omitted escape hatch.
- **WWDG window timing is computed from the RM0008 formula, not bench-verified on real
  silicon** — worth re-checking with a logic analyzer once this is ever flashed to physical
  hardware.
- Lockout's rolling trip-count window is tracked against the tick counter, which resets on
  reboot — a real deployment would want an RTC or a persisted monotonic counter so the window
  survives a power cycle the same way the fault latch itself does.

## References

Applying, not inventing: IEEE C37.2 device numbering (50/51/49/86); I²t inverse-time
protection theory (in use in motor overload relays for over a century); TIM1's break-input
feature is ST's own documented mechanism for exactly this purpose, not a novel technique.
