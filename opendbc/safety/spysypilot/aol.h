#pragma once

#include "opendbc/safety/spysypilot/aol_types.h"

// Defined globally in safety.h — AOL reads this to decide if the feature is enabled
extern int alternative_experience;

// Global definitions
bool lateral_allowed = false;   // main steering permission flag read by lateral.h
bool lfa_button_press = false;  // set by hyundai / hyundai_canfd rx_hook on LFA button press
AolState aol_state;

// aol_init — reset all AOL state to off.
// Called from set_safety_hooks() on every safety mode change.
static inline void aol_init(void) {
  lateral_allowed = false;
  lfa_button_press = false;

  aol_state.lfa_button.pressed      = false;
  aol_state.lfa_button.last_pressed = false;
  aol_state.lfa_button.rising_edge  = false;
  aol_state.lfa_button.falling_edge = false;

  aol_state.acc_main.current      = false;
  aol_state.acc_main.previous     = false;
  aol_state.acc_main.rising_edge  = false;
  aol_state.acc_main.falling_edge = false;

  aol_state.steering_disengage.current      = false;
  aol_state.steering_disengage.previous     = false;
  aol_state.steering_disengage.rising_edge  = false;
  aol_state.steering_disengage.falling_edge = false;

  aol_state.system_active  = false;
  aol_state.last_disengage = AOL_DISENGAGE_NONE;
}

// aol_on_lag — immediately revoke lateral permission when a safety-critical
// message lags. Called from safety_tick() alongside mads_exit_controls().
static inline void aol_on_lag(void) {
  if (lateral_allowed) {
    lateral_allowed = false;
    aol_state.last_disengage = AOL_DISENGAGE_LAG;
  }
}

// aol_update — core AOL decision function. Called from stock_ecu_check() on
// every relevant CAN message so edges are detected as soon as they occur.
//
// acc_main         : whether the car's ACC main switch is currently armed
// steering_disengage : whether the car has flagged a steering disengage/fault
//
// Activation (sets lateral_allowed = true):
//   - Rising edge on the physical LFA/LKAS button  (lfa_button_press set by rx_hook)
//   - Rising edge on acc_main                       (ACC system just armed)
//
// Deactivation (clears lateral_allowed):
//   - Falling edge on acc_main   (ACC main turned off)
//   - Rising edge on steering_disengage (EPS fault / car disengagement)
//   - AOL feature disabled via alternative_experience
//   - Message lag  (handled separately in aol_on_lag)
//
// REMAIN_ACTIVE: brake / gas presses do NOT affect lateral_allowed.
static inline void aol_update(bool acc_main, bool steering_disengage) {
  // Check if the AOL feature is enabled
  aol_state.system_active = (alternative_experience & ALT_EXP_AOL_ENABLE) != 0;

  // If the feature is disabled but we're still allowing lateral, revoke it
  if (!aol_state.system_active) {
    if (lateral_allowed) {
      lateral_allowed = false;
      aol_state.last_disengage = AOL_DISENGAGE_DISABLED;
    }
    return;
  }

  // --- Edge detection: acc_main ---
  aol_state.acc_main.current      = acc_main;
  aol_state.acc_main.rising_edge  = aol_state.acc_main.current && !aol_state.acc_main.previous;
  aol_state.acc_main.falling_edge = !aol_state.acc_main.current && aol_state.acc_main.previous;

  // --- Edge detection: steering_disengage ---
  aol_state.steering_disengage.current      = steering_disengage;
  aol_state.steering_disengage.rising_edge  = aol_state.steering_disengage.current && !aol_state.steering_disengage.previous;
  aol_state.steering_disengage.falling_edge = !aol_state.steering_disengage.current && aol_state.steering_disengage.previous;

  // --- Edge detection: LFA button (written by hyundai rx_hook each frame) ---
  aol_state.lfa_button.pressed      = lfa_button_press;
  aol_state.lfa_button.rising_edge  = aol_state.lfa_button.pressed && !aol_state.lfa_button.last_pressed;
  aol_state.lfa_button.falling_edge = !aol_state.lfa_button.pressed && aol_state.lfa_button.last_pressed;

  // --- Activation: LFA button pressed OR acc_main just armed ---
  if (aol_state.lfa_button.rising_edge || aol_state.acc_main.rising_edge) {
    lateral_allowed = true;
  }

  // --- Deactivation: acc_main turned off ---
  if (aol_state.acc_main.falling_edge) {
    lateral_allowed = false;
    aol_state.last_disengage = AOL_DISENGAGE_ACC_MAIN_OFF;
  }

  // --- Deactivation: car reported steering fault / disengage ---
  if (aol_state.steering_disengage.rising_edge) {
    lateral_allowed = false;
    aol_state.last_disengage = AOL_DISENGAGE_STEERING_FAULT;
  }

  // Save current values as previous for next cycle
  aol_state.acc_main.previous           = aol_state.acc_main.current;
  aol_state.steering_disengage.previous = aol_state.steering_disengage.current;
  aol_state.lfa_button.last_pressed     = aol_state.lfa_button.pressed;
}
