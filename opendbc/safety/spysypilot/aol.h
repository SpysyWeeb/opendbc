#pragma once

#include "opendbc/safety/spysypilot/aol_types.h"

#include "opendbc/safety/declarations.h"

// Global definitions
bool lateral_allowed = false;    // main steering permission flag read by lateral.h
bool aol_rx_checks_invalid = false;
bool lfa_button_press = false;   // set by hyundai / hyundai_canfd rx_hook on LFA button press
bool main_button_press = false;  // set by hyundai / hyundai_canfd rx_hook on cruise main button press
AolState aol_state;
bool heartbeat_engaged_aol = false;
uint32_t heartbeat_engaged_aol_mismatches = 0U;

// aol_init — reset all AOL state to off.
// Called from set_safety_hooks() on every safety mode change.
static inline void aol_init(void) {
  lateral_allowed = false;
  aol_rx_checks_invalid = false;
  lfa_button_press = false;
  main_button_press = false;
  heartbeat_engaged_aol = false;
  heartbeat_engaged_aol_mismatches = 0U;

  aol_state.lfa_button.pressed      = false;
  aol_state.lfa_button.last_pressed = false;
  aol_state.lfa_button.rising_edge  = false;
  aol_state.lfa_button.falling_edge = false;

  aol_state.main_button.pressed      = false;
  aol_state.main_button.last_pressed = false;
  aol_state.main_button.rising_edge  = false;
  aol_state.main_button.falling_edge = false;

  aol_state.acc_main.current      = false;
  aol_state.acc_main.previous     = false;
  aol_state.acc_main.rising_edge  = false;
  aol_state.acc_main.falling_edge = false;

  aol_state.op_controls_allowed.current      = false;
  aol_state.op_controls_allowed.previous     = false;
  aol_state.op_controls_allowed.rising_edge  = false;
  aol_state.op_controls_allowed.falling_edge = false;

  aol_state.steering_fault.current      = false;
  aol_state.steering_fault.previous     = false;
  aol_state.steering_fault.rising_edge  = false;
  aol_state.steering_fault.falling_edge = false;

  aol_state.system_active  = false;
  aol_state.activation_pending = false;
  aol_state.last_disengage = AOL_DISENGAGE_NONE;
}

// aol_on_lag — immediately revoke lateral permission when a safety-critical
// message lags. Called from safety_tick() to clear lateral_allowed on OP crash.
static inline void aol_on_lag(void) {
  aol_rx_checks_invalid = true;
  aol_state.activation_pending = false;
  if (lateral_allowed) {
    lateral_allowed = false;
    aol_state.last_disengage = AOL_DISENGAGE_LAG;
  }
}

// aol_update — core AOL decision function. Called from safety_rx_hook() on
// every relevant CAN message so edges are detected as soon as they occur.
//
// acc_main         : whether the car's ACC main switch is currently armed
// steering_disengage : whether the car has flagged a steering disengage/fault
//
// CAN activation candidates (do NOT alone grant lateral permission):
//   - Rising edge on the physical LFA/LKAS button  (lfa_button_press set by rx_hook)
//   - Rising edge on the cruise main button         (main_button_press set by rx_hook;
//                                                     used as the AOL toggle on cars,
//                                                     like the Palisade, with no LFA button)
//   - Rising edge on acc_main                       (ACC system just armed)
//   - Rising edge on controls_allowed               (full engagement)
// A candidate waits for a true host SOL heartbeat, since CAN reaches safety
// before the host processes the button and sends its request. False heartbeats
// never grant permission, even if new CAN activation edges arrive afterwards.
// Do not toggle every physical button here: safety does not receive the host's
// HAS_LDA_BUTTON capability, which determines whether mainCruise is ignored.
//
// Deactivation (clears lateral_allowed):
//   - Falling edge on acc_main   (ACC main turned off)
//   - Rising edge on steering_disengage (EPS fault / car disengagement)
//   - AOL feature disabled via alternative_experience
//   - Message lag  (handled separately in aol_on_lag)
//   - Explicit host off heartbeat (handled synchronously in USB receive)
//
// REMAIN_ACTIVE: brake / gas presses do NOT affect lateral_allowed.
static inline void aol_update(bool acc_main, bool steering_fault) {
  // Check if the AOL feature is enabled
  const bool supported = (current_safety_mode == SAFETY_HYUNDAI) || (current_safety_mode == SAFETY_HYUNDAI_LEGACY) ||
                         (current_safety_mode == SAFETY_HYUNDAI_CANFD);
  aol_state.system_active = supported && ((alternative_experience & ALT_EXP_AOL_ENABLE) != 0);

  // If the feature is disabled but we're still allowing lateral, revoke it
  if (!aol_state.system_active) {
    aol_state.activation_pending = false;
    if (lateral_allowed) {
      lateral_allowed = false;
      aol_state.last_disengage = AOL_DISENGAGE_DISABLED;
    }
  } else {
    // --- Edge detection: acc_main ---
    aol_state.acc_main.current      = acc_main;
    aol_state.acc_main.rising_edge  = aol_state.acc_main.current && !aol_state.acc_main.previous;
    aol_state.acc_main.falling_edge = !aol_state.acc_main.current && aol_state.acc_main.previous;

    // --- Edge detection: openpilot's own controls_allowed (full engagement) ---
    aol_state.op_controls_allowed.current      = controls_allowed;
    aol_state.op_controls_allowed.rising_edge  = aol_state.op_controls_allowed.current && !aol_state.op_controls_allowed.previous;
    aol_state.op_controls_allowed.falling_edge = !aol_state.op_controls_allowed.current && aol_state.op_controls_allowed.previous;

    // --- Edge detection: steering_disengage ---
    aol_state.steering_fault.current      = steering_fault;
    aol_state.steering_fault.rising_edge  = aol_state.steering_fault.current && !aol_state.steering_fault.previous;
    aol_state.steering_fault.falling_edge = !aol_state.steering_fault.current && aol_state.steering_fault.previous;

    // --- Edge detection: LFA button (written by hyundai rx_hook each frame) ---
    aol_state.lfa_button.pressed      = lfa_button_press;
    aol_state.lfa_button.rising_edge  = aol_state.lfa_button.pressed && !aol_state.lfa_button.last_pressed;
    aol_state.lfa_button.falling_edge = !aol_state.lfa_button.pressed && aol_state.lfa_button.last_pressed;

    // --- Edge detection: cruise main button (written by hyundai rx_hook each frame) ---
    aol_state.main_button.pressed      = main_button_press;
    aol_state.main_button.rising_edge  = aol_state.main_button.pressed && !aol_state.main_button.last_pressed;
    aol_state.main_button.falling_edge = !aol_state.main_button.pressed && aol_state.main_button.last_pressed;

    // --- Activation: LFA button, cruise main button, acc_main, or controls_allowed rising edge ---
    if (aol_state.lfa_button.rising_edge || aol_state.main_button.rising_edge ||
        aol_state.acc_main.rising_edge || aol_state.op_controls_allowed.rising_edge) {
      aol_state.activation_pending = !aol_rx_checks_invalid;
    }
    if (aol_state.activation_pending && heartbeat_engaged_aol && !aol_rx_checks_invalid) {
      lateral_allowed = true;
      aol_state.activation_pending = false;
    }

    // --- Deactivation: acc_main turned off ---
    if (aol_state.acc_main.falling_edge) {
      lateral_allowed = false;
      aol_state.activation_pending = false;
      aol_state.last_disengage = AOL_DISENGAGE_ACC_MAIN_OFF;
    }

    // --- Deactivation: car reported steering fault / disengage ---
    if (steering_fault) {
      lateral_allowed = false;
      aol_state.activation_pending = false;
      aol_state.last_disengage = AOL_DISENGAGE_STEERING_FAULT;
    }

    // Save current values as previous for next cycle
    aol_state.acc_main.previous           = aol_state.acc_main.current;
    aol_state.op_controls_allowed.previous = aol_state.op_controls_allowed.current;
    aol_state.steering_fault.previous = aol_state.steering_fault.current;
    aol_state.lfa_button.last_pressed     = aol_state.lfa_button.pressed;
    aol_state.main_button.last_pressed    = aol_state.main_button.pressed;
  }
}

// An explicit host off request is authoritative immediately, not after a
// mismatch grace period. Called by panda's USB heartbeat handler.
// cppcheck-suppress unusedFunction ; called by panda board/main_comms.h
static inline void aol_heartbeat_received(bool engaged) {
  heartbeat_engaged_aol = engaged;
  if (!engaged) {
    lateral_allowed = false;
    aol_state.last_disengage = AOL_DISENGAGE_HEARTBEAT_MISMATCH;
  }
}

// Defensive mismatch check at 1Hz. Normal explicit-off reception revokes above
// immediately, so it never relies on this grace period. Complete heartbeat
// loss is handled by panda's existing heartbeat_counter -> SILENT watchdog;
// the mode reset clears both the request and any pending CAN activation.
// cppcheck-suppress unusedFunction ; called by panda board/main.c
static inline void aol_heartbeat_engaged_check(void) {
  if (lateral_allowed && !heartbeat_engaged_aol) {
    heartbeat_engaged_aol_mismatches += 1U;
    if (heartbeat_engaged_aol_mismatches >= 3U) {
      lateral_allowed = false;
      aol_state.last_disengage = AOL_DISENGAGE_HEARTBEAT_MISMATCH;
    }
  } else {
    heartbeat_engaged_aol_mismatches = 0U;
  }
}
