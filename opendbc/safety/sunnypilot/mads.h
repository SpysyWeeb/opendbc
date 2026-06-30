#pragma once

#include "opendbc/safety/sunnypilot/mads_declarations.h"

// Defined globally in safety.h
extern int alternative_experience;
extern bool controls_allowed;
extern bool vehicle_moving;
extern bool acc_main_on;
extern bool brake_pressed;
extern bool regen_braking;
extern bool steering_disengage;

// Global definitions (declared extern in mads_declarations.h)
ButtonState mads_button_press = MADS_BUTTON_NOT_PRESSED;
MADSState m_mads_state;
bool controls_allowed_lateral = false;

static inline void m_update_binary_state(BinaryStateTracking *state) {
  state->transition = (state->current && !state->previous) ? MADS_EDGE_RISING :
                      (!state->current && state->previous) ? MADS_EDGE_FALLING :
                      MADS_EDGE_NO_CHANGE;
  state->previous = state->current;
}

static inline void m_update_button_state(ButtonStateTracking *state) {
  if (state->current == MADS_BUTTON_UNAVAILABLE) {
    state->transition = MADS_EDGE_NO_CHANGE;
  } else {
    bool cur = (state->current == MADS_BUTTON_PRESSED);
    bool prev = (state->last == MADS_BUTTON_PRESSED);
    state->transition = (cur && !prev) ? MADS_EDGE_RISING :
                        (!cur && prev) ? MADS_EDGE_FALLING :
                        MADS_EDGE_NO_CHANGE;
  }
  state->last = state->current;
}

static inline void mads_exit_controls(const DisengageReason reason) {
  m_mads_state.current_disengage.pending_reasons = (DisengageReason)(m_mads_state.current_disengage.pending_reasons | reason);
  if (controls_allowed_lateral) {
    m_mads_state.current_disengage.active_reason = reason;
    m_mads_state.controls_requested_lateral = false;
    controls_allowed_lateral = false;
  }
}

static inline void m_mads_state_init(void) {
  m_mads_state.is_vehicle_moving = false;
  m_mads_state.acc_main.current = false;
  m_mads_state.acc_main.previous = false;
  m_mads_state.acc_main.transition = MADS_EDGE_NO_CHANGE;
  m_mads_state.mads_button.current = MADS_BUTTON_UNAVAILABLE;
  m_mads_state.mads_button.last = MADS_BUTTON_UNAVAILABLE;
  m_mads_state.mads_button.transition = MADS_EDGE_NO_CHANGE;
  m_mads_state.op_controls_allowed.current = false;
  m_mads_state.op_controls_allowed.previous = false;
  m_mads_state.op_controls_allowed.transition = MADS_EDGE_NO_CHANGE;
  m_mads_state.braking.current = false;
  m_mads_state.braking.previous = false;
  m_mads_state.braking.transition = MADS_EDGE_NO_CHANGE;
  m_mads_state.mads_steering_disengage.current = false;
  m_mads_state.mads_steering_disengage.previous = false;
  m_mads_state.mads_steering_disengage.transition = MADS_EDGE_NO_CHANGE;
  m_mads_state.current_disengage.active_reason = MADS_DISENGAGE_REASON_NONE;
  m_mads_state.current_disengage.pending_reasons = MADS_DISENGAGE_REASON_NONE;
  m_mads_state.system_enabled = false;
  m_mads_state.disengage_lateral_on_brake = false;
  m_mads_state.pause_lateral_on_brake = false;
  m_mads_state.controls_requested_lateral = false;
  controls_allowed_lateral = false;
}

static inline void m_update_control_state(void) {
  bool allowed = true;

  // Request triggers: rising edge on any activation signal
  if ((m_mads_state.acc_main.transition == MADS_EDGE_RISING) ||
      (m_mads_state.mads_button.transition == MADS_EDGE_RISING) ||
      (m_mads_state.op_controls_allowed.transition == MADS_EDGE_RISING)) {
    m_mads_state.controls_requested_lateral = true;
  }

  // Primary blockers
  if (m_mads_state.acc_main.transition == MADS_EDGE_FALLING) {
    mads_exit_controls(MADS_DISENGAGE_REASON_ACC_MAIN_OFF);
    allowed = false;
  }

  if (m_mads_state.mads_steering_disengage.transition == MADS_EDGE_RISING) {
    mads_exit_controls(MADS_DISENGAGE_REASON_STEERING_DISENGAGE);
    allowed = false;
  }

  // REMAIN_ACTIVE: disengage_lateral_on_brake and pause_lateral_on_brake are both false
  // so brake press does not affect controls_allowed_lateral

  // Enable controls if all conditions met
  if (allowed &&
      m_mads_state.system_enabled &&
      m_mads_state.controls_requested_lateral &&
      !controls_allowed_lateral) {
    m_mads_state.controls_requested_lateral = false;
    controls_allowed_lateral = true;
    m_mads_state.current_disengage.active_reason = MADS_DISENGAGE_REASON_NONE;
    m_mads_state.current_disengage.pending_reasons = MADS_DISENGAGE_REASON_NONE;
  }
}

static inline void mads_state_update(const bool op_vehicle_moving,
                                     const bool op_acc_main,
                                     const bool op_allowed,
                                     const bool is_braking,
                                     const bool _steering_disengage) {
  // Re-read alternative_experience flags on every update
  m_mads_state.system_enabled = (alternative_experience & ALT_EXP_ENABLE_MADS) != 0;
  m_mads_state.disengage_lateral_on_brake = (alternative_experience & ALT_EXP_MADS_DISENGAGE_LATERAL_ON_BRAKE) != 0;
  m_mads_state.pause_lateral_on_brake = (alternative_experience & ALT_EXP_MADS_PAUSE_LATERAL_ON_BRAKE) != 0;

  // If MADS is disabled by alternativeExperience, clear lateral control
  if (!m_mads_state.system_enabled && controls_allowed_lateral) {
    mads_exit_controls(MADS_DISENGAGE_REASON_NONE);
    return;
  }

  // Snapshot inputs
  m_mads_state.is_vehicle_moving = op_vehicle_moving;
  m_mads_state.acc_main.current = op_acc_main;
  m_mads_state.op_controls_allowed.current = op_allowed;
  m_mads_state.mads_button.current = mads_button_press;
  m_mads_state.braking.current = is_braking;
  m_mads_state.mads_steering_disengage.current = _steering_disengage;

  // Detect edges
  m_update_binary_state(&m_mads_state.acc_main);
  m_update_binary_state(&m_mads_state.op_controls_allowed);
  m_update_binary_state(&m_mads_state.braking);
  m_update_binary_state(&m_mads_state.mads_steering_disengage);
  m_update_button_state(&m_mads_state.mads_button);

  // Run state machine
  m_update_control_state();
}
