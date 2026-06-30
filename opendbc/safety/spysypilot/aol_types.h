#pragma once

// ALT_EXP_AOL_ENABLE: set in alternative_experience to enable Always-On-Lateral (AOL).
// When set, steering is permitted even without ACC engagement.
#define ALT_EXP_AOL_ENABLE 1024

// Reason why AOL lateral was last deactivated (for debugging / telemetry)
typedef enum {
  AOL_DISENGAGE_NONE = 0,
  AOL_DISENGAGE_ACC_MAIN_OFF,      // ACC main switch turned off
  AOL_DISENGAGE_STEERING_FAULT,    // EPS / car reported a steering disengage
  AOL_DISENGAGE_LAG,               // Safety-critical message lag detected
  AOL_DISENGAGE_DISABLED,          // AOL feature disabled via alternative_experience
  AOL_DISENGAGE_HEARTBEAT_MISMATCH,  // Python stopped confirming AOL is active via USB heartbeat
} AolDisengageReason;

// Track a boolean signal with per-cycle edge detection
typedef struct {
  bool current;
  bool previous;
  bool rising_edge;   // true when current=true and previous=false
  bool falling_edge;  // true when current=false and previous=true
} AolBoolState;

// Track a button (pressed / not-pressed) with per-cycle edge detection
typedef struct {
  bool pressed;       // current state set by the rx hook
  bool last_pressed;  // state from the previous cycle
  bool rising_edge;   // button newly pressed this cycle
  bool falling_edge;  // button newly released this cycle
} AolButtonState;

// Full AOL state: all tracked signals and feature configuration
typedef struct {
  AolButtonState lfa_button;          // Physical LFA/LKAS button on the steering wheel
  AolButtonState main_button;         // Cruise main button (used as AOL toggle on cars with no LFA button)
  AolBoolState   acc_main;            // ACC main armed state
  AolBoolState   op_controls_allowed; // openpilot's own controls_allowed (full engagement) state
  AolBoolState   steering_disengage;  // Car-reported steering disengage / fault flag
  bool           system_active;       // True when ALT_EXP_AOL_ENABLE is set
  AolDisengageReason last_disengage;  // Most recent reason for deactivation
} AolState;

// Globals defined in aol.h — declared here so other headers can reference them
extern bool lateral_allowed;    // Main per-cycle steering permission flag read by lateral.h
extern bool lfa_button_press;   // Set by hyundai / hyundai_canfd rx_hook on LFA button press
extern bool main_button_press;  // Set by hyundai / hyundai_canfd rx_hook on cruise main button press
extern AolState aol_state;
extern bool heartbeat_engaged_aol;             // Set by USB heartbeat command (0xf3, param2) when Python confirms AOL is active
extern uint32_t heartbeat_engaged_aol_mismatches; // count of cycles lateral_allowed and heartbeat_engaged_aol have disagreed
