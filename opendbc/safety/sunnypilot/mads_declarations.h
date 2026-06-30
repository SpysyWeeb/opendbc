#pragma once

typedef enum __attribute__((packed)) {
  MADS_BUTTON_UNAVAILABLE = -1,
  MADS_BUTTON_NOT_PRESSED = 0,
  MADS_BUTTON_PRESSED = 1
} ButtonState;

typedef enum __attribute__((packed)) {
  MADS_EDGE_NO_CHANGE = 0,
  MADS_EDGE_RISING = 1,
  MADS_EDGE_FALLING = 2
} EdgeTransition;

typedef enum __attribute__((packed)) {
  MADS_DISENGAGE_REASON_NONE = 0,
  MADS_DISENGAGE_REASON_BRAKE = 1,
  MADS_DISENGAGE_REASON_LAG = 2,
  MADS_DISENGAGE_REASON_BUTTON = 4,
  MADS_DISENGAGE_REASON_ACC_MAIN_OFF = 8,
  MADS_DISENGAGE_REASON_STEERING_DISENGAGE = 64,
} DisengageReason;

typedef struct {
  DisengageReason active_reason;
  DisengageReason pending_reasons;
} DisengageState;

typedef struct {
  ButtonState current;
  ButtonState last;
  EdgeTransition transition;
} ButtonStateTracking;

typedef struct {
  EdgeTransition transition;
  bool current : 1;
  bool previous : 1;
} BinaryStateTracking;

typedef struct {
  bool is_vehicle_moving : 1;
  ButtonStateTracking mads_button;
  BinaryStateTracking acc_main;
  BinaryStateTracking op_controls_allowed;
  BinaryStateTracking braking;
  BinaryStateTracking mads_steering_disengage;
  DisengageState current_disengage;
  bool system_enabled : 1;
  bool disengage_lateral_on_brake : 1;
  bool pause_lateral_on_brake : 1;
  bool controls_requested_lateral : 1;
} MADSState;

// ALT_EXP flags for MADS (must match Python side)
#define ALT_EXP_ENABLE_MADS                       1024  // 0x400
#define ALT_EXP_MADS_DISENGAGE_LATERAL_ON_BRAKE   2048  // 0x800
#define ALT_EXP_MADS_PAUSE_LATERAL_ON_BRAKE       4096  // 0x1000

extern ButtonState mads_button_press;
extern MADSState m_mads_state;
extern bool controls_allowed_lateral;
