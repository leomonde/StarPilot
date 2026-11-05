#pragma once
#include "opendbc/safety/declarations.h"

// EUCD (V60 2010-18). Bus 0 is the car, bus 2 the forward camera; bus 1 is a radar-only aux bus, not monitored.
#define VOLVO_EUCD_MAIN_BUS  0U
#define VOLVO_EUCD_PARTY_BUS 2U

#define VOLVO_EUCD_AccPedal      0x020U  // RX, gas pedal
#define VOLVO_EUCD_FSM0          0x051U  // RX from the camera; ACC state
#define VOLVO_EUCD_EngineData    0x12AU  // RX, ECM ACC state
#define VOLVO_EUCD_VehicleSpeed1 0x148U  // RX, vehicle speed
#define VOLVO_EUCD_Brake_Info    0x20AU  // RX, driver brake pressed
#define VOLVO_EUCD_CCButtons     0x127U  // TX by OP, CC buttons
#define VOLVO_EUCD_PSCM1         0x246U  // TX by OP to camera, PSCM state
#define VOLVO_EUCD_FSM2          0x262U  // TX by OP, LKA command

static void volvo_eucd_rx_hook(const CANPacket_t *msg) {
  if (msg->bus == VOLVO_EUCD_MAIN_BUS) {
    const unsigned int addr = msg->addr;
    if (addr == VOLVO_EUCD_VehicleSpeed1) {
      // Signal: VehicleSpeed
      unsigned int speed_raw = ((unsigned int)msg->data[6] << 8) | (unsigned int)msg->data[7];
      vehicle_moving = speed_raw > 36U;
      UPDATE_VEHICLE_SPEED(speed_raw * 0.01 / 3.6);
    }

    if (addr == VOLVO_EUCD_AccPedal) {
      // Signal: AccPedal
      unsigned int gas_raw = (((unsigned int)msg->data[2] & 0x03U) << 8) | (unsigned int)msg->data[3];
      gas_pressed = gas_raw >= 100U;
    }

    if (addr == VOLVO_EUCD_Brake_Info) {
      // Signal: BrakePedal
      brake_pressed = (((unsigned int)msg->data[2] & 0x0CU) >> 2U) == 2U;
    }

    if (addr == VOLVO_EUCD_PSCM1) {
      // Signal: SteeringAngleServo (16-bit big-endian, bytes 2-3, factor 0.0447, offset -1465 deg).
      // Stored as angle_deg * 100 to match VOLVO_EUCD_STEERING_LIMITS.angle_deg_to_can in the tx hook.
      unsigned int angle_raw = ((unsigned int)msg->data[2] << 8) | (unsigned int)msg->data[3];
      int angle_meas_new = ROUND(((angle_raw * 0.0447f) - 1465.0f) * 100.0f);
      update_sample(&angle_meas, angle_meas_new);
    }

    if (addr == VOLVO_EUCD_EngineData) {
      // Signal: ACC_State_ECM (byte 4 bits 5:4: 1 off, 2 standby, 3 engaged)
      bool cruise_engaged = (((unsigned int)msg->data[4] >> 4) & 0x3U) == 0x3U;
      pcm_cruise_check(cruise_engaged);
    }
  } else if (msg->bus == VOLVO_EUCD_PARTY_BUS) {
    if (msg->addr == VOLVO_EUCD_FSM0) {
      // Signal: ACC_Available, the camera's ACC main on
      acc_main_on = GET_BIT(msg, 17U);
    }
  } else {
    // no other bus is monitored
  }
}

// EUCD sends LKAAngleReq=0 with LKASteerDirection=NONE when not steering, so the commanded angle must
// be exactly zero while disabled, not near the measurement like the shared angle check requires.
static bool volvo_eucd_angle_cmd_checks(int desired_angle, const AngleSteeringLimits limits) {
  const bool allowed = lateral_controls_allowed();
  bool violation = false;

  if (allowed) {
    // Same rate limits as steer_angle_cmd_checks: speed fudged by 1 m/s, with 1 count of margin
    const float fudged_speed = (vehicle_speed.min / VEHICLE_SPEED_FACTOR) - 1.;
    int delta_angle_up = (safety_interpolate(limits.angle_rate_up_lookup, fudged_speed) * limits.angle_deg_to_can) + 1.;
    int delta_angle_down = (safety_interpolate(limits.angle_rate_down_lookup, fudged_speed) * limits.angle_deg_to_can) + 1.;

    int highest_desired_angle = desired_angle_last + ((desired_angle_last > 0) ? delta_angle_up : delta_angle_down);
    int lowest_desired_angle = desired_angle_last - ((desired_angle_last >= 0) ? delta_angle_down : delta_angle_up);

    violation |= safety_max_limit_check(desired_angle, highest_desired_angle, lowest_desired_angle);
    desired_angle_last = desired_angle;
  } else {
    violation |= desired_angle != 0;
  }

  if (violation || !allowed) {
    desired_angle_last = 0;
  }

  return violation;
}

static bool volvo_eucd_tx_hook(const CANPacket_t *msg) {
  static const AngleSteeringLimits VOLVO_EUCD_STEERING_LIMITS = {
    .max_angle = 9000,  // 90 deg
    .angle_deg_to_can = 100,
    .angle_rate_up_lookup = {
      {0., 5., 15.},
      {5., .8, .15}
    },
    .angle_rate_down_lookup = {
      {0., 5., 15.},
      {5., 3.5, .4}
    },
  };

  bool tx = true;
  bool violation = false;
  const unsigned int addr = msg->addr;

  if ((addr == VOLVO_EUCD_CCButtons) && (msg->bus == VOLVO_EUCD_MAIN_BUS)) {
    // Violation if resume button is pressed while controls not allowed, or
    // if cancel button is pressed when cruise isn't engaged.
    violation |= !cruise_engaged_prev && (GET_BIT(msg, 59U) || !(GET_BIT(msg, 43U)));  // Signals: ACCOnOffBtn, ACCOnOffBtnInv (cancel)
    violation |= !controls_allowed && (GET_BIT(msg, 61U) || !(GET_BIT(msg, 45U)));  // Signals: ACCResumeBtn, ACCResumeBtnInv (resume)
  }

  // Safety check for Lane Keep Assist action.
  if (addr == VOLVO_EUCD_FSM2) {
    // Signal: LKAAngleReq (14-bit big-endian over bytes 3-4, factor 0.04, offset -327.68 deg).
    // raw*0.04 - 327.68 deg, carried here as angle_deg * 100 => raw*4 - 32768.
    unsigned int angle_raw = (((unsigned int)msg->data[3] & 0x3FU) << 8) | (unsigned int)msg->data[4];
    int desired_angle = ((int)angle_raw * 4) - 32768;

    // Gate on the angle command, not LKASteerDirection: EUCD needs a ~8-frame NONE pause on a
    // direction change while OP still emits a tracking angle; gating direction would stall lateral.
    if (volvo_eucd_angle_cmd_checks(desired_angle, VOLVO_EUCD_STEERING_LIMITS)) {
      violation = true;
    }
  }

  if (violation) {
    tx = false;
  }

  return tx;
}

static safety_config volvo_eucd_init(uint16_t param) {
  (void)param;
  static const CanMsg VOLVO_EUCD_TX_MSGS[] = {
    {VOLVO_EUCD_CCButtons, VOLVO_EUCD_MAIN_BUS,  8, .check_relay = false},
    {VOLVO_EUCD_PSCM1,     VOLVO_EUCD_PARTY_BUS, 8, .check_relay = true},  // OP replaces stock steering servo state
    {VOLVO_EUCD_FSM2,      VOLVO_EUCD_MAIN_BUS,  8, .check_relay = true},  // OP replaces stock LKA command
  };

  // TODO: add counters
  static RxCheck volvo_eucd_rx_checks[] = {
    {.msg = {{VOLVO_EUCD_AccPedal,      VOLVO_EUCD_MAIN_BUS,  8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 100U}, { 0 }, { 0 }}},
    {.msg = {{VOLVO_EUCD_FSM0,          VOLVO_EUCD_PARTY_BUS, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 100U}, { 0 }, { 0 }}},
    {.msg = {{VOLVO_EUCD_EngineData,    VOLVO_EUCD_MAIN_BUS,  8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 67U}, { 0 }, { 0 }}},
    {.msg = {{VOLVO_EUCD_CCButtons,     VOLVO_EUCD_MAIN_BUS,  8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 66U}, { 0 }, { 0 }}},
    {.msg = {{VOLVO_EUCD_VehicleSpeed1, VOLVO_EUCD_MAIN_BUS,  8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 50U}, { 0 }, { 0 }}},
    {.msg = {{VOLVO_EUCD_Brake_Info,    VOLVO_EUCD_MAIN_BUS,  8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 50U}, { 0 }, { 0 }}},
    {.msg = {{VOLVO_EUCD_PSCM1,         VOLVO_EUCD_MAIN_BUS,  8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 50U}, { 0 }, { 0 }}},  // steering angle meas
  };

  return BUILD_SAFETY_CFG(volvo_eucd_rx_checks, VOLVO_EUCD_TX_MSGS);
}
