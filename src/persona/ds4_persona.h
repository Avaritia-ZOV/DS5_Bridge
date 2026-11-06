#ifndef DS5_BRIDGE_DS4_PERSONA_H
#define DS5_BRIDGE_DS4_PERSONA_H

#include <cstdint>

#include "persona/host_persona.h"

constexpr uint8_t kDs4InputReportId = 0x01;
constexpr uint8_t kDs4InputReportSize = 64;
constexpr uint8_t kDs4OutputReportId = 0x05;
constexpr uint8_t kDs4FeatureCalibrationUsb = 0x02;
constexpr uint8_t kDs4FeatureCalibrationBt = 0x05;

bool ds4_persona_encode_input(
    BridgeControllerState const &state,
    HostPersonaInputReport &report
);

bool ds4_persona_decode_output_to_ds5_payload(
    uint8_t const *data,
    uint16_t len,
    uint8_t *payload,
    uint16_t payload_capacity,
    uint16_t &payload_len
);

// dualsense_calibration is the controller's DualSense calibration feature
// report 0x05 without its report ID, when the bridge has read it. Without it
// the calibration reports carry synthetic values.
uint16_t ds4_persona_get_feature_report(
    uint8_t report_id,
    uint8_t *buffer,
    uint16_t reqlen,
    uint8_t const *dualsense_calibration = nullptr,
    uint16_t dualsense_calibration_len = 0
);

void ds4_persona_set_feature_report(
    uint8_t report_id,
    uint8_t const *buffer,
    uint16_t bufsize
);

// Replaces the default synthetic serial and address with ones derived from
// the bridge's unique board ID. Call once at boot, before USB enumerates.
void ds4_persona_set_identity_seed(uint8_t const *seed, uint16_t seed_len);

#endif // DS5_BRIDGE_DS4_PERSONA_H
