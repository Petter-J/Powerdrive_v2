#pragma once
#include <Arduino.h>

struct RemotePacket
{
    uint32_t buttonMask = 0;

    // Fast BNO085 i båten / remote-enheten
    uint16_t boatHeadingDeg10 = 0; // grader * 10
    uint8_t boatFlags = 0;
};

struct StatusPacket
{
    uint8_t mode = 0;
    uint8_t motorTiltUnsafe = 0;
    uint8_t manualThrustPct = 0;     // 0..100
    uint8_t targetSpeedPct = 0;      // 0..100
    uint16_t boatHeadingDeg10 = 0;   // Boat BNO heading      
    uint16_t motorHeadingDeg10 = 0;  // Motor BNO heading
    uint16_t targetHeadingDeg10 = 0; // Target heading
    uint8_t flags = 0;     // bit0 = gpsValid
    int8_t steerState = 0; // -1 = vänster, 0 = ingen, 1 = höger
    uint8_t counter = 0;
    uint16_t gpsSpeedCmps = 0;  // GPS speed * 100
    uint16_t targetSpeedCmps = 0; // target speed * 100
    uint16_t gpsCogDeg10 = 0;   // GPS COG * 10
};

static constexpr uint8_t STATUS_FLAG_GPS_VALID = 1 << 0;
static constexpr uint8_t STATUS_FLAG_OTA_ACTIVE = 1 << 1;
static constexpr uint8_t REMOTE_FLAG_BOAT_IMU_VALID = 1 << 0;

