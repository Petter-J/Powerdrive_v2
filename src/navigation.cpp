#include "navigation.h"
#include <Arduino.h>

bool Navigation::begin()
{
    const bool gpsOk = _gps.begin();

    const bool imuOk = _imu.begin(
        CompassConfig::RX_PIN,
        CompassConfig::TX_PIN,
        CompassConfig::BAUD,
        CompassConfig::M_HEADING_OFFSET_DEG);

    return gpsOk || imuOk;
}

void Navigation::update(SensorData &sensors)
{
    static GpsFix gpsFix{};
    static ImuHeading motorImuHeading{};
    static uint32_t lastMotorImuUpdateMs = 0;

    const uint32_t now = millis();

    // GPS TEST AVSTÄNGD
    gpsFix = {};

    if (now - lastMotorImuUpdateMs >= 20)
    {
        lastMotorImuUpdateMs = now;
        _imu.update(motorImuHeading);
    }

    _fusion.update(
        gpsFix,
        motorImuHeading,
        sensors);
}