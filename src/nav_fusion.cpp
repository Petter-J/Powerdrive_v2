#include "nav_fusion.h"
#include <cstring>
#include "config.h"

// ============================================================
// WORLD HEADING GPS CALIBRATION
// ------------------------------------------------------------
// GPS COG får bara användas som world-referens när båten har
// gått tillräckligt fort och haft stabil kurs en stund.
//
// Detta använder en stabilitetspoäng istället för hård reset:
// - bra GPS COG ger +100 ms
// - dålig GPS COG ger -200 ms
// - målet är 5000 ms
//
// Enstaka GPS-jitter förstör alltså inte hela kalibreringen,
// men många dåliga COG-värden gör att world inte blir godkänd.
// ============================================================
static bool gpsHeadingCalibrationOk(const GpsFix &gps)
{
    static bool active = false;
    static float referenceCourseDeg = 0.0f;
    static int32_t stableMs = 0;
    

    static constexpr float MIN_CAL_SPEED_MPS = 2.0f;
    static constexpr float MAX_COURSE_DEVIATION_DEG = 3.0f;
    static constexpr int32_t REQUIRED_STABLE_MS = 5000;
    static constexpr int32_t GOOD_STEP_MS = 100;
    static constexpr int32_t BAD_PENALTY_MS = 200;

    if (!gps.speedValid ||
        !gps.courseValid ||
        gps.speedMps < MIN_CAL_SPEED_MPS)
    {
        active = false;
        stableMs = 0;
        return false;
    }

    if (!active)
    {
        active = true;
        referenceCourseDeg = gps.courseDeg;
        stableMs = 0;
        return false;
    }

    // Räkna bara när GPS faktiskt har levererat en ny position.
    // Annars skulle samma COG kunna räknas flera loopvarv.
    if (!gps.locationUpdated)
    {
        return stableMs >= REQUIRED_STABLE_MS;
    }

    const float courseDeviationDeg =
        fabsf(shortestAngleErrorDeg(
            gps.courseDeg,
            referenceCourseDeg));

    if (courseDeviationDeg <= MAX_COURSE_DEVIATION_DEG)
    {
        stableMs += GOOD_STEP_MS;

        if (stableMs > REQUIRED_STABLE_MS)
            stableMs = REQUIRED_STABLE_MS;
    }
    else
    {
        stableMs -= BAD_PENALTY_MS;

        if (stableMs < 0)
            stableMs = 0;

        // Starta om referenskursen från den nya GPS-kursen.
        // Stabilitetspoängen nollas inte, men får straff.
        referenceCourseDeg = gps.courseDeg;
    }

    return stableMs >= REQUIRED_STABLE_MS;
}

void NavFusion::update(
    const GpsFix &gps,
    const ImuHeading &imu,
    SensorData &s)
{
    s.headingValid = false;
    s.gpsValid = false;
    s.speedValid = false;

    strcpy(s.headingSource, "NONE");

    s.gpsValid = gps.locationValid;
    s.locationUpdated = gps.locationUpdated;
    s.speedValid = gps.speedValid;
    s.courseValid = gps.courseValid;

    if (gps.locationValid)
    {
        s.latitudeDeg = gps.latDeg;
        s.longitudeDeg = gps.lonDeg;
    }

    s.satellites = gps.satellites;
    s.satellitesInView = gps.satellitesInView;

    if (gps.speedValid)
    {
        s.gpsSpeedMps = gps.speedMps;
        s.speedMps = gps.speedMps;
    }
    else
    {
        s.gpsSpeedMps = 0.0f;
        s.speedMps = 0.0f;
    }

    if (gps.courseValid)
        s.courseOverGroundDeg = gps.courseDeg;
    else
        s.courseOverGroundDeg = 0.0f;

    const float maxSpeedMps = AutoConfig::MAX_SPEED_MPS;
    const float minSpeedThreshold = 0.3f;

    float speed = s.speedMps;
    if (speed < minSpeedThreshold)
        speed = 0.0f;

    float pct = (speed / maxSpeedMps) * 100.0f;
    pct = clampf(pct, 0.0f, 100.0f);

    s.speedPct = pct;

    const uint32_t nowMs = millis();

    static uint32_t lastMotorImuValidMs = 0;

    if (imu.valid)
    {
        s.motorHeadingRawDeg = imu.headingDeg;

        if (s.motorHeadingWorldValid)
        {
            s.motorHeadingWorldDeg =
                wrap360(s.motorHeadingRawDeg + s.motorHeadingWorldOffsetDeg);

            s.motorHeadingDeg = s.motorHeadingWorldDeg;
        }
        else
        {
            s.motorHeadingDeg = s.motorHeadingRawDeg;
        }

        s.motorPitchDeg = imu.pitchDeg;
        s.motorRollDeg = imu.rollDeg;
        s.motorImuValid = true;
        lastMotorImuValidMs = nowMs;
    }
    else
    {
        s.motorImuValid = false;
    }

    const bool motorImuUsable =
        s.motorImuValid ||
        ((uint32_t)(nowMs - lastMotorImuValidMs) <
         CompassConfig::MOTOR_HEADING_HOLD_MS);

    static uint32_t lastBoatImuValidMs = 0;

    if (s.boatImuValid)
    {
        s.boatHeadingRawDeg = s.boatHeadingDeg;

        if (s.boatHeadingWorldValid)
        {
            s.boatHeadingWorldDeg =
                wrap360(s.boatHeadingRawDeg + s.boatHeadingWorldOffsetDeg);

            s.boatHeadingDeg = s.boatHeadingWorldDeg;
        }

        lastBoatImuValidMs = nowMs;
    }

    const bool boatImuUsable =
        s.boatImuValid ||
        ((uint32_t)(nowMs - lastBoatImuValidMs) <
         BoatCompassConfig::BOAT_HEADING_HOLD_MS);

    static bool worldHeadingCalibrated = false;

    if (!worldHeadingCalibrated &&
        gpsHeadingCalibrationOk(gps) &&
        s.motorImuValid &&
        s.boatImuValid)
    {
        s.motorHeadingWorldOffsetDeg =
            shortestAngleErrorDeg(
                gps.courseDeg,
                s.motorHeadingRawDeg);

        s.boatHeadingWorldOffsetDeg =
            shortestAngleErrorDeg(
                gps.courseDeg,
                s.boatHeadingRawDeg);

        s.motorHeadingWorldDeg =
            wrap360(s.motorHeadingRawDeg + s.motorHeadingWorldOffsetDeg);

        s.boatHeadingWorldDeg =
            wrap360(s.boatHeadingRawDeg + s.boatHeadingWorldOffsetDeg);

        s.motorHeadingWorldValid = true;
        s.boatHeadingWorldValid = true;

        worldHeadingCalibrated = true;
    }

    if (s.motorHeadingWorldValid && s.boatHeadingWorldValid)
    {
        s.motorAngleDeg =
            shortestAngleErrorDeg(
                s.boatHeadingWorldDeg,
                s.motorHeadingWorldDeg);
    }
    else
    {
        s.motorAngleDeg = 0.0f;
    }

    if (s.boatHeadingWorldValid)
    {
        s.headingDeg = s.boatHeadingWorldDeg;
        s.headingValid = true;
        strcpy(s.headingSource, "BH");
    }
    else if (s.motorHeadingWorldValid)
    {
        s.headingDeg = s.motorHeadingWorldDeg;
        s.headingValid = true;
        strcpy(s.headingSource, "MH");
    }
    else
    {
        s.headingDeg = 0.0f;
        s.headingValid = false;
        strcpy(s.headingSource, "NONE");
    }
}