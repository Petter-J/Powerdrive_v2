#include "gps_sensor.h"



static void sendUbx(HardwareSerial &serial, const uint8_t *payload, uint16_t len)
{
    uint8_t ckA = 0;
    uint8_t ckB = 0;

    serial.write(0xB5);
    serial.write(0x62);

    for (uint16_t i = 0; i < len; i++)
    {
        ckA += payload[i];
        ckB += ckA;
        serial.write(payload[i]);
    }

    serial.write(ckA);
    serial.write(ckB);
}

static void disableNmeaMessage(HardwareSerial &serial, uint8_t msgId)
{
    const uint8_t msg[] = {
        0x06, 0x01,  // UBX-CFG-MSG
        0x08, 0x00,  // length 8
        0xF0, msgId, // NMEA class, message id
        0x00,        // I2C
        0x00,        // UART1
        0x00,        // UART2
        0x00,        // USB
        0x00,        // SPI
        0x00         // reserved
    };

    sendUbx(serial, msg, sizeof(msg));
}

bool GpsSensor::begin()
{
    _serial.begin(
        GpsConfig::BAUD,
        SERIAL_8N1,
        GpsConfig::RX_PIN,
        GpsConfig::TX_PIN);

    _serial.setTimeout(0);

    delay(2000);

    // Disable NMEA GSA, GSV, GLL and VTG
    disableNmeaMessage(_serial, 0x02); // GSA
    delay(50);

    disableNmeaMessage(_serial, 0x03); // GSV
    delay(50);

    disableNmeaMessage(_serial, 0x01); // GLL
    delay(50);

    disableNmeaMessage(_serial, 0x05); // VTG
    delay(50);

    return true;
}

void GpsSensor::update(GpsFix &out)
{
    uint16_t bytesRead = 0;
    static constexpr uint16_t MAX_GPS_BYTES_PER_UPDATE = 135;

    while (_serial.available() &&
           bytesRead < MAX_GPS_BYTES_PER_UPDATE)
    {
        char c = _serial.read();
        _gps.encode(c);
        bytesRead++;
    }

    out.locationValid =
        _gps.location.isValid() &&
        _gps.location.age() < 2000;
    out.speedValid =
        _gps.speed.isValid() &&
        _gps.speed.age() < 2000;
    out.courseValid = false;

    if (out.locationValid)
    {
        out.latDeg = _gps.location.lat();
        out.lonDeg = _gps.location.lng();
        out.locationUpdated = _gps.location.isUpdated();
    }
    else
    {
        out.locationUpdated = false;
    }

    if (out.speedValid)
    {
        out.speedMps = _gps.speed.mps();
    }
    else
    {
        out.speedMps = 0.0f;
    }

    if (_gps.course.isValid() &&
        _gps.course.age() < 2000 &&
        out.speedValid &&
        out.speedMps >= AutoConfig::MIN_GPS_COURSE_SPEED_MPS)
    {
        const float course = _gps.course.deg();

        if (course >= 0.0f && course <= 360.0f)
        {
            out.courseDeg = course;
            out.courseValid = true;
        }
        else
        {
            out.courseDeg = 0.0f;
            out.courseValid = false;
        }
    }
    else
    {
        out.courseDeg = 0.0f;
        out.courseValid = false;
    }

   
}