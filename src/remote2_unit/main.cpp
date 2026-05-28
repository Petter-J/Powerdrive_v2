#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <cstring>
#include "ota_update.h"
#include "remote_protocol.h"
#include "remote2/display_lcd.h"

// ============================================================
// BUTTON IDS
// ============================================================
enum class ButtonId : uint8_t
{
    STOP = 0,
    MODE_MANUAL,
    MODE_AUTO,
    MODE_ANCHOR,
    THRUST_UP,
    THRUST_DOWN,
    STEER_LEFT,
    STEER_RIGHT
};

constexpr uint32_t buttonBit(ButtonId id)
{
    return (1UL << static_cast<uint8_t>(id));
}

// ============================================================
// PIN CONFIG - REMOTE2
// ============================================================
namespace ButtonPins
{
    static constexpr int STOP = 2;
    static constexpr int MODE_MANUAL = 3;
    static constexpr int MODE_AUTO = 16;
    static constexpr int MODE_ANCHOR = 17;

    static constexpr int THRUST_UP = 18;
    static constexpr int THRUST_DOWN = 10;

    static constexpr int STEER_LEFT = 11;
    static constexpr int STEER_RIGHT = 44;
}

// ============================================================
// RECEIVER MAC
// ============================================================
static uint8_t RECEIVER_MAC[6] = {0xB4, 0x3A, 0x45, 0xB9, 0xE1, 0x6C};

// ============================================================
// STATUS
// ============================================================
static StatusPacket gStatus;
static bool gHasStatus = false;
static uint32_t gLastStatusMs = 0;

// ============================================================
// BUTTON READ
// ============================================================
static uint32_t readButtons()
{
    uint32_t mask = 0;

    if (digitalRead(ButtonPins::STOP) == LOW)
        mask |= buttonBit(ButtonId::STOP);

    if (digitalRead(ButtonPins::MODE_MANUAL) == LOW)
        mask |= buttonBit(ButtonId::MODE_MANUAL);

    if (digitalRead(ButtonPins::MODE_AUTO) == LOW)
        mask |= buttonBit(ButtonId::MODE_AUTO);

    if (digitalRead(ButtonPins::MODE_ANCHOR) == LOW)
        mask |= buttonBit(ButtonId::MODE_ANCHOR);

    if (digitalRead(ButtonPins::THRUST_UP) == LOW)
        mask |= buttonBit(ButtonId::THRUST_UP);

    if (digitalRead(ButtonPins::THRUST_DOWN) == LOW)
        mask |= buttonBit(ButtonId::THRUST_DOWN);

    if (digitalRead(ButtonPins::STEER_LEFT) == LOW)
        mask |= buttonBit(ButtonId::STEER_LEFT);

    if (digitalRead(ButtonPins::STEER_RIGHT) == LOW)
        mask |= buttonBit(ButtonId::STEER_RIGHT);

    return mask;
}

// ============================================================
// CALLBACKS
// ============================================================
void onSent(const uint8_t *, esp_now_send_status_t) {}

void onRecv(const uint8_t *, const uint8_t *data, int len)
{
    if (!data || len != (int)sizeof(StatusPacket))
        return;

    memcpy(&gStatus, data, sizeof(StatusPacket));
    gHasStatus = true;
    gLastStatusMs = millis();
}

// ============================================================
// SETUP
// ============================================================
void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);

    Serial.begin(115200);
    delay(1500);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    WiFi.setSleep(false);

    Serial.println();
    Serial.println("REMOTE2 START");
    Serial.print("REMOTE2 MAC: ");
    Serial.println(WiFi.macAddress());

    display_lcd_begin();
    delay(100);

    pinMode(ButtonPins::STOP, INPUT_PULLUP);

    const uint32_t bootStartMs = millis();
    bool forceOta = false;

    while (millis() - bootStartMs < 5000)
    {
        if (digitalRead(ButtonPins::STOP) == LOW)
        {
            forceOta = true;
            break;
        }

        delay(10);
    }

    if (forceOta)
    {
        ota_begin();
        gStatus.flags |= STATUS_FLAG_OTA_ACTIVE;

        while (true)
        {
            ota_handle();

            display_lcd_update(
                gStatus,
                true,
                0,
                true);

            delay(50);
        }
    }

    pinMode(ButtonPins::MODE_MANUAL, INPUT_PULLUP);
    pinMode(ButtonPins::MODE_AUTO, INPUT_PULLUP);
    pinMode(ButtonPins::MODE_ANCHOR, INPUT_PULLUP);

    pinMode(ButtonPins::THRUST_UP, INPUT_PULLUP);
    pinMode(ButtonPins::THRUST_DOWN, INPUT_PULLUP);

    pinMode(ButtonPins::STEER_LEFT, INPUT_PULLUP);
    pinMode(ButtonPins::STEER_RIGHT, INPUT_PULLUP);

    if (esp_now_init() != ESP_OK)
    {
        Serial.println("ESP-NOW init failed");
        return;
    }

    esp_now_register_send_cb(onSent);
    esp_now_register_recv_cb(onRecv);

    esp_now_peer_info_t peer{};
    memcpy(peer.peer_addr, RECEIVER_MAC, 6);
    peer.channel = 0;
    peer.encrypt = false;

    esp_now_add_peer(&peer);
}

// ============================================================
// LOOP
// ============================================================
void loop()
{
    static uint32_t lastSendMs = 0;
    static uint32_t lastPrintMask = 0;
    static uint32_t lastSentMask = 0;

    const uint32_t now = millis();
    const uint32_t buttonMask = readButtons();

    static uint32_t earlyOtaStopHoldStartMs = 0;
    static bool earlyOtaTriggered = false;

    const bool stopHeld =
        (buttonMask & buttonBit(ButtonId::STOP)) != 0;

    if (stopHeld)
    {
        if (earlyOtaStopHoldStartMs == 0)
        {
            earlyOtaStopHoldStartMs = millis();
        }

        if (!earlyOtaTriggered &&
            (millis() - earlyOtaStopHoldStartMs) >= 5000)
        {
            earlyOtaTriggered = true;
            ota_begin();
        }
    }
    else
    {
        earlyOtaStopHoldStartMs = 0;
        earlyOtaTriggered = false;
    }

    ota_handle();

    if (ota_is_active())
    {
        gStatus.flags |= STATUS_FLAG_OTA_ACTIVE;

        display_lcd_update(
            gStatus,
            true,
            buttonMask,
            true);

        return;
    }

    // Skicka knappar
    const bool changed = (buttonMask != lastSentMask);
    const bool heartbeat = (now - lastSendMs >= 50);

    if (changed || heartbeat)
    {
        lastSendMs = now;
        lastSentMask = buttonMask;

        RemotePacket pkt = {};
        pkt.buttonMask = buttonMask;

        esp_now_send(RECEIVER_MAC,
                     reinterpret_cast<const uint8_t *>(&pkt),
                     sizeof(pkt));
    }

    // Link status
    const bool linkAlive = gHasStatus && ((now - gLastStatusMs) < 1000);

    // Uppdatera display max 10 Hz
    static uint32_t lastDisplayMs = 0;

    if (now - lastDisplayMs >= 100)
    {
        lastDisplayMs = now;

        display_lcd_update(
            gStatus,
            gHasStatus,
            buttonMask,
            linkAlive);
    }
}