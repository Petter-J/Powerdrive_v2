#pragma once
#include <Arduino.h>
#include "remote_protocol.h"

bool display_is_available();
void display_begin();

void display_update(
    const StatusPacket &status,
    bool hasStatus,
    uint32_t buttonMask,
    bool linkAlive,
    bool localBoatHeadingValid = false,
    float localBoatHeadingDeg = 0.0f);


void display_set_local_ota(bool active);