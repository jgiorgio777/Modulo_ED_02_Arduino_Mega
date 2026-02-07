#include <Arduino.h>

const uint8_t LED_PIN = 13;

void setup() {
    pinMode(LED_PIN, OUTPUT);
}

void loop() {
    digitalWrite(LED_PIN, HIGH);
    delay(2000);          // 2 segundos ON

    digitalWrite(LED_PIN, LOW);
    delay(2000);          // 2 segundos OFF
}
