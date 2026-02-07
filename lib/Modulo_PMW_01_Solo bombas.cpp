#include <Arduino.h>

// Arduino Mega2560
// 3 PWM independientes (frecuencia + duty) usando timers 16 bits:
//  - Bomba A -> Timer4  OC4B -> Pin 7
//  - Bomba B -> Timer1  OC1A -> Pin 11
//  - Bomba C -> Timer5  OC5C -> Pin 44
//
// Serial (terminar cada comando con '\n'):
//   FA<Hz>  -> frecuencia A   (FA20000\n)
//   FB<Hz>  -> frecuencia B
//   FC<Hz>  -> frecuencia C
//   DA<0..100> duty A
//   DB<0..100> duty B
//   DC<0..100> duty C
//
// Nota: NO tocamos Timer0 (millis/delay).

static const uint8_t PIN_A = 7;   // OC4B (Timer4)
static const uint8_t PIN_B = 11;  // OC1A (Timer1)
static const uint8_t PIN_C = 44;  // OC5C (Timer5)

static uint32_t fA = 20000, fB = 20000, fC = 20000; // Hz
static float    dA = 0,     dB = 0,     dC = 0;     // %

static bool choosePrescaler_16(uint32_t f, uint16_t &topOut, uint16_t &csBitsOut, uint16_t &prescValOut) {
  const uint16_t prescList[] = {1, 8, 64, 256, 1024};
  // Bits 0..2 son equivalentes en TCCR1B/TCCR4B/TCCR5B para selección de prescaler.
  const uint16_t bitsList[]  = {
    _BV(CS10),                 // 1
    _BV(CS11),                 // 8
    _BV(CS11) | _BV(CS10),     // 64
    _BV(CS12),                 // 256
    _BV(CS12) | _BV(CS10)      // 1024
  };

  if (f == 0) return false;

  for (uint8_t i = 0; i < 5; i++) {
    uint32_t topCalc = (F_CPU / (uint32_t)(prescList[i]) / f) - 1UL;
    if (topCalc >= 1UL && topCalc <= 65535UL) {
      topOut      = (uint16_t)topCalc;
      csBitsOut   = bitsList[i];
      prescValOut = prescList[i];
      return true;
    }
  }
  return false;
}

static uint16_t dutyToOCR(float dutyPct, uint16_t top) {
  dutyPct = constrain(dutyPct, 0.0f, 100.0f);
  uint32_t ocr = (uint32_t)((dutyPct / 100.0f) * (float)(top + 1U) + 0.5f);
  if (ocr > top) ocr = top;
  return (uint16_t)ocr;
}

// ----- STOP PWM helpers (F=0) -----
static void stopA() {
  TCCR4A = 0; TCCR4B = 0;
  pinMode(PIN_A, OUTPUT);
  digitalWrite(PIN_A, LOW);
}
static void stopB() {
  TCCR1A = 0; TCCR1B = 0;
  pinMode(PIN_B, OUTPUT);
  digitalWrite(PIN_B, LOW);
}
static void stopC() {
  TCCR5A = 0; TCCR5B = 0;
  pinMode(PIN_C, OUTPUT);
  digitalWrite(PIN_C, LOW);
}

// ----- Apply each channel -----
static void applyA(uint32_t freqHz, float dutyPct) {
  if (freqHz == 0 || dutyPct <= 0.0f) { stopA(); return; }

  uint16_t top, csBits, prescVal;
  if (!choosePrescaler_16(freqHz, top, csBits, prescVal)) {
    Serial.println("Timer4 (A): Frecuencia fuera de rango.");
    return;
  }

  pinMode(PIN_A, OUTPUT);

  TCCR4A = 0; TCCR4B = 0; TCNT4 = 0;
  ICR4   = top;
  OCR4B  = dutyToOCR(dutyPct, top);

  // Modo 14: Fast PWM, TOP=ICR4. OC4B no inversor.
  TCCR4A = _BV(COM4B1) | _BV(WGM41);
  TCCR4B = _BV(WGM43) | _BV(WGM42) | csBits;
}

static void applyB(uint32_t freqHz, float dutyPct) {
  if (freqHz == 0 || dutyPct <= 0.0f) { stopB(); return; }

  uint16_t top, csBits, prescVal;
  if (!choosePrescaler_16(freqHz, top, csBits, prescVal)) {
    Serial.println("Timer1 (B): Frecuencia fuera de rango.");
    return;
  }

  pinMode(PIN_B, OUTPUT);

  TCCR1A = 0; TCCR1B = 0; TCNT1 = 0;
  ICR1   = top;
  OCR1A  = dutyToOCR(dutyPct, top);

  // Modo 14: Fast PWM, TOP=ICR1. OC1A no inversor.
  TCCR1A = _BV(COM1A1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | csBits;
}

static void applyC(uint32_t freqHz, float dutyPct) {
  if (freqHz == 0 || dutyPct <= 0.0f) { stopC(); return; }

  uint16_t top, csBits, prescVal;
  if (!choosePrescaler_16(freqHz, top, csBits, prescVal)) {
    Serial.println("Timer5 (C): Frecuencia fuera de rango.");
    return;
  }

  pinMode(PIN_C, OUTPUT);

  TCCR5A = 0; TCCR5B = 0; TCNT5 = 0;
  ICR5   = top;
  OCR5C  = dutyToOCR(dutyPct, top);

  // Modo 14: Fast PWM, TOP=ICR5. OC5C no inversor.
  TCCR5A = _BV(COM5C1) | _BV(WGM51);
  TCCR5B = _BV(WGM53) | _BV(WGM52) | csBits;
}

static void applyAll() {
  applyA(fA, dA);
  applyB(fB, dB);
  applyC(fC, dC);
}

// ----- Serial line reader -----
static bool readLine(char *buf, size_t n) {
  if (!Serial.available()) return false;
  size_t len = Serial.readBytesUntil('\n', buf, n - 1);
  if (len == 0) return false;
  buf[len] = '\0';

  // trim derecha (CR/espacios)
  while (len > 0 && (buf[len - 1] == '\r' || buf[len - 1] == ' ' || buf[len - 1] == '\t')) {
    buf[len - 1] = '\0';
    len--;
  }
  return true;
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10); // clave para evitar "lag" si falta '\n' o hay pausas

  // Arranque seguro: 20 kHz y duty 0%
  fA = fB = fC = 20000;
  dA = dB = dC = 0;
  applyAll();

  Serial.println("MEGA2560 PWM listo. Envia FAxxxx, FBxxxx, FCxxxx, DAxx, DBxx, DCxx (termina con \\n).");
}

void loop() {
  static char line[40];
  if (!readLine(line, sizeof(line))) return;

  if (strlen(line) < 3) return;

  char c0 = toupper(line[0]);
  char c1 = toupper(line[1]);

  // valor desde el caracter 2 en adelante (acepta "FA20000" o "FA 20000")
  float val = atof(&line[2]);

  if (c0 == 'F' && c1 == 'A') { fA = (val <= 0.0f) ? 0 : (uint32_t)val; applyA(fA, dA); }
  else if (c0 == 'F' && c1 == 'B') { fB = (val <= 0.0f) ? 0 : (uint32_t)val; applyB(fB, dB); }
  else if (c0 == 'F' && c1 == 'C') { fC = (val <= 0.0f) ? 0 : (uint32_t)val; applyC(fC, dC); }
  else if (c0 == 'D' && c1 == 'A') { dA = val; applyA(fA, dA); }
  else if (c0 == 'D' && c1 == 'B') { dB = val; applyB(fB, dB); }
  else if (c0 == 'D' && c1 == 'C') { dC = val; applyC(fC, dC); }
  else {
    Serial.println("Comando no reconocido. Usa FA/FB/FC o DA/DB/DC.");
  }
}
