#include <Arduino.h>

// ====== Configuración ======
#define Tds1Pin A1
#define Tds2Pin A2

// En Mega2560 el ADC por defecto usa AVcc (~5V) y resolución 10-bit (0..1023)
#define VREF     5.0f
#define SCOUNT   30

// Calibración independiente por sensor (recomendado)
#define K1_VALUE 1.8f
#define K2_VALUE 1.8f

// Si luego mides temperatura real, reemplaza este valor
static float temperatureC = 25.0f;

// ====== Buffers por sensor ======
static int buf1[SCOUNT], buf1Tmp[SCOUNT];
static int buf2[SCOUNT], buf2Tmp[SCOUNT];
static uint8_t idx1 = 0, idx2 = 0;

int getMedianNum(int bArray[], int len);

// Calcula TDS (ppm) desde una lectura ADC mediana
static float adcToTdsPpm(int medianAdc, float kValue, float &avgVoltageOut) {
  // ADC 10-bit: 0..1023
  float avgV = (float)medianAdc * VREF / 1024.0f;
  avgVoltageOut = avgV;

  // Compensación de temperatura (modelo típico)
  float compCoeff = 1.0f + 0.02f * (temperatureC - 25.0f);
  float compV = avgV / compCoeff;

  // Conversión a TDS (ppm)
  float tds = (133.42f * compV * compV * compV
             - 255.86f * compV * compV
             + 857.39f * compV) * 0.5f * kValue;

  return tds;
}

void setup() {
  Serial.begin(115200);
  pinMode(Tds1Pin, INPUT);
  pinMode(Tds2Pin, INPUT);

  // Inicializa buffers con la primera lectura para evitar arranque “basura”
  int r1 = analogRead(Tds1Pin);
  int r2 = analogRead(Tds2Pin);
  for (int i = 0; i < SCOUNT; i++) {
    buf1[i] = r1;
    buf2[i] = r2;
  }
}

void loop() {
  static unsigned long sampleT = millis();
  static unsigned long printT  = millis();

  // ====== Muestreo cada 40 ms ======
  if (millis() - sampleT > 40UL) {
    sampleT = millis();

    buf1[idx1] = analogRead(Tds1Pin);
    idx1 = (idx1 + 1) % SCOUNT;

    buf2[idx2] = analogRead(Tds2Pin);
    idx2 = (idx2 + 1) % SCOUNT;
  }

  // ====== Cálculo y salida cada 800 ms ======
  if (millis() - printT > 800UL) {
    printT = millis();

    // Copiar buffers para filtro mediano (evita tocar el buffer mientras se llena)
    for (int i = 0; i < SCOUNT; i++) {
      buf1Tmp[i] = buf1[i];
      buf2Tmp[i] = buf2[i];
    }

    int med1 = getMedianNum(buf1Tmp, SCOUNT);
    int med2 = getMedianNum(buf2Tmp, SCOUNT);

    float v1 = 0.0f, v2 = 0.0f;
    float tds1 = adcToTdsPpm(med1, K1_VALUE, v1);
    float tds2 = adcToTdsPpm(med2, K2_VALUE, v2);

    Serial.print("TDS1: ");
    Serial.print(tds1, 0);
    Serial.print(" ppm\tV1=");
    Serial.print(v1, 2);

    Serial.print(" V\t| TDS2: ");
    Serial.print(tds2, 0);
    Serial.print(" ppm\tV2=");
    Serial.print(v2, 2);
    Serial.println(" V");
  }
}

// ====== Filtro mediano ======
int getMedianNum(int bArray[], int len) {
  // Copia local para ordenar
  int bTab[len];
  for (int i = 0; i < len; i++) bTab[i] = bArray[i];

  // Burbuja simple
  for (int j = 0; j < len - 1; j++) {
    for (int i = 0; i < len - 1 - j; i++) {
      if (bTab[i] > bTab[i + 1]) {
        int t = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = t;
      }
    }
  }

  // Mediana
  if (len & 1) return bTab[(len - 1) / 2];
  return (bTab[len / 2] + bTab[len / 2 - 1]) / 2;
}
