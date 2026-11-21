#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"

const int SENSOR_PIN = A0;
const int LED_PIN = D2;

const int AIR_RAW = 2800;
const int WATER_RAW = 1300;
const int RAW_MIN_VALID = 200;
const int RAW_MAX_VALID = 4000;
const int SENSOR_FAULT = -1;
const int SAMPLES_PER_READING = 10;
const int DRY_THRESHOLD_PCT = 30;

const uint32_t SENSOR_PERIOD_MS = 2000;
const uint32_t SENSOR_STALE_MS = 3 * SENSOR_PERIOD_MS;
const uint32_t LED_SLOW_TOGGLE_MS = 500;
const uint32_t LED_VFAST_TOGGLE_MS = 50;
const uint32_t PUMP_BURST_MS = 3000;
const uint32_t PUMP_SOAK_MS = 10000;

enum PumpState {PUMP_IDLE, PUMP_PUMPING, PUMP_SOAKING};

QueueHandle_t moistureToLedQ;
QueueHandle_t moistureToPumpQ;
TimerHandle_t sampleTimer;
TaskHandle_t sensorTaskHandle;
void sampleTimerCb(TimerHandle_t) {
  xTaskNotifyGive(sensorTaskHandle);
}
int readRawAveraged() {
  long sum = 0;
  for (int i = 0; i < SAMPLES_PER_READING; i++) {
    sum += analogRead(SENSOR_PIN);
    vTaskDelay(pdMS_TO_TICKS(5));
  }
  return sum / SAMPLES_PER_READING;
}
int rawToPercent(int raw) {
  if (raw < RAW_MIN_VALID || raw > RAW_MAX_VALID) return SENSOR_FAULT;
  int pct = (AIR_RAW - raw) * 100 / (AIR_RAW - WATER_RAW);
  return constrain(pct, 0, 100);
}
void sensorTask(void *) {
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    int raw = readRawAveraged();
    int pct = rawToPercent(raw);
    xQueueOverwrite(moistureToLedQ, &pct);
    xQueueOverwrite(moistureToPumpQ, &pct);
  }
}
void ledTask(void *) {
  int moisture = 100;
  bool ledOn = false;
  uint32_t lastToggle = 0;
  uint32_t lastReadingMs = millis();
  for (;;) {
    int m;
    if (xQueueReceive(moistureToLedQ, &m, pdMS_TO_TICKS(100)) == pdTRUE) {
      moisture = m;
      lastReadingMs = millis();
    }
    bool stale = (millis() - lastReadingMs) > SENSOR_STALE_MS;
    uint32_t toggleMs = 0;
    if (stale || moisture == SENSOR_FAULT) toggleMs = LED_VFAST_TOGGLE_MS;
    else if (moisture < DRY_THRESHOLD_PCT) toggleMs = LED_SLOW_TOGGLE_MS;

    if (toggleMs == 0) {
      ledOn = false;
    } else if (millis() - lastToggle >= toggleMs) {
      ledOn = !ledOn;
      lastToggle = millis();
    }
    digitalWrite(LED_PIN, ledOn ? HIGH : LOW);
  }
}
void pumpTask(void *) {
  PumpState state = PUMP_IDLE;
  for (;;) {
    switch (state) {
      case PUMP_IDLE: {
        int m;
        xQueueReceive(moistureToPumpQ, &m, portMAX_DELAY);
        if (m != SENSOR_FAULT && m < DRY_THRESHOLD_PCT) {
          state = PUMP_PUMPING;
          Serial.println("IDLE -> PUMPING");
        }
        break;
      }
      case PUMP_PUMPING:
        vTaskDelay(pdMS_TO_TICKS(PUMP_BURST_MS));
        state = PUMP_SOAKING;
        Serial.println("PUMPING -> SOAKING");
        break;
      case PUMP_SOAKING:
        vTaskDelay(pdMS_TO_TICKS(PUMP_SOAK_MS));
        xQueueReset(moistureToPumpQ);
        state = PUMP_IDLE;
        Serial.println("SOAKING -> IDLE");
        break;
    }
  }
}
void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  Serial.begin(115200);
  delay(1000);
  moistureToLedQ  = xQueueCreate(1, sizeof(int));
  moistureToPumpQ = xQueueCreate(1, sizeof(int));
  xTaskCreate(pumpTask,   "pump",   4096, NULL, 3, NULL);
  xTaskCreate(sensorTask, "sensor", 4096, NULL, 2, &sensorTaskHandle);
  xTaskCreate(ledTask,    "led",    4096, NULL, 1, NULL);
  sampleTimer = xTimerCreate("sample", pdMS_TO_TICKS(SENSOR_PERIOD_MS), pdTRUE, NULL, sampleTimerCb);
  xTimerStart(sampleTimer, 0);
  xTaskNotifyGive(sensorTaskHandle);
}
void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
