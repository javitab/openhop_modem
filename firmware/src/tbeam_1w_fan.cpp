#include "tbeam_1w_fan.h"

#include "board_config.h"

#ifdef ARDUINO_ARCH_ESP32
#include <Arduino.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace TBeam1WFan {
namespace {
portMUX_TYPE mutex = portMUX_INITIALIZER_UNLOCKED;
float cachedTemperature = NAN;
uint32_t cachedAt = 0;
bool shutdown = false;

void task(void*) {
    pinMode(BOARD.thermistor.fan_pin, OUTPUT);
    digitalWrite(BOARD.thermistor.fan_pin, HIGH);
    const esp_err_t setup = adc2_config_channel_atten(
        ADC2_CHANNEL_3, ADC_ATTEN_DB_11);
    if (setup != ESP_OK) {
        Serial.printf("ERROR: T-Beam 1W NTC ADC setup failed: %d; fan stays ON\n",
                      setup);
        vTaskDelete(nullptr);
        return;
    }
    esp_adc_cal_characteristics_t calibration = {};
    const auto source = esp_adc_cal_characterize(
        ADC_UNIT_2, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &calibration);
    if (source == ESP_ADC_CAL_VAL_DEFAULT_VREF ||
        source == ESP_ADC_CAL_VAL_NOT_SUPPORTED) {
        Serial.println("ERROR: T-Beam 1W NTC calibration unavailable; fan stays ON");
        vTaskDelete(nullptr);
        return;
    }

    Hysteresis control;
    for (;;) {
        uint32_t sum = 0;
        esp_err_t error = ESP_OK;
        for (unsigned i = 0; i < 8; ++i) {
            int raw = 0;
            error = adc2_get_raw(ADC2_CHANNEL_3, ADC_WIDTH_BIT_12, &raw);
            if (error != ESP_OK || raw <= 0 || raw >= 4095) break;
            const uint32_t mv = esp_adc_cal_raw_to_voltage(raw, &calibration);
            if (!std::isfinite(ntcTemperatureC(mv))) {
                error = ESP_ERR_INVALID_RESPONSE;
                break;
            }
            sum += mv;
        }
        const float temperature = error == ESP_OK
            ? ntcTemperatureC(sum / 8) : NAN;
        const bool enabled = control.update(temperature);
        portENTER_CRITICAL(&mutex);
        if (shutdown) {
            portEXIT_CRITICAL(&mutex);
            return;
        }
        digitalWrite(BOARD.thermistor.fan_pin, enabled ? HIGH : LOW);
        cachedTemperature = temperature;
        cachedAt = millis();
        portEXIT_CRITICAL(&mutex);
        if (!std::isfinite(temperature)) {
            Serial.printf("ERROR: T-Beam 1W NTC reading failed: %d; fan ON\n",
                          error);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
}  // namespace

void begin() {
    if (BOARD.thermistor.ntc_pin < 0 || BOARD.thermistor.fan_pin < 0) return;
    pinMode(BOARD.thermistor.fan_pin, OUTPUT);
    digitalWrite(BOARD.thermistor.fan_pin, HIGH);
    shutdown = false;
    if (xTaskCreate(task, "tbeam_fan", 4096, nullptr, 1, nullptr) != pdPASS) {
        Serial.println("ERROR: T-Beam 1W fan task could not start; fan stays ON");
    }
}

void powerOff() {
    if (BOARD.thermistor.fan_pin < 0) return;
    portENTER_CRITICAL(&mutex);
    shutdown = true;
    cachedTemperature = NAN;
    cachedAt = 0;
    digitalWrite(BOARD.thermistor.fan_pin, LOW);
    portEXIT_CRITICAL(&mutex);
}

float temperatureC() {
    if (BOARD.thermistor.ntc_pin < 0) return NAN;
    portENTER_CRITICAL(&mutex);
    const float value = cachedTemperature;
    const uint32_t at = cachedAt;
    portEXIT_CRITICAL(&mutex);
    return (millis() - at < 3000) ? value : NAN;
}
}  // namespace TBeam1WFan
#else
namespace TBeam1WFan {
float temperatureC() { return NAN; }
void begin() {}
void powerOff() {}
}  // namespace TBeam1WFan
#endif
