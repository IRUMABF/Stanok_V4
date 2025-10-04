#pragma once
#include <Arduino.h>
#include "pinout.h"
#include "config.h"

struct ButtonState {
    bool current = false;
    bool last = false;
    unsigned long lastChange = 0;
    bool pressedEvent = false;
};

struct SensorState {
    bool current = false;
    bool last = false;
    unsigned long lastChange = 0;
    bool rising = false;
};

enum ButtonMode {
    BUTTON_MOMENTARY = 0,
    BUTTON_TOGGLE = 1
};

struct ControlsConfig {
    // Inversions for buttons and sensors (after reading raw pin)
    bool invertStart = false;
    bool invertStop = false;
    bool invertS1 = false; // INPUT_PULLUP: active when pin LOW by default
    bool invertS2 = false;


    // Button behavior modes
    ButtonMode startMode = BUTTON_MOMENTARY;
    ButtonMode stopMode = BUTTON_MOMENTARY;
    ButtonMode modeMode = BUTTON_TOGGLE;
    ButtonMode singleMode = BUTTON_MOMENTARY;
};

class Controls {
public:
    // Ініціалізація всіх пінів: кнопок та датчиків
    void begin() {
        // Кнопки
        pinMode(start_PIN, INPUT_PULLUP);
        pinMode(stop_PIN, INPUT_PULLUP);

        // Датчики (INPUT_PULLUP - активний стан = LOW)
        pinMode(sensor_1, INPUT_PULLUP);
        pinMode(sensor_2, INPUT_PULLUP);
        
    }

    // Ініціалізація з конфігурацією (інверсії та режими кнопок)
    void begin(const ControlsConfig& cfg) {
        begin();
        config = cfg;
    }

    // Оновлення стану всіх кнопок та датчиків
    void update() {
        // Оновлення кнопок
        updateButton(start_PIN, startBtn);
        updateButton(stop_PIN, stopBtn);

        // Оновлення датчиків з антидребезгом
        updateSensor(sensor_1, sensor1, config.invertS1);
        updateSensor(sensor_2, sensor2, config.invertS2);
    }

    // --- Кнопки ---
    bool startPressed()    { return handleButton(startBtn, startToggleState, config.startMode, config.invertStart); }
    bool stopPressed()     { return handleButton(stopBtn, stopToggleState, config.stopMode, config.invertStop); }


    // Доступ до станів кнопок у режимі TOGGLE
    bool startToggle() const { return startToggleState; }
    bool stopToggle() const { return stopToggleState; }

    bool isModeToggleConfigured() const { return config.modeMode == BUTTON_TOGGLE; }

    // --- Датчики ---
    // Датчик 1: наявність баночки під соплом розливу фарби
    bool isSensor1Active() { return sensor1.current; }
    // Датчик 2: наявність баночки під пресом закривання кришки
    bool isSensor2Active() { return sensor2.current; }

    // Події фронту (rising edge)
    bool sensor1RisingEdge() { bool e = sensor1.rising; sensor1.rising = false; return e; }
    bool sensor2RisingEdge() { bool e = sensor2.rising; sensor2.rising = false; return e; }
    

private:
    static constexpr unsigned long debounceDelay = 50;

    ControlsConfig config;

    ButtonState startBtn, stopBtn;
    bool startToggleState = false;
    bool stopToggleState = false;

    SensorState sensor1, sensor2;
    

    void updateButton(uint8_t pin, ButtonState& btn) {
        bool reading = (digitalRead(pin) == LOW);
        if (reading != btn.last) {
            btn.lastChange = millis();
        }
        if ((millis() - btn.lastChange) > debounceDelay) {
            if (reading != btn.current) {
                btn.current = reading;
                if (btn.current) btn.pressedEvent = true;
            }
        }
        btn.last = reading;
    }

    void updateSensor(uint8_t pin, SensorState& sensor, bool invert) {
        // Читаємо сире значення (INPUT_PULLUP: активний = LOW)
        bool rawReading = (digitalRead(pin) == LOW);
        bool reading = invert ? !rawReading : rawReading;
        
        // Якщо значення змінилося - починаємо відлік часу антидребезгу
        if (reading != sensor.last) {
            sensor.lastChange = millis();
        }
        
        // Перевіряємо, чи пройшов час антидребезгу
        if ((millis() - sensor.lastChange) > SENSOR_DEBOUNCE_TIME_MS) {
            if (reading != sensor.current) {
                // Edge detection для rising edge (перехід з false на true)
                sensor.rising = (!sensor.current && reading);
                sensor.current = reading;
            }
        }
        sensor.last = reading;
    }

    bool handleButton(ButtonState& btn, bool& toggleRef, ButtonMode mode, bool invert) {
        // Stable logical level derived from debounced state
        bool logicalLevel = invert ? !btn.current : btn.current;
        if (mode == BUTTON_MOMENTARY) {
            bool event = false;
            if (btn.pressedEvent) {
                btn.pressedEvent = false;
                event = true;
            }
            return event;
        } else { // BUTTON_TOGGLE
            // Follow maintained switch level; report change on edges
            bool changed = (logicalLevel != toggleRef);
            toggleRef = logicalLevel;
            return changed;
        }
    }
};