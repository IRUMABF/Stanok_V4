#include <Arduino.h>
#include "pinout.h"
#include "config.h"
#include "controls.h"
#include "conveyor.h"
#include "pneumatic_valve.h"

// Глобальні об'єкти
Controls controls;
Conveyor conveyor;
PneumaticValve valve1(PNEUMATIC_1_PIN, true);  // інвертований сигнал
PneumaticValve valve3(PNEUMATIC_3_PIN, true);  // поршень фарби
PneumaticValve valve4(PNEUMATIC_4_PIN);  // завертання кришок
PneumaticValve valve5(PNEUMATIC_5_PIN);  // закривання кришок

// Стани станка
enum MachineState {
  MACHINE_STOPPED,     // станок зупинений
  MACHINE_RUNNING,     // станок працює
  MACHINE_PAUSED       // станок на паузі
};

// Паралельні стани: розлив (PAINT) та закривання (CAP)
enum PaintState {
  P_IDLE,                 // очікування
  P_WAIT_SENSOR,          // очікування датчика 1
  P_DOCIAG,               // дотяжка після датчика 1
  P_PISTON,               // робота поршня фарби
  P_DELAY                 // затримка після розливання
};

enum CapState {
  C_IDLE,                 // очікування
  C_WAIT_SENSOR,          // очікування датчика 2
  C_SCREW_ON,             // увімкнення завертання кришок
  C_SCREW_PAUSE,          // пауза перед закриванням
  C_CLOSE,                // закривання кришок
  C_CLOSE_PAUSE           // пауза після закривання
};

// Глобальні змінні стану
MachineState machineState = MACHINE_STOPPED;
PaintState paintState = P_IDLE;
CapState capState = C_IDLE;

// Лічильники для ігнорування баночок
int paintIgnoreCount = 0;
int capIgnoreCount = 0;

// Час паузи для синхронізації таймерів
unsigned long pauseStartTime = 0;
unsigned long pauseDuration = 0;

// Таймери для неблокуючих затримок
unsigned long paintDelayStart = 0;
unsigned long capScrewPauseStart = 0;
unsigned long capClosePauseStart = 0;

// Оголошення функцій
void handleStartStopButtons();
void handlePaintOperations();
void handleCapOperations();
void arbitrateConveyor();
void updateMachineSignals();
void updateLEDs();
void pauseAllTimers();
void resumeAllTimers();
void shiftAllTimers();

void setup() {
  Serial.begin(9600);
  
  // Ініціалізація всіх компонентів
  controls.begin();
  conveyor.begin();
  valve1.begin();
  valve3.begin();
  valve4.begin();
  valve5.begin();
  
  // Налаштування сигнальних пінів
  pinMode(START_STOP_PIN, OUTPUT);
  digitalWrite(START_STOP_PIN, LOW);
  
  // Налаштування світлодіодів
  pinMode(ledMode0Pin, OUTPUT);
  pinMode(ledMode1Pin, OUTPUT);
  digitalWrite(ledMode0Pin, LOW);
  digitalWrite(ledMode1Pin, HIGH); // станок зупинений
  
  Serial.println("Machine initialized");
}

void loop() {
  // Оновлення всіх компонентів
  controls.update();
  conveyor.update();
  valve1.update();
  valve3.update();
  valve4.update();
  valve5.update();
  
  // Обробка кнопок старт/стоп
  handleStartStopButtons();
  // Тримати вихідний сигнал у синхроні з поточним станом
  updateMachineSignals();
  
  // Якщо станок зупинений - нічого не робимо
  if (machineState == MACHINE_STOPPED) {
    return;
  }
  
  // Якщо станок на паузі - зсуваємо таймери
  if (machineState == MACHINE_PAUSED) {
    shiftAllTimers();
    return;
  }
  
  // Паралельна логіка: розлив і закривання незалежно
  handlePaintOperations();
  handleCapOperations();
  arbitrateConveyor();
}

// Обробка кнопок старт/стоп
void handleStartStopButtons() {
  if (controls.startPressed()) {
    if (machineState == MACHINE_STOPPED) {
      // Запуск станка
      machineState = MACHINE_RUNNING;
      paintState = P_IDLE;
      capState = C_IDLE;
      paintIgnoreCount = 0;
      capIgnoreCount = 0;
      conveyor.start();
      // Імпульс на PNEUMATIC_1 після першого запуску та старту конвеєра
      valve1.onFor(PNEUMATIC1_ON_TIME_MS + PNEUMATIC1_HOLD_TIME_MS);
      updateMachineSignals();
      updateLEDs();
      Serial.println("Machine started");
    } else if (machineState == MACHINE_PAUSED) {
      // Відновлення роботи після паузи
      machineState = MACHINE_RUNNING;
      resumeAllTimers();
      updateMachineSignals();
      updateLEDs();
      Serial.println("Machine resumed");
    }
  }
  
  if (controls.stopPressed()) {
    if (machineState == MACHINE_RUNNING) {
      // Пауза станка
      machineState = MACHINE_PAUSED;
      pauseStartTime = millis();
      pauseAllTimers();
      conveyor.stop();
      updateMachineSignals();
      updateLEDs();
      Serial.println("Machine paused");
    } else if (machineState == MACHINE_PAUSED) {
      // Повна зупинка станка
      machineState = MACHINE_STOPPED;
      paintState = P_IDLE;
      capState = C_IDLE;
      conveyor.stop();
      valve1.off();
      valve3.off();
      valve4.off();
      valve5.off();
      updateMachineSignals();
      updateLEDs();
      Serial.println("Machine stopped");
    }
  }
}

// Розлив фарби — незалежна логіка
void handlePaintOperations() {
  switch (paintState) {
    case P_IDLE:
      paintState = P_WAIT_SENSOR;
      break;
    case P_WAIT_SENSOR:
      if (controls.sensor1RisingEdge()) {
        if (paintIgnoreCount == 0) {
          conveyor.stopWithDociag(JAR_CENTERING_MM);
          paintState = P_DOCIAG;
        } else {
          paintIgnoreCount--;
        }
      }
      break;
    case P_DOCIAG:
      if (!conveyor.isRunning()) {
        valve3.onFor(PAINT_PISTON_HOLD_TIME);
        paintState = P_PISTON;
      }
      break;
    case P_PISTON:
      if (!valve3.isTimerActive()) {
        paintDelayStart = millis();
        paintState = P_DELAY;
      }
      break;
    case P_DELAY:
      if (millis() - paintDelayStart >= 50) {
        paintIgnoreCount = JARS_IN_SET - 1;
        paintState = P_WAIT_SENSOR;
              // Імпульс на PNEUMATIC_1 при відновленні руху
        valve1.onFor(PNEUMATIC1_ON_TIME_MS + PNEUMATIC1_HOLD_TIME_MS);
      }
      break;
  }
}

// Закривання кришок — незалежна логіка
void handleCapOperations() {
  switch (capState) {
    case C_IDLE:
      capState = C_WAIT_SENSOR;
      break;
    case C_WAIT_SENSOR:
      if (controls.sensor2RisingEdge()) {
        if (capIgnoreCount == 0) {
          conveyor.stop();
          valve4.on();
          capState = C_SCREW_ON;
        } else {
          capIgnoreCount--;
        }
      }
      break;
    case C_SCREW_ON:
      capScrewPauseStart = millis();
      capState = C_SCREW_PAUSE;
      break;
    case C_SCREW_PAUSE:
      if (millis() - capScrewPauseStart >= STEP_PAUSE_CAP_SCREW_MS) {
        valve5.onFor(CLOSE_CAP_HOLD_TIME);
        capState = C_CLOSE;
      }
      break;
    case C_CLOSE:
      if (!valve5.isTimerActive()) {
        capClosePauseStart = millis();
        capState = C_CLOSE_PAUSE;
      }
      break;
    case C_CLOSE_PAUSE:
      if (millis() - capClosePauseStart >= STEP_PAUSE_CAP_CLOSE_MS) {
        valve4.off();
        capIgnoreCount = JARS_IN_SET - 1;
        capState = C_WAIT_SENSOR;
      }
      break;
  }
}

// Арбітраж керування конвеєром: обидві підсистеми мають рівні права зупинки
void arbitrateConveyor() {
  if (machineState != MACHINE_RUNNING) return;
  if (conveyor.isDociagActive()) return; // дотягування триває — не втручатися

  bool paintRequiresStop = (paintState == P_DOCIAG || paintState == P_PISTON || paintState == P_DELAY);
  bool capRequiresStop = (capState == C_SCREW_ON || capState == C_SCREW_PAUSE || capState == C_CLOSE || capState == C_CLOSE_PAUSE);

  bool shouldRun = !(paintRequiresStop || capRequiresStop);

  if (!shouldRun) {
    if (conveyor.isRunning()) {
      conveyor.stop();
    }
  } else {
    if (!conveyor.isRunning()) {
      conveyor.start();
    }
  }
}

// Оновлення сигналів станка
void updateMachineSignals() {
  // Активний сигнал для іншого контролера має бути HIGH тільки коли станок працює (RUNNING),
  // і LOW коли зупинений (STOPPED) або на паузі (PAUSED).
  static MachineState lastState = MACHINE_STOPPED;
  bool machineActive = (machineState == MACHINE_RUNNING);
  digitalWrite(START_STOP_PIN, machineActive ? HIGH : LOW);
  if (machineState != lastState) {
    Serial.print("updateMachineSignals: state=");
    Serial.print(machineState == MACHINE_STOPPED ? "STOPPED" : machineState == MACHINE_RUNNING ? "RUNNING" : "PAUSED");
    Serial.print(", pin=");
    Serial.println(machineActive ? "HIGH" : "LOW");
    lastState = machineState;
  }
}

// Оновлення світлодіодів
void updateLEDs() {
  if (machineState == MACHINE_RUNNING) {
    digitalWrite(ledMode0Pin, HIGH);
    digitalWrite(ledMode1Pin, LOW);
  } else {
    digitalWrite(ledMode0Pin, LOW);
    digitalWrite(ledMode1Pin, HIGH);
  }
}

// Пауза всіх таймерів
void pauseAllTimers() {
  pauseStartTime = millis();
  // Таймери пневмоклапанів автоматично зсуваються в update()
}

// Відновлення всіх таймерів
void resumeAllTimers() {
  pauseDuration = millis() - pauseStartTime;
  valve1.shiftTimers(pauseDuration);
  valve3.shiftTimers(pauseDuration);
  valve4.shiftTimers(pauseDuration);
  valve5.shiftTimers(pauseDuration);
  
  // Зсуваємо неблокуючі таймери
  paintDelayStart += pauseDuration;
  capScrewPauseStart += pauseDuration;
  capClosePauseStart += pauseDuration;
}

// Зсув таймерів під час паузи
void shiftAllTimers() {
  // Таймери автоматично зсуваються в update() пневмоклапанів
}