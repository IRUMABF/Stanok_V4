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

// Стани операцій
enum OperationState {
  OP_IDLE,                    // очікування
  OP_PAINT_WAIT_SENSOR,       // очікування датчика 1 для розливання
  OP_PAINT_DOCIAG,            // дотяжка після датчика 1
  OP_PAINT_PISTON,            // робота поршня фарби
  OP_PAINT_DELAY,             // затримка після розливання
  OP_CAP_WAIT_SENSOR,         // очікування датчика 2 для закривання
  OP_CAP_SCREW_ON,            // увімкнення завертання кришок
  OP_CAP_SCREW_PAUSE,         // пауза перед закриванням
  OP_CAP_CLOSE,               // закривання кришок
  OP_CAP_CLOSE_PAUSE,         // пауза після закривання
  OP_CAP_SCREW_OFF            // вимкнення завертання кришок
};

// Глобальні змінні стану
MachineState machineState = MACHINE_STOPPED;
OperationState operationState = OP_IDLE;

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
void handleMachineOperations();
void updateMachineSignals();
void updateLEDs();
void pauseAllTimers();
void resumeAllTimers();
void shiftAllTimers();

void setup() {
  //Serial.begin(115200);
  
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
  
  // Якщо станок зупинений - нічого не робимо
  if (machineState == MACHINE_STOPPED) {
    return;
  }
  
  // Якщо станок на паузі - зсуваємо таймери
  if (machineState == MACHINE_PAUSED) {
    shiftAllTimers();
    return;
  }
  
  // Основна логіка роботи станка
  handleMachineOperations();
}

// Обробка кнопок старт/стоп
void handleStartStopButtons() {
  if (controls.startPressed()) {
    if (machineState == MACHINE_STOPPED) {
      // Запуск станка
      machineState = MACHINE_RUNNING;
      operationState = OP_IDLE;
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
      operationState = OP_IDLE;
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

// Основна логіка операцій станка
void handleMachineOperations() {
  switch (operationState) {
    case OP_IDLE:
      // Запускаємо конвеєр і переходимо до очікування датчиків
      if (!conveyor.isRunning()) {
        conveyor.start();
      }
      operationState = OP_PAINT_WAIT_SENSOR;
      break;
      
    case OP_PAINT_WAIT_SENSOR:
      // Запускаємо конвеєр якщо він не працює
      if (!conveyor.isRunning()) {
        conveyor.start();
      }
      // Очікування спрацювання датчика 1
      if (controls.sensor1RisingEdge()) {
        if (paintIgnoreCount == 0) {
          // Перша баночка - обробляємо
          conveyor.stopWithDociag(JAR_CENTERING_MM);
          operationState = OP_PAINT_DOCIAG;
        } else {
          // Ігноруємо наступні 5 баночок
          paintIgnoreCount--;
        }
      }
      break;
      
    case OP_PAINT_DOCIAG:
      // Очікування завершення дотяжки
      if (!conveyor.isRunning()) {
        valve3.onFor(PAINT_PISTON_HOLD_TIME);
        operationState = OP_PAINT_PISTON;
      }
      break;
      
    case OP_PAINT_PISTON:
      // Очікування завершення роботи поршня
      if (!valve3.isTimerActive()) {
        paintDelayStart = millis();
        operationState = OP_PAINT_DELAY;
      }
      break;
      
    case OP_PAINT_DELAY:
      // Неблокуюча затримка після розливання
      if (millis() - paintDelayStart >= 50) {
        paintIgnoreCount = JARS_IN_SET - 1; // ігноруємо наступні 5 баночок
        operationState = OP_CAP_WAIT_SENSOR;
      }
      break;
      
    case OP_CAP_WAIT_SENSOR:
      // Запускаємо конвеєр якщо він не працює
      if (!conveyor.isRunning()) {
        conveyor.start();
        // Подаємо імпульс на PNEUMATIC_1 при відновленні руху після розливки
        valve1.onFor(PNEUMATIC1_ON_TIME_MS + PNEUMATIC1_HOLD_TIME_MS);
      }
      // Очікування спрацювання датчика 2
      if (controls.sensor2RisingEdge()) {
        if (capIgnoreCount == 0) {
          // Перша баночка - обробляємо
          conveyor.stop();
          valve4.on(); // увімкнення завертання кришок
          operationState = OP_CAP_SCREW_ON;
        } else {
          // Ігноруємо наступні 5 баночок
          capIgnoreCount--;
        }
      }
      break;
      
    case OP_CAP_SCREW_ON:
      // Пауза перед закриванням кришок
      capScrewPauseStart = millis();
      operationState = OP_CAP_SCREW_PAUSE;
      break;
      
    case OP_CAP_SCREW_PAUSE:
      // Неблокуюча пауза перед закриванням кришок
      if (millis() - capScrewPauseStart >= STEP_PAUSE_CAP_SCREW_MS) {
        valve5.onFor(CLOSE_CAP_HOLD_TIME);
        operationState = OP_CAP_CLOSE;
      }
      break;
      
    case OP_CAP_CLOSE:
      // Очікування завершення закривання кришок
      if (!valve5.isTimerActive()) {
        capClosePauseStart = millis();
        operationState = OP_CAP_CLOSE_PAUSE;
      }
      break;
      
    case OP_CAP_CLOSE_PAUSE:
      // Неблокуюча пауза після закривання кришок
      if (millis() - capClosePauseStart >= STEP_PAUSE_CAP_CLOSE_MS) {
        valve4.off(); // вимкнення завертання кришок
        capIgnoreCount = JARS_IN_SET - 1; // ігноруємо наступні 5 баночок
        operationState = OP_IDLE; // повертаємося до початку циклу
      }
      break;
      
    default:
      operationState = OP_IDLE;
      break;
  }
}

// Оновлення сигналів станка
void updateMachineSignals() {
  bool machineRunning = (machineState == MACHINE_RUNNING);
  digitalWrite(START_STOP_PIN, machineRunning ? HIGH : LOW);
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