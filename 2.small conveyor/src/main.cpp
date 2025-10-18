#include <Arduino.h>

/*
 * Конвеєр з розподілювачем №6 для упаковки баночок у шахматному порядку
 * 
 * Підключення:
 * - Драйвер крокового двигуна: STEP=2, DIR=5, EN=8
 * - Датчик: пін 9
 * - Пневмоклапан: пін 12
 * - Сигнальний світлодіод: пін 13
 * 
 * Налаштування мікростепів драйвера:
 * - 1x = повний крок (найшвидше, менша точність)
 * - 8x = 1/8 кроку (баланс швидкості та точності)
 * - 16x = 1/16 кроку (найповільніше, найвища точність)
 * 
 * Команди через Serial Monitor (9600 baud):
 * - micro:1, micro:8, micro:16 - змінити мікростепи
 * - speed:XX - змінити швидкість (мм/с)
 * - decel:XX - змінити коефіцієнт гальмування (0.1-1.0)
 * - status - показати поточний стан
 * - help - показати всі команди
 */

// ========== НАЛАШТУВАЛЬНІ ПАРАМЕТРИ ==========

// Параметри конвеєра
const int STEP_PIN = 4;           // Пін для кроків
const int DIR_PIN = 7;            // Пін для напрямку

const int ENABLE_PIN = 8;         // Пін для увімкнення драйвера
const int SENSOR_PIN = 9;         // Пін датчика
const int PNEUMATIC_PIN = 12;     // Пін пневмоклапана (інвертований сигнал: LOW=увімкнено, HIGH=вимкнено)
const int SIGNAL_PIN = 13;        // Пін сигналу готовності 4 спайок
const int START_STOP_PIN = 11;  // сигнал для старту/стопу іншого контролера

// Параметри двигуна
const float PULLEY_DIAMETER_MM = 40.0;    // Діаметр шківа в мм
const float DESIRED_SPEED_MM_S = 60.0;    // Бажана швидкість в мм/с
const int STEPS_PER_REVOLUTION = 200;     // Кроків на оберт (повний крок)

// Налаштування мікростепів драйвера
// Доступні значення: 1, 2, 4, 8, 16
int MICROSTEPS = 8;                       // Мікростепи (1 = повний крок, 8 = 1/8 кроку, 16 = 1/16 кроку)

// Параметри дотягування для шахматного порядку
const float CONVEYOR_Z_OFFSET_MM_FIRST = 10.0;   // Дотягування для 1-ї та 3-ї партії (мм)
const float CONVEYOR_Z_OFFSET_MM_SECOND = 2.0;  // Дотягування для 2-ї та 4-ї партії (мм)

// Параметри плавного гальмування
const float MIN_DECELERATION_DISTANCE_MM = 0.5;  // Мінімальна відстань для гальмування (мм)
const float MAX_DECELERATION_DISTANCE_MM = 8.0;  // Максимальна відстань для гальмування (мм)
float DECELERATION_FACTOR = 0.3;                 // Коефіцієнт гальмування (0.1 = дуже плавно, 0.5 = швидко)

// Параметри пневматики
const unsigned long PNEUMATIC_DELAY_MS = 2000;   // Час роботи пневматики (мс) для партій 1–3
const unsigned long CYL_EXTEND_TIME_MS = 1100;    // Час висування циліндра (мс) для 4-ї партії
const unsigned long CYL_HOLD_TIME_MS = 2000;     // Час утримання у висунутому стані (мс) для 4-ї партії

// Параметри сигналу
const unsigned long SIGNAL_DELAY_MS = 5000;      // Час сигналу після 4 партій (мс)

// ========== РОЗРАХУНКОВІ ПАРАМЕТРИ ==========

// Розрахунок кроків на мм (буде перераховано при зміні мікростепів)
float MM_PER_STEP;
float STEPS_PER_MM;

// Розрахунок затримки між кроками для бажаної швидкості (буде перераховано)
unsigned long STEP_DELAY_US;

// Мінімальна затримка між кроками (для стабільності)
const unsigned long MIN_STEP_DELAY_US = 1000; // 1мс мінімум

// Змінна для налаштування швидкості (можна змінювати через серіальний порт)
float currentSpeed = DESIRED_SPEED_MM_S;

// ========== ЗМІННІ СТАНУ ==========

enum ConveyorState {
  IDLE,           // Очікування
  MOVING,         // Рух конвеєра
  SENSOR_TRIGGERED, // Датчик спрацював
  PULLING,        // Дотягування
  PNEUMATIC_WORKING, // Робота пневматики
  SIGNAL_ACTIVE   // Сигнал після 4 партій
};

ConveyorState currentState = IDLE;
int batchCount = 0;                    // Лічильник партій
bool sensorState = false;              // Поточний стан датчика
bool lastSensorState = false;          // Попередній стан датчика
unsigned long stateStartTime = 0;      // Час початку поточного стану
float currentOffset = 0;               // Поточне дотягування
bool ignoreSensor = false;             // Ігнорувати датчик під час роботи пневматики

// ========== ПРОТОТИПИ ФУНКЦІЙ ==========

void handleIdleState();
void handleMovingState();
void handleSensorTriggeredState();
void handlePullingState();
void handlePneumaticWorkingState();
void handleSignalActiveState();
void performPull(float offsetMm);
void performSmoothPull(float offsetMm);
float calculateDecelerationDistance(float totalDistance);
void checkSerialCommands();
void recalculateParameters();

// ========== ФУНКЦІЇ ==========

void setup() {
  // Налаштування пінів
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(SENSOR_PIN, INPUT_PULLUP);
  pinMode(PNEUMATIC_PIN, OUTPUT);
  pinMode(SIGNAL_PIN, OUTPUT);
  pinMode(START_STOP_PIN, INPUT);

  // Початкові стани
  digitalWrite(ENABLE_PIN, HIGH);      // Вимкнути драйвер
  digitalWrite(PNEUMATIC_PIN, HIGH);   // Вимкнути пневматику (інвертований сигнал)
  digitalWrite(SIGNAL_PIN, LOW);       // Вимкнути сигнал
  
  // Налаштування серіального порту для налагодження
  Serial.begin(9600);
  
  // Розрахувати початкові параметри
  recalculateParameters();
  
  Serial.println("Конвеєр з розподілювачем №6 запущено");
  Serial.println("Параметри:");
  Serial.print("Швидкість: "); Serial.print(DESIRED_SPEED_MM_S); Serial.println(" мм/с");
  Serial.print("Мікростепи: "); Serial.print(MICROSTEPS); Serial.println("x");
  Serial.print("Кроків на мм: "); Serial.println(STEPS_PER_MM);
  Serial.print("Розрахована затримка: "); Serial.print(STEP_DELAY_US); Serial.println(" мкс");
  Serial.print("Мінімальна затримка: "); Serial.print(MIN_STEP_DELAY_US); Serial.println(" мкс");
  Serial.print("Фактична затримка: "); Serial.print(max(STEP_DELAY_US - 10, MIN_STEP_DELAY_US)); Serial.println(" мкс");
  Serial.print("Коефіцієнт гальмування: "); Serial.println(DECELERATION_FACTOR);
  Serial.print("Відстань гальмування: "); Serial.print(MIN_DECELERATION_DISTANCE_MM); 
  Serial.print(" - "); Serial.print(MAX_DECELERATION_DISTANCE_MM); Serial.println(" мм");
  
  currentState = IDLE;
}

void loop() {
  // Перевірка дозволу роботи з відслідковуванням фронту сигналу START/STOP
  static bool lastStartSignalHigh = false; // запам'ятовуємо попередній стан сигналу
  bool startSignalHigh = (digitalRead(START_STOP_PIN) == HIGH);

  if (!startSignalHigh) {
    // При низькому рівні зупиняємо все
    digitalWrite(ENABLE_PIN, HIGH);      // Вимкнути драйвер
    digitalWrite(PNEUMATIC_PIN, HIGH);   // Вимкнути пневматику
    digitalWrite(SIGNAL_PIN, LOW);       // Вимкнути сигнал
    lastStartSignalHigh = false;         // фіксуємо, що сигнал був LOW
    return; // Вихід з loop() - нічого далі не виконується
  }

  // На фронті (LOW -> HIGH) готуємо систему до коректного перезапуску
  if (startSignalHigh && !lastStartSignalHigh) {
    ignoreSensor = false;                // зняти ігнорування датчика
    currentState = IDLE;                 // перейти у стан ініціалізації руху
    // драйвер буде увімкнено в handleIdleState()
  }
  lastStartSignalHigh = startSignalHigh;

  // Перевірка команд через серіальний порт
  checkSerialCommands();
  
  // Читання стану датчика
  sensorState = digitalRead(SENSOR_PIN) == LOW; // LOW = спрацював (підтяжка до VCC)
  
  // Обробка станів
  switch (currentState) {
    case IDLE:
      handleIdleState();
      break;
      
    case MOVING:
      handleMovingState();
      break;
      
    case SENSOR_TRIGGERED:
      handleSensorTriggeredState();
      break;
      
    case PULLING:
      handlePullingState();
      break;
      
    case PNEUMATIC_WORKING:
      handlePneumaticWorkingState();
      break;
      
    case SIGNAL_ACTIVE:
      handleSignalActiveState();
      break;
  }
  
  // Оновлення попереднього стану датчика
  lastSensorState = sensorState;
  
  // Невелика затримка тільки для станів, де не потрібен постійний рух
  if (currentState != MOVING && currentState != PULLING) {
    delay(10);
  }
}

void handleIdleState() {
  // Увімкнути драйвер і почати рух
  digitalWrite(ENABLE_PIN, LOW);
  digitalWrite(DIR_PIN, HIGH); // Напрямок руху
  currentState = MOVING;
  stateStartTime = millis();
  Serial.println("Конвеєр почав рух");
}

void handleMovingState() {
  // Виконання кроку
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(STEP_PIN, LOW);
  
  // Розрахувати затримку на основі поточної швидкості та поточних мікростепів
  unsigned long stepDelay = (unsigned long)(1000000.0 / (currentSpeed * STEPS_PER_MM));
  unsigned long actualDelay = max(stepDelay - 10, MIN_STEP_DELAY_US);
  delayMicroseconds(actualDelay);
  
  // Перевірка датчика (тільки якщо не ігноруємо)
  if (!ignoreSensor && sensorState && !lastSensorState) {
    // Датчик спрацював
    currentState = SENSOR_TRIGGERED;
    stateStartTime = millis();
    Serial.println("Датчик спрацював!");
  }
}

void handleSensorTriggeredState() {
  // Зупинити конвеєр
  digitalWrite(ENABLE_PIN, HIGH);
  
  // Визначити яка це партія і відповідне дотягування
  batchCount++;
  if (batchCount == 1 || batchCount == 3) {
    currentOffset = CONVEYOR_Z_OFFSET_MM_FIRST;
  } else {
    currentOffset = CONVEYOR_Z_OFFSET_MM_SECOND;
  }
  
  Serial.print("=== ПАРТІЯ "); Serial.print(batchCount); Serial.println(" ===");
  Serial.print("Дотягування: "); Serial.print(currentOffset); Serial.println(" мм");
  Serial.println("Пневматика буде активна на цій зупинці");
  
  // Встановити ігнорування датчика
  ignoreSensor = true;
  
  currentState = PULLING;
  stateStartTime = millis();
}

void handlePullingState() {
  // Виконати плавне дотягування
  performSmoothPull(currentOffset);
  
  // Перейти до роботи пневматики
  currentState = PNEUMATIC_WORKING;
  stateStartTime = millis();
  Serial.print("Дотягування завершено, запуск пневматики на "); 
  Serial.print(PNEUMATIC_DELAY_MS); Serial.println(" мс");
}

void handlePneumaticWorkingState() {
  // Особлива логіка для 4-ї партії: спочатку повний цикл циліндра (висування + утримання + згортання), потім сигнал
  if (batchCount == 4) {
    unsigned long elapsed = millis() - stateStartTime; // загальний час у цьому стані

    // Етап 1: лише висування — до CYL_EXTEND_TIME_MS
    if (elapsed < CYL_EXTEND_TIME_MS) {
      if (digitalRead(PNEUMATIC_PIN) != LOW) {
        digitalWrite(PNEUMATIC_PIN, LOW);
        Serial.println("4-та партія: циліндр увімкнено (висування)");
      }
      if (digitalRead(SIGNAL_PIN) != LOW) {
        digitalWrite(SIGNAL_PIN, LOW);
      }
      return;
    }

    // Етап 2: після висування — одночасно стартують тривалість утримання і сигналу
    unsigned long phaseElapsed = elapsed - CYL_EXTEND_TIME_MS; // час від старту етапу 2

    // Увімкнути сигнал на початку етапу 2 (один раз)
    if (digitalRead(SIGNAL_PIN) == LOW) {
      digitalWrite(SIGNAL_PIN, HIGH);
      Serial.println("4-та партія: сигнал увімкнено (старт одночасно з утриманням)");
    }

    // Тримати циліндр увімкненим під час утримання, потім вимкнути
    if (phaseElapsed < CYL_HOLD_TIME_MS) {
      if (digitalRead(PNEUMATIC_PIN) != LOW) {
        digitalWrite(PNEUMATIC_PIN, LOW);
      }
    } else {
      if (digitalRead(PNEUMATIC_PIN) == LOW) {
        digitalWrite(PNEUMATIC_PIN, HIGH);
        Serial.println("4-та партія: циліндр вимкнено (після утримання)");
      }
    }

    // Завершити сигнал після його власного таймера від старту етапу 2
    if (phaseElapsed >= SIGNAL_DELAY_MS) {
      if (digitalRead(SIGNAL_PIN) == HIGH) {
        digitalWrite(SIGNAL_PIN, LOW);
      }
      batchCount = 0;
      ignoreSensor = false;
      currentState = IDLE;
      Serial.println("4-та партія: сигнал завершено, початок нового циклу");
    }
    return;
  }

  // Звичайна логіка для партій 1–3
  // Увімкнути пневматику (інвертований сигнал)
  digitalWrite(PNEUMATIC_PIN, LOW);

  // Перевірити чи минув час роботи пневматики
  if (millis() - stateStartTime >= PNEUMATIC_DELAY_MS) {
    // Вимкнути пневматику (інвертований сигнал)
    digitalWrite(PNEUMATIC_PIN, HIGH);
    // Відновити рух конвеєра для наступної партії
    ignoreSensor = false;
    currentState = IDLE;
    Serial.print("Партія "); Serial.print(batchCount); Serial.print(" завершена, залишилось партій: ");
    Serial.print(4 - batchCount); Serial.println(", відновлення руху");
  }
}

void handleSignalActiveState() {
  // Перевірити чи минув час сигналу
  if (millis() - stateStartTime >= SIGNAL_DELAY_MS) {
    // Вимкнути сигнал і скинути все
    digitalWrite(SIGNAL_PIN, LOW);
    batchCount = 0;  // Скинути лічильник партій
    ignoreSensor = false;
    currentState = IDLE;
    Serial.println("Сигнал завершено, скидання системи, початок нового циклу");
  }
}

void performPull(float offsetMm) {
  // Розрахувати кількість кроків для дотягування
  int steps = (int)(offsetMm * STEPS_PER_MM);
  
  Serial.print("Виконуємо дотягування на "); Serial.print(offsetMm); 
  Serial.print(" мм ("); Serial.print(steps); Serial.println(" кроків)");
  
  // Увімкнути драйвер
  digitalWrite(ENABLE_PIN, LOW);
  digitalWrite(DIR_PIN, HIGH); // Напрямок дотягування
  
  // Виконати кроки
  for (int i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(STEP_PIN, LOW);
    
    // Розрахувати затримку на основі поточної швидкості та поточних мікростепів
    unsigned long stepDelay = (unsigned long)(1000000.0 / (currentSpeed * STEPS_PER_MM));
    unsigned long actualDelay = max(stepDelay - 10, MIN_STEP_DELAY_US);
    delayMicroseconds(actualDelay);
  }
  
  // Вимкнути драйвер
  digitalWrite(ENABLE_PIN, HIGH);
}

void checkSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command.startsWith("speed:")) {
      float newSpeed = command.substring(6).toFloat();
      if (newSpeed > 0 && newSpeed <= 200) {
        currentSpeed = newSpeed;
        Serial.print("Швидкість змінено на: "); Serial.print(currentSpeed); Serial.println(" мм/с");
      } else {
        Serial.println("Невірна швидкість! Діапазон: 0.1 - 200 мм/с");
      }
    } else if (command.startsWith("micro:")) {
      int newMicrosteps = command.substring(6).toInt();
      if (newMicrosteps == 1 || newMicrosteps == 2 || newMicrosteps == 4 || 
          newMicrosteps == 8 || newMicrosteps == 16) {
        MICROSTEPS = newMicrosteps;
        recalculateParameters();
        Serial.print("Мікростепи змінено на: "); Serial.print(MICROSTEPS); Serial.println("x");
        Serial.print("Нові кроки на мм: "); Serial.println(STEPS_PER_MM);
        Serial.print("Нова затримка: "); Serial.print(STEP_DELAY_US); Serial.println(" мкс");
      } else {
        Serial.println("Невірні мікростепи! Доступні: 1, 2, 4, 8, 16");
      }
    } else if (command.startsWith("decel:")) {
      float newDecelFactor = command.substring(6).toFloat();
      if (newDecelFactor >= 0.1 && newDecelFactor <= 1.0) {
        DECELERATION_FACTOR = newDecelFactor;
        Serial.print("Коефіцієнт гальмування змінено на: "); Serial.println(DECELERATION_FACTOR);
      } else {
        Serial.println("Невірний коефіцієнт гальмування! Діапазон: 0.1 - 1.0");
      }
    } else if (command == "status") {
      Serial.print("Поточна швидкість: "); Serial.print(currentSpeed); Serial.println(" мм/с");
      Serial.print("Мікростепи: "); Serial.print(MICROSTEPS); Serial.println("x");
      Serial.print("Кроків на мм: "); Serial.println(STEPS_PER_MM);
      Serial.print("Коефіцієнт гальмування: "); Serial.println(DECELERATION_FACTOR);
      Serial.print("Стан: "); Serial.println(currentState);
      Serial.print("Партія: "); Serial.println(batchCount);
    } else if (command == "help") {
      Serial.println("Команди:");
      Serial.println("speed:XX - встановити швидкість (наприклад: speed:30)");
      Serial.println("micro:XX - встановити мікростепи (1, 2, 4, 8, 16)");
      Serial.println("decel:XX - встановити коефіцієнт гальмування (0.1-1.0)");
      Serial.println("status - показати поточний стан");
      Serial.println("help - показати цю довідку");
    }
  }
}

void recalculateParameters() {
  // Перерахунок кроків на мм
  MM_PER_STEP = (PULLEY_DIAMETER_MM * PI) / (STEPS_PER_REVOLUTION * MICROSTEPS);
  STEPS_PER_MM = 1.0 / MM_PER_STEP;
  
  // Перерахунок затримки між кроками
  STEP_DELAY_US = (unsigned long)(1000000.0 / (DESIRED_SPEED_MM_S * STEPS_PER_MM));
}

float calculateDecelerationDistance(float totalDistance) {
  // Розрахувати відстань гальмування на основі загальної відстані
  // Для коротких відстаней - більш різке гальмування
  // Для довгих відстаней - більш плавне гальмування
  
  float decelDistance;
  
  if (totalDistance <= 3.0) {
    // Для коротких відстаней (до 3 мм) - швидке гальмування
    decelDistance = MIN_DECELERATION_DISTANCE_MM;
  } else if (totalDistance <= 8.0) {
    // Для середніх відстаней (3-8 мм) - пропорційне гальмування
    decelDistance = MIN_DECELERATION_DISTANCE_MM + 
                   (totalDistance - 3.0) * (MAX_DECELERATION_DISTANCE_MM - MIN_DECELERATION_DISTANCE_MM) / 5.0;
  } else {
    // Для довгих відстаней (більше 8 мм) - максимальне плавне гальмування
    decelDistance = MAX_DECELERATION_DISTANCE_MM;
  }
  
  // Переконатися, що відстань гальмування не перевищує половину загальної відстані
  decelDistance = min(decelDistance, totalDistance * 0.5);
  
  return decelDistance;
}

void performSmoothPull(float offsetMm) {
  // Розрахувати кількість кроків для дотягування
  int totalSteps = (int)(offsetMm * STEPS_PER_MM);
  
  // Розрахувати відстань гальмування
  float decelDistanceMm = calculateDecelerationDistance(offsetMm);
  int decelSteps = (int)(decelDistanceMm * STEPS_PER_MM);
  
  // Кроки з постійною швидкістю
  int constantSpeedSteps = totalSteps - decelSteps;
  
  Serial.print("Виконуємо плавне дотягування на "); Serial.print(offsetMm); 
  Serial.print(" мм ("); Serial.print(totalSteps); Serial.println(" кроків)");
  Serial.print("Відстань гальмування: "); Serial.print(decelDistanceMm); 
  Serial.print(" мм ("); Serial.print(decelSteps); Serial.println(" кроків)");
  
  // Увімкнути драйвер
  digitalWrite(ENABLE_PIN, LOW);
  digitalWrite(DIR_PIN, HIGH); // Напрямок дотягування
  
  // Розрахувати базову затримку для поточної швидкості
  unsigned long baseStepDelay = (unsigned long)(1000000.0 / (currentSpeed * STEPS_PER_MM));
  baseStepDelay = max(baseStepDelay - 10, MIN_STEP_DELAY_US);
  
  // Етап 1: Рух з постійною швидкістю
  for (int i = 0; i < constantSpeedSteps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(baseStepDelay);
  }
  
  // Етап 2: Плавне гальмування
  for (int i = 0; i < decelSteps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(STEP_PIN, LOW);
    
    // Розрахувати поточну затримку для гальмування
    // Використовуємо квадратичну функцію для плавного збільшення затримки
    float progress = (float)i / (float)decelSteps; // 0.0 до 1.0
    float decelFactor = 1.0 + (progress * progress * DECELERATION_FACTOR * 10.0);
    
    unsigned long currentDelay = (unsigned long)(baseStepDelay * decelFactor);
    currentDelay = max(currentDelay, MIN_STEP_DELAY_US);
    
    delayMicroseconds(currentDelay);
  }
  
  // Вимкнути драйвер
  digitalWrite(ENABLE_PIN, HIGH);
  
  Serial.println("Плавне дотягування завершено");
}
