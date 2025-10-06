#include <Arduino.h>
/*
 * Оновлена логіка управління вакуумним краном:
 * - Пін 10: Керування пневморозподілювачем (2 положення)
 *   * LOW - Позиція 1: Подача вакууму на присоски для захвату пакету
 *   * HIGH - Позиція 2: Переключення на вакуумування пакету
 * - Пін 11: Керування електроклапаном скидання тиску
 *   * Активація після запайки пакету лентою коли силіконова планка піднімається
 * 
 * ІНВЕРТОВАНА ЛОГІКА ЦИЛІНДРІВ:
 * - true (flagState) = LOW (циліндр висувається)
 * - false (flagState) = HIGH (циліндр засувається)
 * - Початковий стан всіх циліндрів: HIGH (засунуті)
 */
// Конфігурація затримок (в мілісекундах)
// Циліндри
const int DELAY_DIST_7_MOVE = 800;      // Платформа з присосками (400мм)
const int DELAY_DIST_8_UP_DOWN = 500;   // Піднімання/опускання платформи
const int DELAY_DIST_8_OUT_PACET = 200;   // відкривання пакету
const int DELAY_DIST_9_MOVE = 1200;      // Рух циліндра вперед/назад
const int DELAY_DIST_10_MOVE = 200;     // Утримання спайок і пакета
const int DELAY_DIST_11_MOVE = 400;     // Рух соплом вперед/назад
const int DELAY_DIST_12_MOVE = 400;     // Силіконова планка для запайки
const int DELAY_DIST_13_MOVE = 300;     // Скидання готового пакету
const int DELAY_DIST_14_MOVE = 500;     // Охолодження ленти
const int DELAY_VACUM_SOPLO = 1000;     // Затримка після переключення клапана на вакуумування перед відпусканням присосок

// Процеси
const int DELAY_HEATING = 1700;          // Час нагріву
const int DELAY_HEATING_POSLE = 500;    // Час переачі тепла від ленти після виключення нагріву
const int DELAY_COOLING = 0;          // Час охолодження
const int DELAY_PARALLEL_CYLINDERS = 50; // Затримка між активацією паралельних циліндрів

const int DELAY_BETWEEN_CYCLES = 2000;  // 2 секунди паузи між циклами

enum VacuumValvePosition {
    VALVE_POS_1 = 1, // Подача вакууму на присоски для захвату пакету
    VALVE_POS_2 = 2  // Переключення на вакуумування пакету
};

// Пневморозподілювачі (пини)
#define DIST_7 2   // Платформа з присосками (400мм)
#define DIST_8 3   // Піднімання/опускання платформи
#define DIST_9 4   // Рух циліндра вперед/назад
#define DIST_10 5  // Утримання спайок і пакета
#define DIST_11 6  // Рух соплом вперед/назад
#define DIST_12 7  // Силіонова планка для запайки
#define DIST_13 8  // Скидання готового пакету
#define DIST_14 9  // охолодження ленти
#define VACUUM_VALVE_PIN 10  // Керування пневморозподілювачем вакууму (2 положення)
#define PRESSURE_RELEASE_VALVE_PIN 11  // Керування електроклапаном скидання тиску
#define PIN_IN_RELE 12

#define SIGNAL_PIN A0        // Пін сигналу готовності 4 спайок
#define START_STOP_PIN A2  // сигнал для старту/стопу  контролера

void setup() {
  // Налаштування пінів як виходи
  pinMode(DIST_7, OUTPUT);
  pinMode(DIST_8, OUTPUT);
  pinMode(DIST_9, OUTPUT);
  pinMode(DIST_10, OUTPUT);
  pinMode(DIST_11, OUTPUT);
  pinMode(DIST_12, OUTPUT);
  pinMode(DIST_13, OUTPUT);
  pinMode(DIST_14, OUTPUT);
  pinMode(VACUUM_VALVE_PIN, OUTPUT);
  pinMode(PRESSURE_RELEASE_VALVE_PIN, OUTPUT);
  pinMode(PIN_IN_RELE, OUTPUT);
  pinMode(SIGNAL_PIN, INPUT);
  pinMode(START_STOP_PIN, INPUT);

  // Всі розподілювачі вимкнені (інвертовано для циліндрів)
  digitalWrite(DIST_7, HIGH);  // Інвертовано: циліндри в початковому положенні (засунуті)
  digitalWrite(DIST_8, HIGH);
  digitalWrite(DIST_9, HIGH);
  digitalWrite(DIST_10, HIGH);
  digitalWrite(DIST_11, HIGH);
  digitalWrite(DIST_12, HIGH);
  digitalWrite(DIST_13, HIGH);
  digitalWrite(DIST_14, HIGH);
  digitalWrite(VACUUM_VALVE_PIN, HIGH);  // Вакуум і клапан скидання залишаються без змін
  digitalWrite(PRESSURE_RELEASE_VALVE_PIN, HIGH);
  digitalWrite(PIN_IN_RELE, LOW);
}

inline void setVacuumValve(uint8_t position) {
    switch (position) {
        case VALVE_POS_1:
            // Подача вакууму на присоски для захвату пакету
            digitalWrite(VACUUM_VALVE_PIN, HIGH);
            break;
        case VALVE_POS_2:
            // Переключення на вакуумування пакету
            digitalWrite(VACUUM_VALVE_PIN, LOW);
            break;
        default:
            // Можна додати обробку помилки
            break;
    }
}

// Функція керування електроклапаном скидання тиску
inline void setPressureReleaseValve(bool state) {
    digitalWrite(PRESSURE_RELEASE_VALVE_PIN, state ? HIGH : LOW);
}
void cylinderActivate(int pin, int duration,bool flagState) {
  if(flagState){
      digitalWrite(pin, LOW);  // Інвертовано: true = LOW (висування)
  }else{
      digitalWrite(pin, HIGH); // Інвертовано: false = HIGH (засування)
  }
  
  delay(duration); // затримка в мілісекундах
}

// Функція для паралельної активації двох циліндрів
void cylindersActivateParallel(int pin1, int pin2, int duration1, int duration2, bool flagState1, bool flagState2, int delay_between = 0) {
  // Активація першого циліндра (інвертовано)
  digitalWrite(pin1, flagState1 ? LOW : HIGH);
  
  // Затримка між активаціями (якщо потрібна)
  if (delay_between > 0) {
    delay(delay_between);
  }
  
  // Активація другого циліндра (інвертовано)
  digitalWrite(pin2, flagState2 ? LOW : HIGH);
  
  // Очікування завершення роботи обох циліндрів (беремо максимальний час)
  delay(max(duration1, duration2));
}

void vacuumPackage() {
  // Переключення на вакуумування пакету
  setVacuumValve(VALVE_POS_2);
}

void heatingOn() {
  // Розжарювання ленти
  digitalWrite(PIN_IN_RELE, HIGH);
}

void heatingOff() {
  // Виключення нагріву
  digitalWrite(PIN_IN_RELE, LOW);
}

void coolingOn() {
  // Включення охолодження ленти
  cylinderActivate(DIST_14, DELAY_DIST_14_MOVE, true);
  delay(DELAY_COOLING); // час охолодження в мілісекундах
  cylinderActivate(DIST_14, DELAY_DIST_14_MOVE, false);
}

// Функція підготовки пакету (сигнал СТАРТ)
void preparePackage() {
  // Початкове положення: платформа з присосками над складом з пакетами
  
  // 2.1. Опускання платформи з присосками
  cylinderActivate(DIST_8, DELAY_DIST_8_UP_DOWN, true);
  
  // 2.2. Подання вакууму на присоски
  setVacuumValve(VALVE_POS_1);
  
  // 2.3. Піднімання платформи разом із пакетом
  cylinderActivate(DIST_8, DELAY_DIST_8_UP_DOWN, false);
  
  // 3.1. Пересування платформи з пакетом у зону завантаження
  cylinderActivate(DIST_7, DELAY_DIST_7_MOVE, true);
  
  // 3.2. Опускання платформи з пакетом, пакет ще закритий
  cylinderActivate(DIST_8, DELAY_DIST_8_OUT_PACET, true);
  
  // 3.3. Відкривання пакету: піднімання платформи з присосками, пакет відкрито
  cylinderActivate(DIST_8, DELAY_DIST_8_OUT_PACET, false);
  
  // Результат: відкритий порожній пакет готовий для завантаження
}

// Функція пакування (сигнал ГОТОВНІСТЬ)
void packageSpikes() {
  // 1.1. Засування спайок з платформи в пакет (циліндр вперед, утримання положення)
  cylinderActivate(DIST_9, DELAY_DIST_9_MOVE, true);
  
  // 1.2. Фіксація пакету: циліндр утримання висунутий
  cylinderActivate(DIST_10, DELAY_DIST_10_MOVE, true);
  
  // 2.1. Сопло відходить назад до початку пакету
  cylinderActivate(DIST_11, DELAY_DIST_11_MOVE, true);
  
  // 2.2. Клапан у режим вакуумування пакету, початок відкачки повітря
  //vacuumPackage();
  setVacuumValve(VALVE_POS_2);
  // 2.3. Водночас присоски відпускають пакет
  //setVacuumValve(VALVE_POS_1);
 // тут затримка 500 мс//////////////////////////////////////////////////////
  delay(DELAY_VACUM_SOPLO);
  // 3.1. Опускання силіконової планки
  cylinderActivate(DIST_12, DELAY_DIST_12_MOVE, true);
  
  // 3.2. Розжарювання ленти (активація нагріву)
  heatingOn();
  delay(DELAY_HEATING); // час нагріву в мілісекундах
  heatingOff();
  delay(DELAY_HEATING_POSLE); // час нагріву в мілісекундах після виключення нагріву
  // 3.3. Вимкнення вакууму, клапан у положення "атмосфера"
  //setVacuumValve(VALVE_POS_1);
  
  // 3.4. Піднімання планки
  cylinderActivate(DIST_12, DELAY_DIST_12_MOVE, false);
  
  // 3.5. Активація клапана скидання тиску після піднімання силіконової планки
  setPressureReleaseValve(true);

  // 3.6. Увімкнення охолодження ленти на заданий час
  coolingOn();
  
  // 4.2-4.3. Паралельна робота: сопло рухається вперед + циліндр засовування спайок повертається
  // Невелика затримка між активаціями для безпеки
  cylindersActivateParallel(DIST_11, DIST_9, DELAY_DIST_11_MOVE, DELAY_DIST_9_MOVE, false, false, DELAY_PARALLEL_CYLINDERS);
  

  // 4.4. Платформа з присосками повертається над склад з пакетами
  cylinderActivate(DIST_7, DELAY_DIST_7_MOVE, false);
     // 4.1. Піднімання циліндра утримання пакету
  cylinderActivate(DIST_10, DELAY_DIST_10_MOVE, false);

  // 4.5. Скидання готового пакету з платформи
  cylinderActivate(DIST_13, DELAY_DIST_13_MOVE, true);  // Висування циліндра скидання
  cylinderActivate(DIST_13, DELAY_DIST_13_MOVE, false); // Засування циліндра скидання
  setVacuumValve(VALVE_POS_1);// Відновлення подачі вакууму на присоски для захвату пакету
  setPressureReleaseValve(false);// Вимкнення клапана скидання тиску
  // Результат: спайки упаковані, пакет запаяний, готовий виріб скинуто
}

void loop() {
  // Перевіряємо сигнал START_STOP_PIN
  if (digitalRead(START_STOP_PIN) == HIGH) {
    // Запускаємо підготовку пакету
    preparePackage();
    
    // Чекаємо поки обидва сигнали будуть активні
    while (true) {
      if (digitalRead(START_STOP_PIN) == HIGH && digitalRead(SIGNAL_PIN) == HIGH) {
        // Якщо обидва сигнали активні - запускаємо пакування
        packageSpikes();
        
        // Пауза перед наступним циклом
        delay(DELAY_BETWEEN_CYCLES);
        break;  // Виходимо з внутрішнього циклу
      }
      delay(100);  // Невелика затримка щоб не навантажувати процесор
    }
  }
  delay(100);  // Затримка в головному циклі
}