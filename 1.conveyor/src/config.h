#ifndef CONFIG_H
#define CONFIG_H
// -------------------------
//ТАЙМІНГИ ТА ІНТЕРВАЛИ ПНЕВМОКЛАПАНІВ
// -------------------------

// Розлив фарби: один імпульс поршня (утримання в мс)
#define PAINT_PISTON_HOLD_TIME                  1000
// Другий пневмоциліндр для розливу фарби (утримання в мс)
#define PAINT_PISTON_2_HOLD_TIME                500
// ПНЕВМОКЛАПАН 1: час увімкнення та додаткове утримання (мс)
#define PNEUMATIC1_ON_TIME_MS                   1000  // скільки тримати сигнал активним спочатку
#define PNEUMATIC1_HOLD_TIME_MS                 400   // додаткове утримання після увімкнення
// Закривання кришок
#define STEP_PAUSE_CAP_SCREW_MS                 300   // пауза перед запуском Valve 5 після Valve 4
#define CLOSE_CAP_HOLD_TIME                     800  // утримання в положенні закривання (вимкнеться раніше за Valve 4)
#define STEP_PAUSE_CAP_CLOSE_MS                 300   // мінімальна пауза після Valve 5

// Кількість баночок у збірці
#define JARS_IN_SET             6     // кількість баночок у збірці

// -------------------------
// ПАРАМЕТРИ КОНВЕЄРА (XY) — основний конвеєр з ременем/шківом
// -------------------------
#define BELT_PITCH_MM_XY         2.0     // Крок ременя GT2 (мм)
#define PULLEY_TEETH_XY          20      // Кількість зубів на шківі
#define MICROSTEPS_XY            8       // Дріблення кроку (1/8)
#define MOTOR_STEPS_PER_REV_XY   200     // Кроків на оберт двигуна (звичайно 200)
// Швидкість конвеєра XY:
#define BELT_SPEED_XY_MM_PER_S   50.0    // Бажана швидкість у мм/с

// -------------------------
// ОБЧИСЛЕННЯ КІНЕМАТИКИ
// -------------------------

// Кроків на мм руху (XY): ремінь/шків
#define STEPS_PER_MM_XY ((MOTOR_STEPS_PER_REV_XY * MICROSTEPS_XY) / (BELT_PITCH_MM_XY * PULLEY_TEETH_XY))
// Загальна кількість кроків на секунду (XY)
#define STEPS_PER_SECOND_XY (STEPS_PER_MM_XY * BELT_SPEED_XY_MM_PER_S)
// Інтервал між кроками в мікросекундах (XY)
#define STEP_INTERVAL_XY_MICROS (1000000.0 / STEPS_PER_SECOND_XY)

// Тривалість STEP імпульсу
#define PULSE_WIDTH_MICROS 10
// Напрямки моторів
#define MOTOR_X_DIR LOW // Напрямок мотора X

// -------------------------
// ДАТЧИКИ ТА КНОПКИ
// -------------------------
#define SENSOR_POLL_INTERVAL      10    // мс, інтервал опитування датчиків
#define SENSOR_DEBOUNCE_TIME_MS   50    // мс, час антидребезгу для механічних сенсорів (рекомендовано 20-100мс)

#define JAR_CENTERING_MM 8.0 // На скільки мм зрушити баночку вперед після спрацювання датчика //8мм


#endif
