#ifndef PINOUT_H
#define PINOUT_H
// For RAMPS 1.4 
#define PNEUMATIC_1_PIN	17  // видача спайок (розподілювач №1)
#define PNEUMATIC_2_PIN	10  // другий пневмоциліндр для розливу фарби (розподілювач №2)
#define PNEUMATIC_3_PIN	16  // рух поршня для забору/видачі фарби (розподілювач №3)
#define PNEUMATIC_4_PIN	9   // завертання кришок баночок (розподілювач №4)
#define PNEUMATIC_5_PIN	8   // закривання кришок баночок (розподілювач №5)

//сигнали для інщих контролерів
#define START_STOP_PIN     11  // сигнал для старту/стопу іншого контролера 
#define START_CONVEYOR_PIN     6 // сигнал коли конвеєр рухається
// мотор конвеєра x 
#define X_STEP_PIN         54
#define X_DIR_PIN          55
#define X_ENABLE_PIN       38

//кінцеві вимикачі
#define sensor_1          14 //датчик наявності баночки під соплом роливу фарби(на платі як Y_MIN_PIN)
#define sensor_2          15 //датчик наявності баночки під прессом закривання кришки(на платі як Y_MAX_PIN)

// панель управління
#define start_PIN         18 // кнопка для запуску станка  підключено до Z_MIN_PIN
#define stop_PIN          19 // кнопка для зупинки станка  підключено до Z_MAX_PIN
//#define modeButtonPin      68  // кнопка для перемикання режимів роботи підключено до А14, на шилді T1
#define ledMode0Pin        20 // світлодіод для індикації станок працює
#define ledMode1Pin        21 // світлодіод для індикації станок в режимі очікування
//#define LED_PIN            13 // світлодіод індикації роботи(той що на платі як LED_BUILTIN)

#endif