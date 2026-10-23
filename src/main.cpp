#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// --- НАСТРОЙКИ ДИСПЛЕЯ OLED 0.96" (128x64) ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1      // Общий сброс (не используется)
#define OLED_ADDR     0x3C    // Стандартный I2C адрес дисплея (иногда 0x3D)

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_ADS1115 ads;         // Стандартный I2C адрес ADS1115 — 0x48

// --- ПИНЫ I2C ДЛЯ ESP32 ---
#define I2C_SDA 21
#define I2C_SCL 22

// --- КАЛИБРОВОЧНЫЕ КОЭФФИЦИЕНТЫ ---
// 1. Датчик тока ACS758LCB-100U (паспортные данные при VCC = 5.0 В)
const float SENSITIVITY_AT_5V = 0.0129; // 40 мВ/А = 0.040 В/А при 5.0В

// Коэффициент нулевой точки (отношение VIOUT при 0 А к питанию VCC).
// По даташиту при 5.0 В нулевая точка составляет 0.50 В (0.10 * VCC) или 0.60 В (0.12 * VCC).
// При первом запуске без нагрузки этот коэффициент определится автоматически в setup()!
float zeroRatio = 0.12; 

// 2. Делитель напряжения батареи на канале A2 (например, два одинаковых резистора 100к + 100к)
const float BAT_DIVIDER_RATIO = 2.0; 

// 3. Делитель силовой линии (до 30 В) на канале A3 (100 кОм и 10 кОм -> (100+10)/10 = 11.0)
const float LOAD_DIVIDER_RATIO = 11.0;

// Количество выборок для усреднения (убирает мелкое дрожание цифр)
const int SAMPLES = 10;

// --- ФУНКЦИЯ АВТОКАЛИБРОВКИ НУЛЯ ТОКА ПРИ ВКЛЮЧЕНИИ ---
void calibrateZeroCurrent() {
  double sumRatio = 0;
  for (int i = 0; i < 20; i++) {
    float v_out = ads.computeVolts(ads.readADC_SingleEnded(0));
    float v_cc  = ads.computeVolts(ads.readADC_SingleEnded(1));
    if (v_cc > 2.5) {
      sumRatio += (v_out / v_cc);
    }
    delay(10);
  }
  zeroRatio = sumRatio / 20.0;
}

// --- ФУНКЦИЯ РАСЧЕТА ПРОЦЕНТА ЗАРЯДА LI-ION (3.2V - 4.2V) ---
int getBatteryPercent(float v_bat) {
  int pct = (int)((v_bat - 3.20) * 100.0 / (4.20 - 3.20));
  if (pct > 100) pct = 100;
  if (pct < 0)   pct = 0;
  return pct;
}

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);

  // Инициализация OLED дисплея
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println(F("Ошибка: OLED дисплей не найден!"));
    while (true) delay(100);
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // Инициализация ADS1115
  // GAIN_ONE: диапазон ±4.096 В (1 бит = 0.125 мВ). Безопасно для питания 3.3 В!
  ads.setGain(GAIN_ONE);
  ads.setDataRate(RATE_ADS1115_250SPS); // Ускоряем замеры до 250 выборок/сек для быстрой фильтрации

  if (!ads.begin()) {
    display.setTextSize(1);
    display.setCursor(0, 20);
    display.println(F("ADS1115 Error!"));
    display.display();
    Serial.println(F("Ошибка: ADS1115 не найден!"));
    while (true) delay(100);
  }

  // Заставка и калибровка нуля (при включении нагрузка должна быть 0 А)
  display.setTextSize(1);
  display.setCursor(10, 4);
  display.println(F("AMPEREMETER INIT"));
  display.setCursor(10, 28);
  display.println(F("Zero calibration..."));
  display.display();

  calibrateZeroCurrent(); // Запоминаем точный "ноль" датчика относительно текущего VCC
  delay(500);
}

void loop() {
  double sum_a0 = 0, sum_a1 = 0, sum_a2 = 0, sum_a3 = 0;

  // Считываем и усредняем показания всех 4 каналов
  for (int i = 0; i < SAMPLES; i++) {
    sum_a0 += ads.computeVolts(ads.readADC_SingleEnded(0)); // A0: Выход датчика VIOUT
    sum_a1 += ads.computeVolts(ads.readADC_SingleEnded(1)); // A1: Питание датчика VCC (~3.3 В)
    sum_a2 += ads.computeVolts(ads.readADC_SingleEnded(2)); // A2: Делитель АКБ
    sum_a3 += ads.computeVolts(ads.readADC_SingleEnded(3)); // A3: Делитель силовой линии
  }

  float v_viout = sum_a0 / SAMPLES;
  float v_vcc   = sum_a1 / SAMPLES;
  float v_a2    = sum_a2 / SAMPLES;
  float v_a3    = sum_a3 / SAMPLES;

  // 1. Расчет тока с логометрической компенсацией питания (по даташиту ACS758)
  float currentSensitivity = SENSITIVITY_AT_5V * (v_vcc / 5.0); // ~0.0264 В/А при 3.3 В
  float v_zero             = v_vcc * zeroRatio;                 // Текущий "ноль" в Вольтах
  float currentAmps        = (v_viout - v_zero) / currentSensitivity;

  // Отсекаем мелкий шум в районе нуля (меньше 0.08 А считаем нулем)
  if (currentAmps > -0.08 && currentAmps < 0.08) {
    currentAmps = 0.0;
  }

  // 2. Расчет напряжения АКБ (канал A2)
  float batteryVolts = v_a2 * BAT_DIVIDER_RATIO;
  int batteryPct     = getBatteryPercent(batteryVolts);

  // 3. Расчет напряжения силовой линии (канал A3)
  float loadVolts = v_a3 * LOAD_DIVIDER_RATIO;
  if (loadVolts < 0.05) loadVolts = 0.0;

  // --- ОТРИСОВКА НА ДВУХЦВЕТНОМ ДИСПЛЕЕ ---
  display.clearDisplay();

  // [ЖЕЛТАЯ ЗОНА: Y от 0 до 15] — Статус АКБ прибора
  display.setTextSize(1);
  display.setCursor(0, 3);
  display.print(F("AKB:"));
  display.print(batteryVolts, 2);
  display.print(F("V ("));
  display.print(batteryPct);
  display.print(F("%)"));

  // Иконка батарейки в правом верхнем углу
  display.drawRect(108, 2, 18, 10, SSD1306_WHITE);
  display.fillRect(126, 4, 2, 6, SSD1306_WHITE);
  int barWidth = map(batteryPct, 0, 100, 0, 14);
  display.fillRect(110, 4, barWidth, 6, SSD1306_WHITE);

  // Разделительная линия на границе цветов
  display.drawFastHLine(0, 15, 128, SSD1306_WHITE);

  // [СИНЯЯ ЗОНА: Y от 16 до 63] — Ток и Напряжение силовой линии
  // Вывод ТОКА (крупно, размер 2)
  display.setTextSize(2);
  display.setCursor(0, 21);
  display.print(F("I:"));
  if (currentAmps < 10.0 && currentAmps >= 0.0) display.print(F(" ")); // Выравнивание разрядов
  display.print(currentAmps, 1); // 1 знак после запятой (для сотых замените на 2)
  display.print(F(" A"));

  // Вывод НАПРЯЖЕНИЯ силовой линии (крупно, размер 2)
  display.setCursor(0, 45);
  display.print(F("U:"));
  if (loadVolts < 10.0) display.print(F(" ")); // Выравнивание разрядов
  display.print(loadVolts, 2); // 2 знака после запятой
  display.print(F(" V"));

  display.display();

  // Дублируем отладочную информацию в Serial-порт
  Serial.printf("VCC: %.3fV | VIOUT: %.3fV | I: %.2fA | U_load: %.2fV | Bat: %.2fV\n", 
                v_vcc, v_viout, currentAmps, loadVolts, batteryVolts);

  delay(100);
}