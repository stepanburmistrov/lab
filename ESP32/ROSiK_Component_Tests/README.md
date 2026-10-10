# ROSiK — тестирование компонентов

Набор самостоятельных диагностических скетчей из комплекта `tests.zip`. Каждый тест загружается **отдельно** вместо основной программы робота; одновременная работа всех тестов не предусмотрена.

| Компонент | Скетч | Что проверяет |
| --- | --- | --- |
| Кнопка | [RosikButtonTest](RosikButtonTest/RosikButtonTest.ino) | Нажатие и отпускание, GPIO25, `INPUT_PULLUP` |
| Светодиодное кольцо | [rosikLedTest](rosikLedTest/rosikLedTest.ino) | Радужная анимация WS2812, GPIO14; 16 светодиодов в тесте |
| Датчик чёрной линии | [RosikLineSensorTest](RosikLineSensorTest/RosikLineSensorTest.ino) | Аналоговый сигнал GPIO39 и цифровой GPIO12 |
| Лазерный дальномер VL53L0X | [RosikVL53L0XCalib](RosikVL53L0XCalib/RosikVL53L0XCalib.ino) | Сырые и скорректированные расстояния по I²C (SDA21, SCL22) |
| Лидар — сырые пакеты | [RosikLidarRawSniff](RosikLidarRawSniff/RosikLidarRawSniff.ino) | Байты из Serial2, RX16/TX17 |
| Лидар — секторная маска | [RosikLidarRingMask](RosikLidarRingMask/RosikLidarRingMask.ino) | Восемь секторов и индикация приближения на WS2812 |
| ESP32-CAM | [ESP32CamUniversal_v1_5](ESP32CamUniversal_v1_5/ESP32CamUniversal_v1_5.ino) | Видеопоток MJPEG, настройки Wi-Fi/камеры в NVS, UART-команды |

## Порядок проверки

1. Перед тестом убедитесь, что распиновка соответствует вашей ревизии ROSiK и плате ESP32.
2. Откройте нужный `.ino` в Arduino IDE, выберите плату и порт, установите используемые библиотеки (`FastLED`, `VL53L0X by Pololu` и прочие зависимости выбранного теста).
3. Откройте Serial Monitor на 115200 бод для тестов, использующих Serial.
4. После диагностики загрузите рабочую прошивку обратно.

## Замечания к оригинальным тестам

- **RosikLineSensorTest:** строка `algBlack = raw < THRESHOLD` и `digBlack = (dLevel == LOW)` в исходнике сопровождаются **перевёрнутыми текстовыми метками** в `Serial.printf` (`WHITE` при `true`). Ориентируйтесь на RAW/DOUT, а не на эти подписи. Код сохранён как в исходном архиве.
- **rosikLedTest:** установлено **16 светодиодов**, тогда как в секторной маске лидара используется 8. Сверьте количество со своей конфигурацией.
- **RosikVL53L0XCalib:** пример `60/90` и `200/250` в комментарии не соответствует записанным константам `G=0.969`, `O=-37.21`; для конкретного датчика коэффициенты нужно вычислить заново. Подробнее — [калибровка](../VL53L0X/README.md).
- **ESP32CamUniversal_v1_5:** прошивка предназначена для **AI-Thinker ESP32-CAM**, а не для основной платы ROSiK. Команда `RESETCFG` очищает конфигурацию камеры в её NVS.

[← Все проекты ESP32](../README.md) · [Виртуальная лаборатория ROSiK](https://rosikbot.ru/lab/)
