🌦️ STM32 Weather Station with BME280 and OLED SSD1306
Projekt przedstawia prostą stację pogodową zbudowaną na mikrokontrolerze STM32 (NUCLEO-F411RE), która:

odczytuje temperaturę, wilgotność i ciśnienie z czujnika BME280 przez I2C,

wyświetla dane na ekranie OLED SSD1306 (SPI),

obsługuje przycisk sprzętowy do zmiany trybu wyświetlania (inwersja),

przesyła dane przez UART do terminala,

działa w oparciu o system operacyjny FreeRTOS z podziałem na niezależne zadania.

📦 Zawartość projektu
main.c – główny plik programu, inicjalizacja peryferiów, konfiguracja FreeRTOS i sensorów.

ssd1306.c/h – biblioteka do obsługi wyświetlacza OLED.

bme280.c/h – biblioteka firmy Bosch do obsługi czujnika BME280.

FreeRTOS – system RTOS do zarządzania wielozadaniowością.

🔧 Wymagania sprzętowe
Płytka STM32 – np. NUCLEO-F411RE

Czujnik BME280 – podłączony przez I2C1

Wyświetlacz OLED SSD1306 – podłączony przez SPI

Przycisk – do zmiany trybu wyświetlacza (np. USER Button na NUCLEO)

UART (USART2) – do debugowania przez terminal (np. PuTTY)

⚙️ Schemat połączeń
Urządzenie	STM32 Pin	Interfejs
BME280 SDA	PB9 (I2C1 SDA)	I2C
BME280 SCL	PB8 (I2C1 SCL)	I2C
SSD1306 MOSI	PB5	SPI
SSD1306 SCK	PB3	SPI
SSD1306 CS	PB6	GPIO
SSD1306 DC	PB4	GPIO
SSD1306 RES	PB10	GPIO
Button	PC13	GPIO (EXTI)

🧠 Struktura FreeRTOS
Projekt dzieli się na cztery zadania:

Zadanie	Priorytet	Funkcja
defaultTask	Normal	Zadanie domyślne – może być użyte jako tło
BME280_Task	Normal	Odczyt danych z czujnika BME280
Display_Task	BelowNormal	Aktualizacja ekranu OLED
UART_Task	Low	Wysyłanie danych przez UART

🖥️ Przykład danych UART

System initialized
Chip ID: 0x60
Temperature: 23.45 °C
Humidity: 45.23 %
Pressure: 1013.22 hPa
📷 Ekran OLED
Wyświetlacz pokazuje dane pogodowe w formacie:


Temp: 23.4 C
Wilg: 45.2 %
Cisn: 1013.2 hPa
Przycisk (PC13) przełącza tryb wyświetlania (negatyw/pozytyw).

✅ Jak uruchomić projekt
Skonfiguruj sprzęt i połączenia według tabeli powyżej.

Załaduj projekt do STM32CubeIDE.

Zbuduj i wgraj firmware na płytkę.

Otwórz terminal UART (np. PuTTY, 115200 baud).

Obserwuj dane pogodowe na ekranie OLED i w terminalu UART.

📚 Źródła i biblioteki
Bosch Sensortec BME280 API

SSD1306 HAL STM32 library

FreeRTOS (wbudowany w STM32CubeMX)
