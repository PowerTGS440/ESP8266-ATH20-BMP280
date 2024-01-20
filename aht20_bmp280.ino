// █████████████████████████████████████████████████████
//
// ESP32/ESP8266 BlackBook by PowerTGS (2024)
//
// author : Tomasz Majcher 
// e-mail : powerTGS440@gmail.com
// phone  : +48 668 082121
//
// █████████████████████████████████████████████████████
//
// ATH10/20 + BMP280 demo file
//
// source file      : sourcce/sensor/ah10/aht10.ino
// device           : esp32/esp2866/arduino uno/arduino nano/arduono mega
// code platform    : ArduinoIDE
// 
// █████████████████████████████████████████████████████
// M A K R O
// █████████████████████████████████████████████████████
//
// PROGMEM to funkcja Arduino AVR, która została przeniesiona do ESP8266, aby zapewnić 
// kompatybilność z istniejącymi bibliotekami Arduino, a także zaoszczędzić pamięć RAM. 
//
// memory used 29920 / 80192 bytes (36%) bez PROGMEM dla Serial.print
// memory used 28760 / 80192 bytes (35%) z PROGMEM dla Serial.print
// memory used 28740 / 80192 bytes (35%) z wyłaczonym SERIAL
//
// już w tak małym szkicu jak ten, różnica zajętej pamięci RAM jest ogromna, ponad 1kB,
// więc warto od samego początku optymalizować kod, niż potem szukać sposobu na wolny
// RAM bo go zabrakło i szkic nie będzie działać. Jak widać wyłączenie monitora portu
// szeregowego to oszczędność tylko 20 bajtów, oznacza to jedno : bardzo dobrze 
// zoptymalizowane polecenia dla wyświetlania komunikatów. Czemu stosuje makra ? 
// bo potem i tak wyłącze SERIAL MONITOR jedną komendą : #define SERIAL false, nie będę
// z niego korzystał bo wartości będę wizualizował na ekranie graficzny, lub urządzenie
// będzie pracowało w trybie odczytywania wartości i przesyłania do innego.
//
// █████████████████████████████████████████████████████
                                    
#define FPSTR(pstr_pointer) (reinterpret_cast<const __FlashStringHelper *>(pstr_pointer))
#define F(string_literal) (FPSTR(PSTR(string_literal)))

#define SERIAL        true                         // SERIAL włączony
#define SERIAL_SPEED  115200                       // prędkość SERIAL
#define LOOP_DELAY    5000                         // opóźnienie pętli LOOP 

// █████████████████████████████████████████████████████
// B I B L I O T E K I
// █████████████████████████████████████████████████████                

#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <AHT10.h>

// █████████████████████████████████████████████████████
// O B I E K T   K L A S Y   B M E 2 8 0   O R A Z   A H T 2 0
// █████████████████████████████████████████████████████

Adafruit_BMP280 bmp;
AHT10 myAHT20(AHT10_ADDRESS_0X38, AHT20_SENSOR);

// █████████████████████████████████████████████████████
// Z M I E N N E   G L O B A L N E
// █████████████████████████████████████████████████████

struct ATH_SENSOR                     // struktura dla czujnika ATH10/ATH20
{
    float   temperatura = 0;  
    float   wilgotnosc = 0;  
}
ath_sensor;

struct BMP_SENSOR                     // struktura dla czujnika BMP280
{
    float temperatura = 0;
    float cisnienie = 0;
}
bmp_sensor;

// █████████████████████████████████████████████████████
// D E K L A R A C J E   F U N K C J I
// █████████████████████████████████████████████████████

void Read_ATH_Sensor (float &temp, float &humi);
void Read_BMP_Sensor (float &temp, float &pres);
void Show_ATH_Sensor (float &temp, float &humi);
void Show_BMP_Sensor (float &temp, float &pres);

// █████████████████████████████████████████████████████
// S E T U P
// █████████████████████████████████████████████████████

void setup() 
{
    #if SERIAL
        Serial.begin(115200);        
    #endif

    // ------------------------------------------------------------- //
    // inicjalizacja sensora AHT10/20
    // ------------------------------------------------------------- //
    
    while (myAHT20.begin() != true) 
    {
        #if SERIAL
            Serial.printf_P( PSTR ("\n\nSensor AHT20 nie jest podłączony") ); 
        #endif
        delay(LOOP_DELAY);
    }
    #if SERIAL
        Serial.printf_P( PSTR ("\nPomyślnie zainicjalizowano czujnik AHT20") );
    #endif

    // ------------------------------------------------------------- //
    // inicjalizacja sensora BMP280
    // ------------------------------------------------------------- //

    if (!bmp.begin()) 
    {     
      #if SERIAL
          Serial.printf_P( PSTR ("Nie można odnaleźć czujnika BMP280") );
          Serial.printf_P( PSTR ("Sprawdź podłączenia do urządzenia!") );
      #endif      
      while (1);
    }
    else
    {
        #if SERIAL
            Serial.printf_P( PSTR ("Pomyślnie zainicjalizowano czujnik BME280") );          
        #endif
    }  
    
    // ------------------------------------------------------------- //
    // ustawienia domyślne dla sensora BMP280
    // ------------------------------------------------------------- //
  
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* tryb pracy */
                    Adafruit_BMP280::SAMPLING_X2,     /* próbkowanie temperatury */
                    Adafruit_BMP280::SAMPLING_X16,    /* próbkowanie ciśnienia */
                    Adafruit_BMP280::FILTER_X16,      /* filtracja */
                    Adafruit_BMP280::STANDBY_MS_500); /* czuwanie */
}

// █████████████████████████████████████████████████████
// L O O P
// █████████████████████████████████████████████████████


void loop() 
{
    #if SERIAL
        Serial.printf_P( PSTR ("\nRozpoczynam pobieranie wartości z czujników : ATH10/20 oraz BMP280\n") );
    #endif
  
    Read_ATH_Sensor(ath_sensor.temperatura, ath_sensor.wilgotnosc);
    Show_ATH_Sensor(ath_sensor.temperatura, ath_sensor.wilgotnosc);
    
    Read_BMP_Sensor(bmp_sensor.temperatura, bmp_sensor.cisnienie);
    Show_BMP_Sensor(bmp_sensor.temperatura, bmp_sensor.cisnienie);
 
    delay(LOOP_DELAY);
}

// █████████████████████████████████████████████████████
// R E A D   A T H 2 0   S E N S O R
// █████████████████████████████████████████████████████
// funkcja odczytuje wartości z czujnika ATH20
// zwraca wartości temperatury i wilgotności
// na wejsciu otrzymuje adresy struktury ath_sensor 
// █████████████████████████████████████████████████████

void Read_ATH_Sensor (float &temp, float &humi)
{
    temp = myAHT20.readTemperature();
    humi = myAHT20.readHumidity();
}

// █████████████████████████████████████████████████████
// R E A D   B M P   S E N S O R
// █████████████████████████████████████████████████████
// funkcja odczytuje wartości z czujnika ATH20
// zwraca wartości temperatury i wilgotności
// na wejsciu otrzymuje adresy struktury ath_sensor 
// █████████████████████████████████████████████████████

void Read_BMP_Sensor (float &temp, float &pres)
{
    temp = bmp.readTemperature();
    pres = bmp.readPressure();
}

// █████████████████████████████████████████████████████
// S H O W   A T H   S E N S O R
// █████████████████████████████████████████████████████
// funkcja odczytuje wartości z czujnika ATH20
// na wejsciu otrzymuje adresy struktury ath_sensor 
// █████████████████████████████████████████████████████

void Show_ATH_Sensor (float &temp, float &humi)
{
    #if SERIAL
        Serial.printf_P( PSTR ("[ATH_Sensor] Temperatura = %.02f *C\n"), temp);
        Serial.printf_P( PSTR ("[ATH Sensor] Wilgotnosc  = %.02f %RH\n"), humi);
    #endif
}

// █████████████████████████████████████████████████████
// S H O W   B M P   S E N S O R
// █████████████████████████████████████████████████████
// funkcja odczytuje wartości z czujnika BMP280
// na wejsciu otrzymuje adresy struktury ath_sensor 
// █████████████████████████████████████████████████████

void Show_BMP_Sensor (float &temp, float &pres)
{
    #if SERIAL
        Serial.printf_P( PSTR ("[BMP_Sensor] Temperatura = %.02f *C\n"), temp);
        Serial.printf_P( PSTR ("[BMP Sensor] Cisnienie  = %.02f hPa\n"), pres / 100 );
    #endif  
}
// █████████████████████████████████████████████████████
// END OF FILE : src/sensor/ath10/ath10.ino
// █████████████████████████████████████████████████████
