// ============================================================================
// ESTACIÓN METEOROLÓGICA CON SENSORES IoT, LORA Y ALMACENAMIENTO EEPROM
// ============================================================================
// Microcontrolador: ATSAMD21G18A-AU (Arduino Zero compatible)
// Sensores: BH1750 (luz), BMP390 (presión/temp), SHT31 (humedad/temp),
//           Anemómetro, Pluviómetro, Veleta, RFM69HCW (LoRa)
// Fecha: Noviembre 2025
// ============================================================================

#include <Wire.h>              // Comunicación I2C para sensores y EEPROM
#include <Adafruit_BMP3XX.h>   // BMP390: presión barométrica y temperatura
#include <Adafruit_SHT31.h>    // SHT31: temperatura y humedad relativa
#include <BH1750.h>            // BH1750: intensidad luminosa (lux)
#include <RadioLib.h>          // RadioLib: control de módulo RF69 (LoRa)

// ============================================================================
// DEFINICIÓN DE PINES - SENSORES MECÁNICOS Y MÓDULO RF69
// ============================================================================

#define WIND_PIN    7          // Pin digital D7 - Anemómetro (pulsos viento)
                               // Interrupción: FALLING (cada pulso = 2.4 km/h)

#define RAIN_PIN    6          // Pin digital D6 - Pluviómetro (volcados lluvia)
                               // Interrupción: FALLING (cada volcado = 0.2794 mm)

#define WIND_DIR_PIN A1        // Pin analógico A1 - Veleta (dirección)
                               // Sensor potenciométrico 0-1023 (10-bit ADC)

#define LED_PIN     LED_BUILTIN // LED integrado - Indicador de estado

// ---- Pines Módulo RF69 (SPI para radiofrecuencia) ----
#define RF69_CS     8          // Chip Select (SPI)
#define RF69_DIO0   3          // Data Input/Output 0 (interrupción TX/RX)
#define RF69_RST    4          // Reset (reinicio del módulo)

// ============================================================================
// INSTANCIAS DE SENSORES I2C (Bus I2C: SDA/SCL)
// ============================================================================

BH1750 lightMeter;             // Sensor BH1750 (0x23): medidor de luz ambiente
                               // Rango: 1-65535 lux, resolución 1 lux

Adafruit_BMP3XX bmp;           // Sensor BMP390 (0x77): presión y temperatura
                               // Rango: ±1200-2000 hPa, ±0.5°C precisión

Adafruit_SHT31 sht31 = 
    Adafruit_SHT31();          // Sensor SHT31 (0x44): humedad y temperatura
                               // Rango: 0-100% RH, ±0.5°C temperatura

// ---- Módulo LoRa RF69HCW (RadioLib) ----
RF69 radio = new Module(RF69_CS, RF69_DIO0, RF69_RST);
                               // Instancia RF69 con pines de control

volatile bool transmittedFlag = false;
                               // Bandera de interrupción: paquete transmitido

int transmissionState = RADIOLIB_ERR_NONE;
                               // Estado actual de transmisión LoRa

// ============================================================================
// DEFINICIÓN DE CONSTANTES Y PARÁMETROS DE CALIBRACIÓN
// ============================================================================

#define SEALEVELPRESSURE_HPA (1013.25)  // Presión estándar a nivel del mar
                                        // Referencia para cálculo de altitud

#define EEPROM_ADDR 0x50       // Dirección I2C de EEPROM AT24C256
                               // Capacidad: 256 kbits (32 kbytes)

#define GUST_INTERVAL   5000   // Intervalo de cálculo de ráfaga: 5 segundos

#define interval        30000  // Intervalo de medición y transmisión: 30 s
                               // Coincide con recomendaciones WMO estándar

// ============================================================================
// VARIABLES GLOBALES - CONTADORES VOLÁTILES (Manejados por interrupciones)
// ============================================================================

volatile unsigned int rainTicks = 0;
// Contador de volcados del pluviómetro acumulado en cada período de 30 s
// Cada tick representa ~0.2794 mm de precipitación

volatile unsigned int windTicks = 0;
// Contador temporal de pulsos del anemómetro para cálculo de ráfaga máxima
// Reiniciado cada 5 segundos (GUST_INTERVAL)

volatile unsigned int thirtySecWindTicks = 0;
// Contador de pulsos del anemómetro acumulado durante 30 segundos
// Utilizado para calcular velocidad promedio de viento

float highGust = 0.0;
// Almacena velocidad máxima de ráfaga detectada en ventana de 5 minutos
// Unidad: km/h, se reinicia después de transmisión

// ============================================================================
// VARIABLES DE TEMPORIZACIÓN Y CONTROL
// ============================================================================

unsigned long lastPrint = 0;
// Marca de tiempo (ms) de la última medición y transmisión
// Control de intervalo regular de 30 segundos

unsigned long lastGust = 0;
// Marca de tiempo (ms) del último cálculo de ráfaga máxima
// Intervalo: 5 segundos para detección de picos transitorios

int measurementCount = 0;
// Contador de mediciones acumuladas para guardado en EEPROM
// Se reinicia después de guardar 10 registros (300 segundos)

unsigned int eepromAddress = 0;
// Dirección actual de escritura en EEPROM
// Se incrementa con cada registro almacenado (sizeof(DataRecord) bytes)

// ============================================================================
// ESTRUCTURA DE DATOS PARA DIRECCIÓN CARDINAL DEL VIENTO
// ============================================================================

struct WindInfo {
  int grados;           // Ángulo en grados (0-360°) desde norte magnético
  const char* direccion; // Abreviatura cardinal (N, NE, E, SE, S, SO, O, NO)
};
// Total de 16 direcciones posibles para máxima precisión

// ============================================================================
// ESTRUCTURA DE DATOS PARA REGISTRO PERSISTENTE EN EEPROM
// ============================================================================

struct DataRecord {
  unsigned long timestamp;      // Tiempo en segundos desde inicialización
  float lluvia;                 // Precipitación acumulada en mm
  float velocidad;              // Velocidad promedio de viento en km/h (30s)
  float rafaga;                 // Velocidad máxima de ráfaga en km/h (5min)
  int dirGrados;                // Dirección del viento en grados (0-360)
  float potencia;               // Radiación solar estimada en W/m² (de lux)
  float lux;                    // Intensidad luminosa en lux
  float tempBMP;                // Temperatura BMP390 en °C
  float presion;                // Presión barométrica en hPa
  float altitud;                // Altitud calculada en metros
  float tempSHT;                // Temperatura SHT31 en °C
  float humSHT;                 // Humedad relativa SHT31 en %
};
// Tamaño total: ~56 bytes por registro

// ============================================================================
// PROTOTIPOS DE FUNCIONES (Declaración anticipada)
// ============================================================================

void windIRQ();                 // ISR: Interrupción anemómetro (pin D7)
void rainIRQ();                 // ISR: Interrupción pluviómetro (pin D6)
WindInfo getWindDirection();    // Convierte ADC a dirección cardinal (16 puntos)
void eepromWrite(unsigned int eeaddress, byte* data, unsigned int length);
                                // Escritura I2C a EEPROM AT24C256
void startBlink();              // Parpadeo de inicialización (3 blinks)
void formatTime(unsigned long totalSeconds);
                                // Formatea tiempo en formato dd:hh:mm:ss
void eepromBlink();             // Parpadeo de confirmación EEPROM (3 rápidos)
void setFlag();                 // Callback: bandera de transmisión completada

// ============================================================================
// FUNCIÓN SETUP() - INICIALIZACIÓN DEL SISTEMA COMPLETO
// ============================================================================

void setup() {
  // --- Comunicación Serial ---
  Serial.begin(115200);
  Serial.println("🌦 Iniciando estación meteorológica con EEPROM y LoRa...");

  // --- Configuración de Pines ---
  pinMode(WIND_PIN, INPUT_PULLUP);    // Anemómetro: entrada con pullup interno
  pinMode(RAIN_PIN, INPUT_PULLUP);    // Pluviómetro: entrada con pullup interno
  pinMode(WIND_DIR_PIN, INPUT);       // Veleta: entrada analógica sin pullup
  pinMode(LED_PIN, OUTPUT);           // LED: salida digital para indicador

  // --- Configuración de Interrupciones Externas ---
  // Ambos sensores generan pulsos en flanco descendente (FALLING)
  attachInterrupt(digitalPinToInterrupt(WIND_PIN), windIRQ, FALLING);
  attachInterrupt(digitalPinToInterrupt(RAIN_PIN), rainIRQ, FALLING);

  // --- Inicialización Bus I2C ---
  Wire.begin();  // Inicia comunicación I2C (SDA=20, SCL=21 en SAMD21)

  // --- Parpadeo de inicio (indicador de sistema encendido) ---
  startBlink();

  // --- Inicialización BH1750 (Sensor de Luz Ambiente) ---
  if (lightMeter.begin()) {
    Serial.println("✅ BH1750 OK");
  } else {
    Serial.println("❌ Error al iniciar BH1750");
  }
  // Modo: CONTINUOUS_HIGH_RES_MODE (máxima resolución)

  // --- Inicialización BMP390 (Presión y Temperatura) ---
  if (!bmp.begin_I2C()) {
    Serial.println("❌ Error al iniciar BMP390");
  } else {
    Serial.println("✅ BMP390 OK");
    // Configuración de sobremuestreo para mejorar SNR
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    // Filtro digital IIR para suavizar ruido ambiental
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    // Tasa de salida de datos: 50 Hz
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  }

  // --- Inicialización SHT31 (Humedad y Temperatura) ---
  if (!sht31.begin(0x44)) {  // Dirección I2C estándar: 0x44
    Serial.println("❌ Error al iniciar SHT31");
  } else {
    Serial.println("✅ SHT31 OK");
  }

  // --- Inicialización RF69 (Módulo LoRa para transmisión inalámbrica) ---
  Serial.print("[RF69] Inicializando ... ");
  if (radio.begin() != RADIOLIB_ERR_NONE) {
    Serial.println("failed!");
    while (true) {}  // Detener sistema si RF69 falla
  }
  Serial.println("success!");

  // ---- Configuración de RF69 optimizada para México (915 MHz ISM) ----
  radio.setFrequency(915.0);        // Frecuencia central: 915 MHz (banda ISM)
  radio.setOutputPower(20);         // Potencia de salida: 20 dBm (máximo)
  radio.setBitRate(9.6);            // Tasa de bits: 9.6 kbps (range/SNR)
  radio.setRxBandwidth(50.0);       // Ancho de banda RX: 50 kHz
  radio.setFrequencyDeviation(20.0);// Desviación de frecuencia: 20 kHz
  radio.setPreambleLength(48);      // Preámbulo: 48 bits (mejor sincronización)

  // ---- Palabra de sincronización (sync word) para evitar colisiones ----
  uint8_t syncWord[] = {0x2D, 0xD4};
  radio.setSyncWord(syncWord, sizeof(syncWord));

  // ---- Callback: bandera cuando transmisión se completa ----
  radio.setPacketSentAction(setFlag);

  Serial.println("🚀 Estación lista\n");
}

// ============================================================================
// FUNCIÓN LOOP() - CICLO PRINCIPAL DE ADQUISICIÓN Y PROCESAMIENTO
// ============================================================================

void loop() {
  unsigned long currentMillis = millis();

  // ---- SECCIÓN 1: Cálculo de Ráfagas (cada 5 segundos) ----
  if (currentMillis - lastGust >= GUST_INTERVAL) {
    // Convierte pulsos a velocidad: (pulsos/5s) × 2.4 km/h/pulso
    float gust = (float)windTicks / (GUST_INTERVAL / 1000.0) * 2.4;
    
    // Actualiza máxima ráfaga si supera el valor almacenado
    if (gust > highGust) {
      highGust = gust;
    }
    
    // Reinicia contador para próximo ciclo de ráfaga
    windTicks = 0;
    lastGust = currentMillis;
  }

  // ---- SECCIÓN 2: Lectura, Transmisión y Almacenamiento (cada 30 s) ----
  if (currentMillis - lastPrint >= interval) {
    lastPrint = currentMillis;
    measurementCount++;

    // --- Conversión de datos brutos a unidades físicas ---
    float lluvia = rainTicks * 0.2794;  // mm (factor de calibración volcados)
    float velocidadViento = (float)thirtySecWindTicks / (interval / 1000.0) * 2.4;  // km/h
    WindInfo direccion = getWindDirection(); // Consulta dirección actual

    // --- Lectura de sensores I2C ---
    int lux = lightMeter.readLightLevel();
    float tempBMP = NAN, presion = NAN, altitud = NAN;
    float tempSHT = NAN, humSHT = NAN;

    // Conversión: lux a radiación solar (W/m²)
    // Factor aproximado: 1 W/m² ≈ 54.6 lux (según norma)
    int Potencia = lux * 0.0183;

    // --- Lectura BMP390 (Presión y Temperatura) ---
    if (bmp.performReading()) {
      tempBMP = bmp.temperature;
      presion = bmp.pressure / 100.0;  // Conversión Pa a hPa
      altitud = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    }

    // --- Lectura SHT31 (Humedad y Temperatura) ---
    tempSHT = sht31.readTemperature();
    humSHT = sht31.readHumidity();

    // ========== IMPRESIÓN DE DATOS EN PUERTO SERIAL ==========
    Serial.println("---- Lectura de estación ----");

    Serial.print("⏱ Tiempo ON: "); 
    formatTime(currentMillis / 1000);  // Muestra tiempo en dd:hh:mm:ss
    Serial.println();

    Serial.print("🌧 Lluvia: "); 
    Serial.print(lluvia, 2); 
    Serial.println(" mm");

    Serial.print("🌬 Velocidad (Promedio 30s): "); 
    Serial.print(velocidadViento, 2); 
    Serial.println(" km/h");

    Serial.print("💨 Ráfaga Máxima (últimos 5 min): "); 
    Serial.print(highGust, 2); 
    Serial.println(" km/h");

    Serial.print("🧭 Dirección: "); 
    Serial.print(direccion.grados); 
    Serial.print("° ("); 
    Serial.print(direccion.direccion); 
    Serial.println(")");

    Serial.print("🔆 Luz BH1750: "); 
    Serial.print(lux); 
    Serial.print(" lx → Potencia solar: "); 
    Serial.print(Potencia); 
    Serial.println(" W/m²");

    if (!isnan(tempBMP)) {
      Serial.print("🌡 Temperatura BMP390: "); 
      Serial.print(tempBMP); 
      Serial.println(" °C");
      
      Serial.print("📊 Presión barométrica: "); 
      Serial.print(presion); 
      Serial.println(" hPa");
      
      Serial.print("⛰ Altitud calculada: "); 
      Serial.print(altitud); 
      Serial.println(" m");
    }

    if (!isnan(tempSHT) && !isnan(humSHT)) {
      Serial.print("🌡 Temperatura SHT31: "); 
      Serial.print(tempSHT); 
      Serial.println(" °C");
      
      Serial.print("💧 Humedad relativa: "); 
      Serial.print(humSHT); 
      Serial.println(" %");
    }

    Serial.println("-----------------------------");

    // ========== TRANSMISIÓN POR LORA ==========
    // Formato de payload: T:xx,P:xxx,A:xxx,H:xx,L:xxxx,R:xx,W:xx,G:xx,D:xxx
    String payload = "";
    payload += "T:" + String(tempSHT, 1) + ",";     // Temperatura SHT31
    payload += "P:" + String(presion, 1) + ",";     // Presión BMP390
    payload += "A:" + String(altitud, 1) + ",";     // Altitud calculada
    payload += "H:" + String(humSHT, 1) + ",";      // Humedad SHT31
    payload += "L:" + String(lux) + ",";            // Lux BH1750
    payload += "R:" + String(lluvia, 2) + ",";      // Lluvia acumulada
    payload += "W:" + String(velocidadViento, 1) + ",";  // Velocidad viento
    payload += "G:" + String(highGust, 1) + ",";    // Ráfaga máxima
    payload += "D:" + String(direccion.grados);     // Dirección en grados

    // Inicia transmisión asincrónica de paquete LoRa
    transmissionState = radio.startTransmit(payload);

    if (transmissionState == RADIOLIB_ERR_NONE) {
      Serial.println("📡 Transmitiendo por LoRa:");
      Serial.println(payload);
    } 
    else {
      Serial.print("❌ Error transmisión LoRa, código error: ");
      Serial.println(transmissionState);
    }

    // ========== ALMACENAMIENTO EN EEPROM (cada 10 mediciones = 300s) ==========
    // Se guardan registros completos para análisis posterior
    if (measurementCount >= 10) {
      DataRecord record;
      record.timestamp = currentMillis / 1000;
      record.lluvia = lluvia;
      record.velocidad = velocidadViento;
      record.rafaga = highGust;
      record.dirGrados = direccion.grados;
      record.potencia = Potencia;
      record.lux = lux;
      record.tempBMP = tempBMP;
      record.presion = presion;
      record.altitud = altitud;
      record.tempSHT = tempSHT;
      record.humSHT = humSHT;

      // Escribe registro en EEPROM I2C
      eepromWrite(eepromAddress, (byte*)&record, sizeof(record));
      eepromAddress += sizeof(record);  // Incrementa dirección para próximo registro
      
      eepromBlink();  // Indica con LED que almacenamiento fue exitoso
      Serial.println("💾 Registro guardado en EEPROM!");
      
      // Reinicia contadores
      highGust = 0.0;
      measurementCount = 0;
    }

    // ========== REINICIO DE CONTADORES ==========
    rainTicks = 0;              // Reinicia lluvia acumulada
    thirtySecWindTicks = 0;     // Reinicia velocidad promedio
  }

  // ========== Finalización de Transmisión LoRa ==========
  // Se ejecuta cuando la bandera transmittedFlag es activada por ISR
  if (transmittedFlag) {
    transmittedFlag = false;    // Limpia la bandera
    radio.finishTransmit();     // Finaliza transacción LoRa
  }
}

// ============================================================================
// FUNCIONES DE INTERRUPCIÓN - RUTINAS DE SERVICIO (ISR)
// ============================================================================

void windIRQ() {
  // ISR: Anemómetro (pin D7) - Se ejecuta en flanco descendente
  // Cada pulso representa: 1 revolución = 2.4 km/h
  windTicks++;              // Para cálculo de ráfaga instantánea (5s)
  thirtySecWindTicks++;     // Para velocidad promedio (30s)
}

void rainIRQ() {
  // ISR: Pluviómetro (pin D6) - Se ejecuta en flanco descendente
  // Cada pulso representa un volcado de ~0.2794 mm
  rainTicks++;              // Incrementa contador de lluvia
}

// ============================================================================
// FUNCIÓN: CONVERSIÓN DE VOLTAJE ANALÓGICO A DIRECCIÓN CARDINAL
// ============================================================================

WindInfo getWindDirection() {
  // Lee valor analógico de la veleta (rango 0-1023, resolución 10-bit)
  // Cada dirección cardinal presenta combinación única de resistencias
  // en divisor de voltaje de la veleta
  
  int windDir = analogRead(WIND_DIR_PIN);
  
  // Tabla de calibración: Rangos ADC → Dirección Cardinal
  // Comprende 16 puntos cardinales (cada 22.5°)
  // Ordenamiento: comienza en ESE, rotación horaria (N, NE, E, etc.)
  
  if (windDir < 74)   return {113, "ESE"};   // Este-Sureste (113°)
  if (windDir < 88)   return {67,  "ENE"};   // Este-Noreste (67°)
  if (windDir < 110)  return {90,  "E"};     // Este (90°)
  if (windDir < 150)  return {158, "SSE"};   // Sur-Sureste (158°)
  if (windDir < 210)  return {135, "SE"};    // Sureste (135°)
  if (windDir < 260)  return {203, "SSO"};   // Sur-Suroeste (203°)
  if (windDir < 340)  return {180, "S"};     // Sur (180°)
  if (windDir < 430)  return {23,  "NNE"};   // Norte-Noreste (23°)
  if (windDir < 530)  return {45,  "NE"};    // Noreste (45°)
  if (windDir < 615)  return {248, "SO"};    // Suroeste (248°)
  if (windDir < 660)  return {225, "SW"};    // Suroeste (225°)
  if (windDir < 740)  return {338, "NNW"};   // Norte-Noroeste (338°)
  if (windDir < 800)  return {0,   "N"};     // Norte (0°)
  if (windDir < 860)  return {293, "WNW"};   // Oeste-Noroeste (293°)
  if (windDir < 960)  return {270, "W"};     // Oeste (270°)
  if (windDir <= 1023) return {315, "NW"};   // Noroeste (315°)
  
  // Caso de error: lectura fuera de rango
  return {-1, "Desconocida"};
}

// ============================================================================
// FUNCIÓN: ESCRITURA A EEPROM I2C (AT24C256)
// ============================================================================

void eepromWrite(unsigned int eeaddress, byte* data, unsigned int length) {
  
  Wire.beginTransmission(EEPROM_ADDR);
  Wire.write((int)(eeaddress >> 8));    // Byte alto de dirección
  Wire.write((int)(eeaddress & 0xFF));  // Byte bajo de dirección
  for (unsigned int c = 0; c < length; c++) {
    Wire.write(data[c]);                // Escribe byte de datos
  }
  Wire.endTransmission();
  delay(5);  // Tiempo de escritura en EEPROM
}

// ============================================================================
// FUNCIÓN: INDICADOR VISUAL CON LED - PARPADEOS
// ============================================================================

void eepromBlink() {
  // Parpadeo rápido triple: indica almacenamiento exitoso en EEPROM
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
}

void startBlink() {
  // Parpadeo inicial lento triple: indica sistema encendido y listo
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(300);
    digitalWrite(LED_PIN, LOW);
    delay(300);
  }
}

// ============================================================================
// FUNCIÓN: FORMATEADOR DE TIEMPO A FORMATO LEGIBLE
// ============================================================================

void formatTime(unsigned long totalSeconds) {
  // Convierte segundos totales a formato: DD días HH:MM:SS
  // Ejemplo: 90061 segundos = 1d 01:01:01
  
  const unsigned long SECONDS_IN_DAY = 86400;
  const unsigned long SECONDS_IN_HOUR = 3600;
  const unsigned long SECONDS_IN_MINUTE = 60;
  
  unsigned long days = totalSeconds / SECONDS_IN_DAY;
  unsigned long remainingSeconds = totalSeconds % SECONDS_IN_DAY;
  unsigned long hours = remainingSeconds / SECONDS_IN_HOUR;
  remainingSeconds %= SECONDS_IN_HOUR;
  unsigned long minutes = remainingSeconds / SECONDS_IN_MINUTE;
  unsigned long seconds = remainingSeconds % SECONDS_IN_MINUTE;
  
  Serial.print(days); Serial.print("d ");
  if (hours < 10) Serial.print("0");
  Serial.print(hours); Serial.print(":");
  if (minutes < 10) Serial.print("0");
  Serial.print(minutes); Serial.print(":");
  if (seconds < 10) Serial.print("0");
  Serial.print(seconds);
}

// ============================================================================
// FUNCIÓN: CALLBACK DE TRANSMISIÓN COMPLETADA (RF69)
// ============================================================================

void setFlag() {
  // ISR de RadioLib: se ejecuta cuando la transmisión LoRa finaliza
  // Activa bandera transmittedFlag para limpieza en loop() principal
  transmittedFlag = true;
}
