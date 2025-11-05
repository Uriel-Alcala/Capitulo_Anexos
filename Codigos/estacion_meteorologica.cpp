// ============================================================================
// ESTACIÓN METEOROLÓGICA MULTIPROPÓSITO CON SENSORES IoT
// ============================================================================
// Microcontrolador: ATSAMD21G18A-AU
// Fecha: Noviembre 2025
// ============================================================================

#include <Wire.h>              // Comunicación I2C para sensores digitales
#include <Adafruit_BMP3XX.h>   // BMP390: presión barométrica y temperatura
#include <Adafruit_SHT31.h>    // SHT31: temperatura y humedad relativa
#include <BH1750.h>            // BH1750: intensidad luminosa (lux)

// ============================================================================
// DEFINICIÓN DE PINES DE ENTRADA (SENSORES ANALÓGICOS Y DIGITALES)
// ============================================================================

#define WIND_PIN    7          // Pin digital D7 - Anemómetro (velocidad viento)
                               // Interrupción externa FALLING en cada pulso
                               // Calibración: 1 pulso = 2.4 km/h

#define RAIN_PIN    6          // Pin digital D6 - Pluviómetro (contador lluvia)
                               // Interrupción externa FALLING en cada volcado
                               // Calibración: 1 volcado = 0.2794 mm

#define WIND_DIR_PIN A1        // Pin analógico A1 - Veleta (dirección viento)
                               // Sensor potenciométrico 0-1023 (resolución 10-bit)

// ============================================================================
// INSTANCIAS DE SENSORES I2C (Dirección de bus I2C SDA/SCL)
// ============================================================================

BH1750 lightMeter;             // Sensor luminoso BH1750 (0x23 - dirección I2C)
                               // Modo: CONTINUOUS_HIGH_RES_MODE (1 lx resolución)

Adafruit_BMP3XX bmp;           // Sensor BMP390 (0x77 - dirección I2C por defecto)
                               // Mide: temperatura, presión barométrica, altitud

Adafruit_SHT31 sht31 = 
    Adafruit_SHT31();          // Sensor SHT31 (0x44 - dirección I2C)
                               // Mide: temperatura y humedad relativa (%RH)

#define SEALEVELPRESSURE_HPA (1013.25)  // Presión estándar a nivel del mar
                                        // Referencia para cálculo de altitud

// ============================================================================
// ESTRUCTURA DE DATOS: INFORMACIÓN DE DIRECCIÓN DEL VIENTO
// ============================================================================

struct WindInfo {
  int grados;           // Ángulo en grados (0-360°) desde norte magnético
  const char* direccion; // Abreviatura cardinal (N, NE, E, SE, S, SO, O, NO)
};
// Nota: Se utilizan 16 direcciones para máxima precisión direccional

// ============================================================================
// VARIABLES GLOBALES - CONTADORES VOLÁTILES (Interrupciones)
// ============================================================================

volatile unsigned int rainTicks = 0;
// Acumula volcados del pluviómetro en cada ciclo de 10 segundos
// Incrementado por interrupción rainIRQ en cada caída de lluvia

volatile unsigned int windTicks = 0;
// Contador temporal para cálculo de ráfagas máximas en 10 segundos
// Reiniciado cada ciclo de medición de ráfagas

volatile unsigned int halfMinuteWindTicks = 0;
// Contador de pulsos del anemómetro durante 30 segundos
// Utilizado para calcular velocidad promedio de viento

float highGust = 0.0;
// Almacena la velocidad máxima de ráfaga detectada en el período actual
// Reiniciado a 0.0 después de cada reporte

// ============================================================================
// VARIABLES DE TEMPORIZACIÓN (Control de ciclos de medición)
// ============================================================================

unsigned long lastPrint = 0;
// Marca de tiempo (ms) de la última impresión de datos en serie
// Utilizada para mantener intervalo regular de 10 segundos

unsigned long lastGust = 0;
// Marca de tiempo (ms) del último cálculo de ráfaga máxima
// Intervalo: 10 segundos para detección de picos de viento

const unsigned long interval = 10000;   // Intervalo de reporte = 10 segundos
                                        // Coincide con intervalo WMO recomendado

// ============================================================================
// PROTOTIPOS DE FUNCIONES (Declaración anticipada)
// ============================================================================

void windIRQ();              // ISR: Interrupción del anemómetro (pin D7)
void rainIRQ();              // ISR: Interrupción del pluviómetro (pin D6)
WindInfo getWindDirection(); // Convierte valor ADC a dirección cardinal

// ============================================================================
// FUNCIÓN SETUP() - INICIALIZACIÓN DEL SISTEMA
// ============================================================================

void setup() {
  // --- Comunicación Serial ---
  Serial.begin(115200);
  while (!Serial);  // Espera conexión USB (necesario en SAMD21)
  Serial.println("🌦 Iniciando estación meteorológica...");

  // --- Configuración de Pines ---
  pinMode(WIND_PIN, INPUT_PULLUP);    // Anemómetro: entrada con pullup
  pinMode(RAIN_PIN, INPUT_PULLUP);    // Pluviómetro: entrada con pullup
  pinMode(WIND_DIR_PIN, INPUT);       // Veleta: entrada analógica

  // --- Interrupciones Externas ---
  // Configuran ISR en flanco descendente (FALLING) para máxima confiabilidad
  attachInterrupt(digitalPinToInterrupt(WIND_PIN), windIRQ, FALLING);
  attachInterrupt(digitalPinToInterrupt(RAIN_PIN), rainIRQ, FALLING);

  // --- Inicialización Bus I2C ---
  Wire.begin();  // Inicia comunicación I2C (SDA=20, SCL=21 en SAMD21)

  // --- Inicialización BH1750 (Sensor de Luz) ---
  if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
    Serial.println("✅ BH1750 OK");
  } else {
    Serial.println("❌ Error al iniciar BH1750");
  }
  // Configuración: Modo continuo de alta resolución (1 lx resolución)

  // --- Inicialización BMP390 (Presión y Temperatura) ---
  if (!bmp.begin_I2C()) {
    Serial.println("❌ Error al iniciar BMP390");
  } else {
    Serial.println("✅ BMP390 OK");
    // Configuración de sobremuestreo para mejorar relación señal-ruido
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    // Filtro IIR para suavizar lecturas
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    // Tasa de salida: 50 Hz
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  }

  // --- Inicialización SHT31 (Humedad y Temperatura) ---
  if (!sht31.begin(0x44)) {  // Dirección I2C por defecto: 0x44
    Serial.println("❌ Error al iniciar SHT31");
  } else {
    Serial.println("✅ SHT31 OK");
  }

  Serial.println("🚀 Estación lista\n");
}

// ============================================================================
// FUNCIÓN LOOP() - CICLO PRINCIPAL DE ADQUISICIÓN Y PROCESAMIENTO
// ============================================================================

void loop() {
  unsigned long currentMillis = millis();

  // ---- SECCIÓN 1: Cálculo de Ráfagas (cada 10 segundos) ----
  if (currentMillis - lastGust >= 10000) {
    // Convierte pulsos a velocidad: (pulsos/10s) × 2.4 km/h/pulso
    float gust = (float)windTicks / 10.0 * 2.4;
    
    // Actualiza máxima ráfaga si supera el valor anterior
    if (gust > highGust) highGust = gust;
    
    // Reinicia contador de pulsos para próximo ciclo
    windTicks = 0;
    lastGust = currentMillis;
  }

  // ---- SECCIÓN 2: Lectura y Reporte de Sensores (cada 10 segundos) ----
  if (currentMillis - lastPrint >= interval) {
    lastPrint = currentMillis;

    // Conversión de datos brutos a unidades físicas
    float lluvia = rainTicks * 0.2794;      // mm (factor de calibración)
    float velocidadViento = (float)halfMinuteWindTicks / 30.0 * 2.4;  // km/h
    WindInfo direccion = getWindDirection(); // Consulta dirección actual

    // --- ENCABEZADO DE REPORTE ---
    Serial.println("---- Lectura de estación ----");

    // --- SENSORES MECÁNICOS (Entrada digital) ---
    Serial.print("⏱ Tiempo: ");
    Serial.print(currentMillis / 1000);
    Serial.println(" s");

    Serial.print("🌧 Lluvia: ");
    Serial.print(lluvia, 2);
    Serial.println(" mm");

    Serial.print("🌬 Velocidad: ");
    Serial.print(velocidadViento, 2);
    Serial.println(" km/h");

    Serial.print("💨 Ráfaga: ");
    Serial.print(highGust, 2);
    Serial.println(" km/h");

    Serial.print("🧭 Dirección: ");
    Serial.print(direccion.grados);
    Serial.print("° (");
    Serial.print(direccion.direccion);
    Serial.println(")");

    // --- SENSORES I2C (Luz) ---
    float lux = lightMeter.readLightLevel();
    Serial.print("🔆 Luz: ");
    Serial.print(lux);
    Serial.println(" lx");

    // --- SENSORES I2C (Presión y Temperatura) ---
    if (bmp.performReading()) {
      Serial.print("🌡 Temp BMP390: ");
      Serial.print(bmp.temperature);
      Serial.println(" °C");

      Serial.print("📊 Presión: ");
      Serial.print(bmp.pressure / 100.0);  // Conversión a hPa
      Serial.println(" hPa");

      Serial.print("⛰ Altitud: ");
      Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
      Serial.println(" m");
    } else {
      Serial.println("❌ Fallo al leer BMP390");
    }

    // --- SENSORES I2C (Humedad y Temperatura) ---
    float t = sht31.readTemperature();
    float h = sht31.readHumidity();
    if (!isnan(t) && !isnan(h)) {  // Valida que lecturas sean números válidos
      Serial.print("🌡 Temp SHT31: ");
      Serial.print(t);
      Serial.println(" °C");

      Serial.print("💧 Humedad: ");
      Serial.print(h);
      Serial.println(" %");
    } else {
      Serial.println("❌ Fallo al leer SHT31");
    }

    Serial.println("-----------------------------\n");

    // --- REINICIO DE CONTADORES ---
    rainTicks = 0;              // Reinicia lluvia acumulada
    halfMinuteWindTicks = 0;    // Reinicia velocidad promedio
    highGust = 0.0;             // Reinicia ráfaga máxima
  }
}

// ============================================================================
// FUNCIONES DE INTERRUPCIÓN - RUTINAS DE SERVICIO (ISR)
// ============================================================================

void windIRQ() {
  // Rutina de Interrupción Externa - Anemómetro (Pin D7)
  // Se ejecuta en flanco descendente de cada pulso del anemómetro
  // Incrementa dos contadores: uno para ráfaga instantánea, otro para promedio
  windTicks++;              // Contador para ráfaga máxima (cada 10s)
  halfMinuteWindTicks++;    // Contador para velocidad promedio (30s)
}

void rainIRQ() {
  // Rutina de Interrupción Externa - Pluviómetro (Pin D6)
  // Se ejecuta en flanco descendente de cada volcado del pluviómetro
  // Cada pulso representa un volcado de ~0.2794 mm de lluvia
  rainTicks++;              // Incrementa acumulador de lluvia
}

// ============================================================================
// FUNCIÓN: CONVERSIÓN DE VOLTAJE A DIRECCIÓN CARDINAL
// ============================================================================

WindInfo getWindDirection() {
  // Lee valor analógico de la veleta (rango 0-1023)
  // Utiliza divisor de voltaje resistivo: cada dirección cardinal
  // presenta una combinación única de resistencias
  
  int windDir = analogRead(WIND_DIR_PIN);
  
  // Tabla de calibración: Rangos de valores ADC vs Dirección Cardinal
  // Se ordenan por dirección de inicio (ESE) en rotación horaria
  // Formato: if (windDir < valor_máximo) return {ángulo_grados, "DIRECCIÓN"};
  
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
  
  // Caso por defecto: error de lectura
  return {-1, "Desconocida"};
}

