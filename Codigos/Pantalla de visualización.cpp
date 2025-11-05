// ============================================================================
// RECEPTOR LORA CON PANTALLA E-PAPER Y INTEGRACIÓN THINGSPEAK
// ============================================================================
// Microcontrolador: ESP32 (con WiFi incorporado)
// Display: GxEPD2_750c (7.5" e-paper tricolor - Negro/Blanco/Rojo)
// Sensores remotos: BH1750, BMP390, SHT31, Anemómetro, Pluviómetro, Veleta
// Módulo LoRa: RF69HCW (915 MHz ISM México)
// Fecha: Noviembre 2025
// ============================================================================

#include <RadioLib.h>           // Control de módulo RF69 (LoRa)
#include <GxEPD2_3C.h>          // Driver para pantalla e-paper tricolor (7.5")
#include <Fonts/FreeMonoBold9pt7b.h>  // Fuente monoespaciada bold
#include <Fonts/FreeMono9pt7b.h>      // Fuente monoespaciada regular
#include <Wire.h>               // Comunicación I2C (EEPROM y display)
#include <Imagen.h>             // Bitmaps de iconos: Temperatura45, Humedad45, etc.

#include <WiFi.h>               // Conectividad WiFi (ESP32)
#include <ThingSpeak.h>         // API para envío de datos a ThingSpeak

// ============================================================================
// CONFIGURACIÓN DE MEMORIA EEPROM I2C (AT24C256 o similar)
// ============================================================================

#define EEPROM_ADDR 0x50        // Dirección I2C de EEPROM
#define EEPROM_SIZE 131072      // Capacidad: 128 kbytes (256 kbits)

unsigned int eepromAddress = 0; // Puntero de escritura actual en EEPROM

// ============================================================================
// ESTRUCTURA DE DATOS PARA REGISTROS PERSISTENTES
// ============================================================================

struct DataRecord {
  unsigned long timestamp;      // Tiempo de recepción en segundos
  float lluvia;                 // Precipitación en mm
  float velocidad;              // Velocidad promedio viento en km/h
  float rafaga;                 // Velocidad máxima ráfaga en km/h
  int dirGrados;                // Dirección viento en grados (0-360)
  float lux;                    // Intensidad luminosa en lux
  float irradiancia;            // Radiación solar en W/m² (convertida de lux)
  float tempBMP;                // Temperatura BMP390 en °C
  float presion;                // Presión barométrica en hPa
  float altitud;                // Altitud calculada en metros
  float tempSHT;                // Temperatura SHT31 en °C
  float humSHT;                 // Humedad relativa SHT31 en %
};
// Tamaño total: ~56 bytes por registro

DataRecord lastReceivedRecord;  // Último registro recibido por LoRa
unsigned long lastEEPROMWrite = 0;  // Marca de tiempo última escritura EEPROM
int measurementCount = 0;       // Contador para almacenamiento cada 10 paquetes
unsigned long paquetesRecibidos = 0;  // Contador total de paquetes LoRa recibidos

// ============================================================================
// CONFIGURACIÓN WIFI Y THINGSPEAK (Ingrese sus credenciales)
// ============================================================================

const char* ssid = "Casa";      // Nombre de red WiFi
const char* password = "1234Alcala.";  // Contraseña WiFi

unsigned long channelID = 3115916;     // ID de canal ThingSpeak
const char* WriteAPIKey = "V4NWRL737P9R9JXD";  // API Key para escritura

WiFiClient client;              // Cliente WiFi para ThingSpeak

// ============================================================================
// ESTRUCTURA PARA INFORMACIÓN CARDINAL DEL VIENTO
// ============================================================================

struct WindInfo {
  int grados;           // Ángulo en grados (0-360)
  const char* direccion; // Abreviatura: N, NE, E, SE, S, SO, O, NO
};

// ============================================================================
// FUNCIÓN: CONVERSIÓN DE GRADOS A DIRECCIÓN CARDINAL (16 PUNTOS)
// ============================================================================

WindInfo getWindDirection(int grados) {
  // Convierte ángulo (0-360°) a dirección cardinal con 16 puntos
  // Rango: ±11° alrededor de cada dirección cardinal
  
  if (grados < 0) return {-1, "Des"};  // Dirección desconocida
  
  // Rangos de 22.5° para cada dirección (360/16 = 22.5)
  if (grados >= 348 || grados < 11)   return {0,   "N"};      // Norte
  if (grados >= 11 && grados < 34)    return {23,  "NNE"};    // N-Noreste
  if (grados >= 34 && grados < 56)    return {45,  "NE"};     // Noreste
  if (grados >= 56 && grados < 78)    return {67,  "ENE"};    // E-Noreste
  if (grados >= 78 && grados < 101)   return {90,  "E"};      // Este
  if (grados >= 101 && grados < 123)  return {113, "ESE"};    // E-Sureste
  if (grados >= 123 && grados < 146)  return {135, "SE"};     // Sureste
  if (grados >= 146 && grados < 168)  return {158, "SSE"};    // S-Sureste
  if (grados >= 168 && grados < 191)  return {180, "S"};      // Sur
  if (grados >= 191 && grados < 213)  return {203, "SSO"};    // S-Suroeste
  if (grados >= 213 && grados < 236)  return {225, "SO"};     // Suroeste
  if (grados >= 236 && grados < 258)  return {248, "OSO"};    // O-Suroeste
  if (grados >= 258 && grados < 281)  return {270, "O"};      // Oeste
  if (grados >= 281 && grados < 303)  return {293, "ONO"};    // O-Noroeste
  if (grados >= 303 && grados < 326)  return {315, "NO"};     // Noroeste
  if (grados >= 326 && grados < 348)  return {338, "NNO"};    // N-Noroeste
  
  return {-1, "?"};  // Error
}

// ============================================================================
// CONFIGURACIÓN DE MÓDULO RF69 (PINES Y COMUNICACIÓN LORA)
// ============================================================================

#define RF69_CS   2     // Chip Select (SPI)
#define RF69_DIO0 33    // Data Input/Output 0 (interrupción)
#define RF69_RST  15    // Reset del módulo

RF69 radio = new Module(RF69_CS, RF69_DIO0, RF69_RST);
                        // Instancia RF69 con pines de control

volatile bool receivedFlag = false;
                        // Bandera de interrupción: paquete recibido

void setFlag(void) { 
  receivedFlag = true;  // ISR: activa cuando paquete LoRa es recibido
}

// ============================================================================
// CONFIGURACIÓN DE PANTALLA E-PAPER TRICOLOR (GxEPD2_750c)
// ============================================================================

#define CS_PIN   5      // Chip Select (SPI)
#define DC_PIN   17     // Data/Command (SPI)
#define RST_PIN  16     // Reset de display
#define BUSY_PIN 4      // Señal BUSY (display ocupado)

// Display: 800x480 píxeles, 7.5", colores: Negro/Blanco/Rojo
GxEPD2_3C<GxEPD2_750c, GxEPD2_750c::HEIGHT> display(
    GxEPD2_750c(CS_PIN, DC_PIN, RST_PIN, BUSY_PIN)
);

// ============================================================================
// FUNCIÓN: PARSEADOR DE CAMPOS CSV EN PAYLOAD LORA
// ============================================================================

float parseField(const String& s, const char* tag) {
  // Extrae valor numérico de formato CSV: "T:23.5,P:1013.2,..."
  // Parámetros:
  //   s: cadena de payload LoRa
  //   tag: etiqueta a buscar (ej: "T:", "P:", "H:")
  // Retorna: valor flotante o NAN si no existe
  
  int idx = s.indexOf(tag);
  if (idx == -1) return NAN;  // Tag no encontrado
  
  idx += strlen(tag);  // Posición del valor después del tag
  int nextComma = s.indexOf(',', idx);
  
  // Extrae substring entre tag y siguiente coma
  String val = (nextComma == -1) ? s.substring(idx) : s.substring(idx, nextComma);
  return val.toFloat();  // Convierte a float
}

// ============================================================================
// FUNCIÓN: ESCRITURA A EEPROM I2C (AT24C256)
// ============================================================================

void eepromWrite(unsigned int eeaddress, byte* data, unsigned int length) {
  // Protocolo I2C para escribir en EEPROM:
  // 1. Inicia transmisión a dirección EEPROM (0x50)
  // 2. Envía byte alto y bajo de dirección de escritura
  // 3. Transmite datos byte a byte
  // 4. Termina y espera ciclo de escritura (~5 ms)
  
  Wire.beginTransmission(EEPROM_ADDR);
  Wire.write((int)(eeaddress >> 8));    // Byte alto de dirección
  Wire.write((int)(eeaddress & 0xFF));  // Byte bajo de dirección
  for (unsigned int c = 0; c < length; c++) {
    Wire.write(data[c]);  // Escribe cada byte de datos
  }
  Wire.endTransmission();
  delay(5);  // Espera a que EEPROM complete escritura
}

// ============================================================================
// FUNCIÓN: DIBUJA ROSA DE LOS VIENTOS (BITMAP 128x128)
// ============================================================================

void drawCompassBitmap(int centerX, int centerY) {
  // Dibuja bitmap de rosa de los vientos centrada en (centerX, centerY)
  // Bitmap: 128x128 píxeles, archivo: Rosa128 en Imagen.h
  
  display.drawXBitmap(centerX - 64, centerY - 64, Rosa128, 128, 128, GxEPD_BLACK);
  display.fillCircle(centerX, centerY, 3, GxEPD_BLACK);  // Punto central
}

// ============================================================================
// FUNCIÓN: DIBUJA DIRECCIÓN ACTUAL EN ROSA DE LOS VIENTOS
// ============================================================================

void drawCompassDirection(String dir, int centroX, int centroY) {
  // Dibuja abreviatura de dirección (N, NE, E, etc) alrededor de la rosa
  // Posicionado según su ubicación cardinal
  
  display.setFont(&FreeMonoBold9pt7b);
  display.setTextColor(GxEPD_BLACK);
  int radio = 80;  // Radio de posicionamiento del texto

  if (dir == "N") {
    display.setCursor(centroX - 5, centroY - radio + 10);
    display.print("N");
  } 
  else if (dir == "NE") {
    display.setCursor(centroX + radio * 0.55, centroY - radio * 0.55);
    display.print("NE");
  } 
  else if (dir == "E") {
    display.setCursor(centroX + radio - 7, centroY + 4);
    display.print("E");
  } 
  else if (dir == "SE") {
    display.setCursor(centroX + radio * 0.55, centroY + radio * 0.55 + 5);
    display.print("SE");
  } 
  else if (dir == "S") {
    display.setCursor(centroX - 5, centroY + radio + 0);
    display.print("S");
  } 
  else if (dir == "SO") {
    display.setCursor(centroX - radio * 0.55 - 15, centroY + radio * 0.55 + 5);
    display.print("SO");
  } 
  else if (dir == "O") {
    display.setCursor(centroX - radio - 7, centroY + 5);
    display.print("O");
  } 
  else if (dir == "NO") {
    display.setCursor(centroX - radio * 0.55 - 15, centroY - radio * 0.55);
    display.print("NO");
  }
}

// ============================================================================
// FUNCIÓN SETUP() - INICIALIZACIÓN DEL SISTEMA
// ============================================================================

void setup() {
  // --- Comunicación Serial ---
  Serial.begin(9600);
  Wire.begin();  // Inicia bus I2C (SDA=21, SCL=22 en ESP32)

  // --- Inicialización RF69 (Módulo LoRa) ---
  Serial.println(F("[RF69] Inicializando..."));
  if (radio.begin() != RADIOLIB_ERR_NONE) {
    Serial.println(F("Error al inicializar RF69!"));
    while (true) {}  // Detener si falla RF69
  }

  // --- Configuración de RF69 optimizada para México (915 MHz) ---
  radio.setFrequency(915.0);        // Frecuencia central: 915 MHz
  radio.setOutputPower(20);         // Potencia TX: 20 dBm (máximo)
  radio.setBitRate(9.6);            // Tasa de bits: 9.6 kbps
  radio.setRxBandwidth(50.0);       // Ancho de banda RX: 50 kHz
  radio.setFrequencyDeviation(20.0); // Desviación: 20 kHz
  radio.setPreambleLength(48);      // Preámbulo: 48 bits

  // --- Palabra de sincronización (sync word) ---
  uint8_t syncWord[] = {0x2D, 0xD4};
  radio.setSyncWord(syncWord, sizeof(syncWord));

  // --- Callback para recepción de paquetes ---
  radio.setPacketReceivedAction(setFlag);
  radio.startReceive();  // Inicia modo recepción continua

  // --- Inicialización Display E-Paper ---
  display.init(115200, true, 2, false);  // Inicializa SPI a 115200 baud
  display.setRotation(0);    // Rotación: 0 (orientación normal)
  display.setFullWindow();   // Usa ventana completa del display

  // --- Conexión WiFi para ThingSpeak ---
  WiFi.begin(ssid, password);
  Serial.print("Conectando a WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Conectado a WiFi");
  Serial.print("Dirección IP: ");
  Serial.println(WiFi.localIP());

  // --- Inicialización ThingSpeak ---
  ThingSpeak.begin(client);
}

// ============================================================================
// FUNCIÓN LOOP() - CICLO PRINCIPAL DE RECEPCIÓN Y VISUALIZACIÓN
// ============================================================================

void loop() {
  // Verifica si se recibió paquete LoRa
  if (receivedFlag) {
    receivedFlag = false;
    String str;
    
    // --- Lectura de paquete LoRa ---
    int state = radio.readData(str);

    if (state == RADIOLIB_ERR_NONE) {
      paquetesRecibidos++;  // Incrementa contador total

      // === PARSEO DE CAMPOS DEL PAYLOAD CSV ===
      // Formato: T:xx,P:xxx,A:xxx,H:xx,L:xxxx,R:xx,W:xx,G:xx,D:xxx
      
      float tempSHT = parseField(str, "T:");      // Temperatura SHT31
      float presion = parseField(str, "P:");      // Presión BMP390
      float altitud = parseField(str, "A:");      // Altitud calculada
      float humSHT  = parseField(str, "H:");      // Humedad SHT31
      float lux     = parseField(str, "L:");      // Lux BH1750
      float irradiancia = lux * 0.0183;           // Conversión lux → W/m²
      float lluvia  = parseField(str, "R:");      // Lluvia acumulada
      float viento  = parseField(str, "W:");      // Velocidad viento
      float rafaga  = parseField(str, "G:");      // Ráfaga máxima
      int dirGradosRaw = (int)parseField(str, "D:");  // Dirección grados

      // --- Conversión de grados a dirección cardinal ---
      WindInfo dirViento = getWindDirection(dirGradosRaw);

      // === ESTRUCTURACIÓN DE REGISTRO PARA EEPROM ===
      DataRecord record = {
        millis() / 1000,        // timestamp (segundos)
        lluvia,                 // lluvia (mm)
        viento,                 // velocidad (km/h)
        rafaga,                 // rafaga (km/h)
        dirViento.grados,       // dirección (grados)
        lux,                    // lux
        irradiancia,            // radiación solar (W/m²)
        NAN,                    // tempBMP (no disponible en receptor)
        presion,                // presión (hPa)
        altitud,                // altitud (m)
        tempSHT,                // temperatura (°C)
        humSHT                  // humedad (%)
      };

      lastReceivedRecord = record;  // Almacena último registro

      // === ESCRITURA EN EEPROM CADA 10 PAQUETES ===
      measurementCount++;
      if (measurementCount >= 10) {
        eepromWrite(eepromAddress, (byte*)&record, sizeof(record));
        eepromAddress += sizeof(record);
        measurementCount = 0;
      }

      // === ENVÍO A THINGSPEAK ===
      // Campos: 1=Temp, 2=Humedad, 3=Radiación, 4=Altitud, 5=Presión,
      //         6=Viento, 7=Dirección, 8=Lluvia
      
      ThingSpeak.setField(1, (int)round(tempSHT));          // Campo 1
      ThingSpeak.setField(2, (int)round(humSHT));           // Campo 2
      ThingSpeak.setField(3, (int)round(irradiancia));      // Campo 3 (W/m²)
      ThingSpeak.setField(4, (int)round(altitud));          // Campo 4
      ThingSpeak.setField(5, (int)round(presion));          // Campo 5
      ThingSpeak.setField(6, viento);                       // Campo 6
      ThingSpeak.setField(7, dirGradosRaw);                 // Campo 7
      ThingSpeak.setField(8, lluvia);                       // Campo 8
      
      ThingSpeak.writeFields(channelID, WriteAPIKey);

      // === DIBUJO EN PANTALLA E-PAPER ===
      display.firstPage();
      do {
        display.fillScreen(GxEPD_WHITE);  // Limpia pantalla (fondo blanco)
        display.setTextColor(GxEPD_BLACK);
        display.setFont(&FreeMonoBold9pt7b);

        // -------- ENCABEZADO --------
        display.setCursor(235, 20);
        display.print("Estacion UAZ 1");
        display.drawLine(0, 30, 800, 30, GxEPD_BLACK);  // Línea divisoria

        // -------- PRIMERA FILA DE SENSORES --------
        // Temperatura
        display.drawXBitmap(10, 60, Temperatura45, 45, 45, GxEPD_BLACK);
        display.setCursor(70, 75);
        display.print("Temperatura: ");
        display.setCursor(70, 95);
        display.print(tempSHT, 1);
        display.print(" C");

        // Humedad
        display.drawXBitmap(220, 60, Humedad45, 45, 45, GxEPD_BLACK);
        display.setCursor(280, 75);
        display.print("Humedad: ");
        display.setCursor(280, 95);
        display.print(humSHT, 1);
        display.print(" %");

        // Radiación solar
        display.drawXBitmap(440, 60, Sol45, 45, 45, GxEPD_BLACK);
        display.setCursor(500, 75);
        display.print("Radiacion: ");
        display.setCursor(500, 95);
        display.print(irradiancia, 1);
        display.print(" W/m2");

        // -------- SEGUNDA FILA DE SENSORES --------
        // Altitud
        display.drawXBitmap(10, 130, Altitud45, 45, 45, GxEPD_BLACK);
        display.setCursor(70, 145);
        display.print("Altitud: ");
        display.setCursor(70, 165);
        display.print(altitud, 1);
        display.print(" m");

        // Presión
        display.drawXBitmap(220, 130, Presion45, 45, 45, GxEPD_BLACK);
        display.setCursor(280, 145);
        display.print("Presion: ");
        display.setCursor(280, 165);
        display.print(presion, 1);
        display.print(" hPa");

        // Dirección cardinal
        display.setCursor(480, 145);
        display.print("Direccion: ");
        display.print(dirViento.direccion);

        // -------- TERCERA FILA DE SENSORES --------
        // Velocidad del viento
        display.drawXBitmap(10, 210, Viento45, 45, 45, GxEPD_BLACK);
        display.setCursor(70, 225);
        display.print("Viento: ");
        display.setCursor(70, 245);
        display.print(viento, 1);
        display.print(" km/h");

        // Ráfaga máxima
        display.drawXBitmap(220, 210, Rafaga45, 45, 45, GxEPD_BLACK);
        display.setCursor(280, 225);
        display.print("Rafaga: ");
        display.setCursor(280, 245);
        display.print(rafaga, 1);
        display.print(" km/h");

        // Lluvia
        display.drawXBitmap(10, 270, LLuvia45, 45, 45, GxEPD_BLACK);
        display.setCursor(70, 285);
        display.print("Lluvia: ");
        display.setCursor(70, 305);
        display.print(lluvia, 1);
        display.print(" mm");

        // -------- ROSA DE LOS VIENTOS --------
        int centroX = 500, centroY = 240;
        drawCompassBitmap(centroX, centroY);
        drawCompassDirection(dirViento.direccion, centroX, centroY);

        // -------- LÍNEA DIVISORIA --------
        display.drawLine(0, 330, 800, 330, GxEPD_BLACK);

        // -------- INFORMACIÓN DE SEÑAL Y CONECTIVIDAD --------
        display.setFont(&FreeMono9pt7b);
        
        // Fila 1: RSSI LoRa, Tiempo encendido, Paquetes recibidos
        display.setCursor(5, 350);
        display.print("RSSI:");
        display.print(radio.getRSSI());
        display.print(" dBm");
        display.setCursor(210, 350);
        display.print("T Enc:");
        display.print(millis() / 1000);
        display.print(" s  Act:");
        display.print(paquetesRecibidos);

        // Fila 2: WiFi SSID y RSSI WiFi
        display.setCursor(5, 365);
        display.print("Red: ");
        display.print(WiFi.SSID());
        display.print(" RSSI: ");
        display.print(String(WiFi.RSSI()));
        display.print(" dBm");

        // -------- ESTADO DE ALMACENAMIENTO EEPROM --------
        int registros_guardados = eepromAddress / sizeof(DataRecord);
        int registros_totales   = EEPROM_SIZE / sizeof(DataRecord);
        int registros_libres    = registros_totales - registros_guardados;

        display.setCursor(438, 350);
        display.print("Guardado: ");
        display.print(registros_guardados);
        display.print(" / ");
        display.print(registros_totales);

      } while (display.nextPage());  // Actualiza display (bifásico)

      display.hibernate();  // Pantalla a modo bajo consumo
      radio.startReceive(); // Vuelve a recibir LoRa
    }
  }
}

