#include <WiFi.h>
#include <HTTPClient.h>
#include <SPI.h>
#include <LoRa.h>
#include <Update.h>


// WiFi Configuration
const char* ssid = "HiTech";
const char* password = "rLegBjgHxaEEcR9";

// Setup
int TipoX = 200;
int SetupX = 1;
//------------------------------------------
// Lora Autenticacion
unsigned char PalabraSincronizadora = 0xCC;
String IdentificadorX = "0xCC";
//------------------------------------------
// Sistema
int FirmwareX = 20240823;  // Versión de Firmware
String VersionX = "Versión: T_LoRa_Gateway_S" + String(SetupX);
String ImeiX, ImsiX, CcidX, SimX, ConexionX;
int SenalX;  // Rango de 0-31 , 31 es mejor
int SenalLoraX;
//------------------------------------------
// Operador
const char simPIN[] = "";
// Movistar
const char apn_1[] = "internet.movistar.com.co";
const char gprsUser_1[] = "movistar";
const char gprsPass_1[] = "movistar";
// Claro
const char apn_2[] = "internet.comcel.com.co";
const char gprsUser_2[] = "comcel";
const char gprsPass_2[] = "comcel";

const char apn_3[] = "internet.tigo.pa";
const char gprsUser_3[] = "internet.tigo.pa";
const char gprsPass_3[] = "internet.tigo.pa";

const char apn_4[] = "apn01.cwpanama.com.pa";
const char gprsUser_4[] = "";
const char gprsPass_4[] = "";
//------------------------------------------
// Servidor
const char server_gsm[] = "sensores.micentinela.com";
String Vps_Lectura = "/nic.php?";
const int port = 80;
// Ota
String overTheAirURL = "http://sensores.micentinela.com/nic_update/";
String LinkX, TimeX;

// Intervalo para actualizacion
unsigned long lastExecutionTime = 0;
unsigned long interval = 1800000;  // Intervalo-> 30 minutos
//------------------------------------------
// Buffer
const int bufferSize = 100;
String buffer[bufferSize];
int bufferIndex = 0;
//------------------------------------------
// Modem
#define TINY_GSM_MODEM_SIM7600   // Modem is SIM800
#define TINY_GSM_RX_BUFFER 1024  // Set RX buffer to 1Kb

#include <TinyGsmClient.h>
#include <ArduinoHttpClient.h>

#define SerialMon Serial  // Establecer serie para la consola de depuración (a Serial Monitor, velocidad predeterminada 115200)
#define SerialAT Serial1  // Establecer serie para comandos AT (al módulo SIM800)

// TTGO T-Call pins
#define UART_BAUD 115200
#define PIN_DTR 25
#define PIN_TX 26
#define PIN_RX 27
#define PWR_PIN 4
#define RESET 5
#define BAT_ADC 35
#define BAT_EN 12
#define PIN_RI 33

#define GSM_NL "\r\n"
//------------------------------------------
// Conexion Lora -> LilyGo Lora32
#define SCK 14   // GPIO5  -- SX1278's SCK
#define MISO 2   // GPIO19 -- SX1278's MISnO
#define MOSI 15  // GPIO27 -- SX1278's MOSI
#define SS 13    // GPIO18 -- SX1278's CS
#define RST 22   // GPIO14 -- SX1278's RESET
#define DI0 21   // GPIO26 -- SX1278's IRQ(Interrupt Request)
#define BAND 915E6
//------------------------------------------
// TinyGSM Client for Internet connection
TinyGsm modem(SerialAT);
TinyGsmClient client(modem);
HttpClient http(client, server_gsm, port);
//------------------------------------------
// Interruccion Temporizador
hw_timer_t* timer = NULL;       // El puntero del temporizador de hardware
int Interruptor_Tiempo = 1800;  // Tiempo en segundos (600 -> 10 minutos)

// Llamada de funcion en memoria RAM -> Mayor velocidad de Respuesta
void IRAM_ATTR onTimer() {
  Serial.println("Reiniciando ESP32");
  ESP.restart();
}
//------------------------------------------
TaskHandle_t HttpTask;
TaskHandle_t HttpTaskWiFi;
TaskHandle_t AnotherTask1;
TaskHandle_t AnotherTask2;
void Http_Vps(void* parameter);
void Http_Vps_WiFi(void* parameter);
void AnotherTask1Func(void* parameter);
void AnotherTask2Func(void* parameter);
//------------------------------------------
void setup() {
  // Interruptor Temporizados
  timer = timerBegin(1000000);
  timerAttachInterrupt(timer, &onTimer);       
  timerAlarm(timer, 1000000 * Interruptor_Tiempo, true, 0);

  // Crear una tarea para ejecutar Http_Vps en un núcleo
  xTaskCreatePinnedToCore(
    Http_Vps,    // Función a ejecutar
    "HttpTask",  // Nombre de la tarea
    10000,       // Tamaño de la pila de la tarea
    NULL,        // Parámetro de la tarea
    2,           // Prioridad de la tarea
    &HttpTask,   // Identificador de la tarea
    0            // Número del núcleo (0 o 1, dependiendo del núcleo que quieras utilizar)
  );

  // Crear una tarea para ejecutar Http_Vps_WiFi en el otro núcleo
  xTaskCreatePinnedToCore(
    Http_Vps_WiFi,    // Función a ejecutar
    "HttpTaskWiFi",  // Nombre de la tarea
    10000,       // Tamaño de la pila de la tarea
    NULL,        // Parámetro de la tarea
    2,           // Prioridad de la tarea
    &HttpTaskWiFi, // Identificador de la tarea
    1            // Número del núcleo (0 o 1, dependiendo del núcleo que quieras utilizar)
  );

  // Crear otra tarea para ejecutar AnotherTask1 en cualquier núcleo
  xTaskCreatePinnedToCore(
    AnotherTask1Func,    // Función a ejecutar
    "AnotherTask1",  // Nombre de la tarea
    10000,       // Tamaño de la pila de la tarea
    NULL,        // Parámetro de la tarea
    1,           // Prioridad de la tarea
    &AnotherTask1, // Identificador de la tarea
    tskNO_AFFINITY // No específico el núcleo, el planificador elige
  );

  // Crear otra tarea para ejecutar AnotherTask2 en cualquier núcleo
  xTaskCreatePinnedToCore(
    AnotherTask2Func,    // Función a ejecutar
    "AnotherTask2",  // Nombre de la tarea
    10000,       // Tamaño de la pila de la tarea
    NULL,        // Parámetro de la tarea
    1,           // Prioridad de la tarea
    &AnotherTask2, // Identificador de la tarea
    tskNO_AFFINITY // No específico el núcleo, el planificador elige
  );

  // Setup Serial Monitor
  SerialMon.begin(115200);

  Serial.println(VersionX);
  Serial.println("Firmware:" + String(FirmwareX));

  // Conexión WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    SerialMon.print(".");
  }
  SerialMon.println("WiFi conectado");

  // Conexión LoRa
  Lora_Start();
  LoRa.setSyncWord(PalabraSincronizadora);

  // Conexión GSM
  Modem_Power_On();
  Modem_Setup();

  ImeiX = modem.getIMEI();
  SenalX = modem.getSignalQuality();

  Log();
}
//------------------------------------------
void loop() {
  if (millis() - lastExecutionTime >= interval) {
    lastExecutionTime = millis();
    Http_Actualizacion();
  }

  int packetSize = LoRa.parsePacket();

  if (packetSize) {
    LinkX = "";

    String received = LoRa.readString();

    if (received.startsWith(IdentificadorX)) {
      // Procesar mensaje
      LinkX = received.substring(4);

      SenalLoraX = LoRa.packetRssi();
      LinkX = LinkX + " Se:" + SenalX + " Si:" + PalabraSincronizadora + "&sen=" + String(SenalLoraX);

      buffer[bufferIndex] = LinkX;
      bufferIndex++;
      Serial.println("Lora Recibido:");
      Serial.println(LinkX);
      Serial.println(".........");
    }
  }
}
//------------------------------------------
void Http_Vps(void* parameter) {
  while (true) {
    if (bufferIndex > 0) {
      for (int i = 0; i < bufferIndex; i++) {
        String url = Vps_Lectura + buffer[i] + "&sim=" + SimX;
        Serial.println("Lora Enviado:");
        Serial.println(url);

        // Conectando GPRS
        if (!modem.isGprsConnected()) {
          modem.gprsDisconnect();
          SerialMon.println(F("GPRS disconnected"));
          bufferIndex = 0;
          Modem_Setup();
        } else {
          // Enviar Lectura
          SerialMon.print(F("Request HTTP GET... "));
          int err = http.get(url);
          if (err != 0) {
            SerialMon.println(F("Error HTTP"));
            http.stop();
            bufferIndex = 0;
          } else {
            // Respuesta del servidor
            int status = http.responseStatusCode();
            SerialMon.print(F("Response: "));
            SerialMon.println(status);

            // Determinar si hay que reintentar
            if (status == 200) {
              // Reiniciar el temporizador
              timerRestart(timer);
            }

            http.stop();
            client.stop();
          }
        }
        Serial.println(".........");

        vTaskDelay(1000 / portTICK_PERIOD_MS);  // Esperar 1 segundo
      }

      bufferIndex = 0;
    }
    vTaskDelay(60000 / portTICK_PERIOD_MS);  // Esperar 1 minuto
  }
}
//------------------------------------------
void Http_Vps_WiFi(void* parameter) {
  while (true) {
    if (bufferIndex > 0) {
      for (int i = 0; i < bufferIndex; i++) {
        String url = "http://sensores.micentinela.com/nic.php?" + buffer[i] + "&sim=" + SimX;
        Serial.println("Lora Enviado WiFi:");
        Serial.println(url);

        if (WiFi.status() != WL_CONNECTED) {
          Serial.println("WiFi desconectado, intentando reconectar...");
          WiFi.begin(ssid, password);
          while (WiFi.status() != WL_CONNECTED) {
            delay(500);
            Serial.print(".");
          }
          Serial.println("WiFi reconectado");
        }

        if (WiFi.status() == WL_CONNECTED) {
          HTTPClient httpWifiClient;
          httpWifiClient.begin(url);
          int httpCode = httpWifiClient.GET();
          if (httpCode > 0) {
            String payload = httpWifiClient.getString();
            Serial.println("HTTP Response code: " + String(httpCode));
            Serial.println("Response: " + payload);
            if (httpCode == 200) {
              timerRestart(timer);
            }
          } else {
            Serial.println("Error en la solicitud HTTP");
          }
          httpWifiClient.end();
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);  // Esperar 1 segundo
      }

      bufferIndex = 0;
    }
    vTaskDelay(60000 / portTICK_PERIOD_MS);  // Esperar 1 minuto
  }
}

//------------------------------------------
void AnotherTask1Func(void* parameter) {
  while (true) {
    // Lógica para la primera tarea adicional
    Serial.println("Ejecutando AnotherTask1");
    vTaskDelay(5000 / portTICK_PERIOD_MS);  // Esperar 5 segundos
  }
}

//------------------------------------------
void AnotherTask2Func(void* parameter) {
  while (true) {
    // Lógica para la segunda tarea adicional
    Serial.println("Ejecutando AnotherTask2");
    vTaskDelay(10000 / portTICK_PERIOD_MS);  // Esperar 10 segundos
  }
}

//------------------------------------------
void Modem_Power_On() {
  pinMode(BAT_EN, OUTPUT);
  digitalWrite(BAT_EN, HIGH);

  // A7608 Reset
  pinMode(RESET, OUTPUT);
  digitalWrite(RESET, LOW);
  delay(100);
  digitalWrite(RESET, HIGH);
  delay(3000);
  digitalWrite(RESET, LOW);

  pinMode(PWR_PIN, OUTPUT);
  digitalWrite(PWR_PIN, LOW);
  delay(100);
  digitalWrite(PWR_PIN, HIGH);
  delay(1000);
  digitalWrite(PWR_PIN, LOW);
  delay(3000);

  SerialAT.begin(115200, SERIAL_8N1, PIN_RX, PIN_TX);
}
//------------------------------------------
void Modem_Setup() {
  SerialMon.println("Initializing modem...");
  modem.init();
  delay(1000);

  if (strlen(simPIN) && modem.getSimStatus() != 3) modem.simUnlock(simPIN);

  // Preferencia modo Conexion
  modem.setNetworkMode(38);  // Metodo -> Automatico(2) GSM(13) LTE(38) GSM y LTE(51)");
  delay(3000);

  // Conectar el dispositivo
  SerialMon.println("Waiting for network... ");
  // 60000L -> 1 Minuto
  if (!modem.waitForNetwork(60000L)) {
    SerialMon.println("Network not connected");
    Modem_Power_On();
    Modem_Setup();
  }

  // Datos Conexion
  ImsiX = modem.getIMSI();
  CcidX = modem.getSimCCID().substring(2, 19);

  if (ImsiX.substring(0, 5)) {
    if (ImsiX.substring(0, 5) == "21407") {
      SimX = ImsiX;
    }

    if (ImsiX.substring(0, 5) == "73210") {
      SimX = CcidX;
    }

    if (ImsiX.substring(0, 5) == "71402") {
      SimX = ImsiX;
    }

    if (ImsiX.substring(0, 5) == "71401") {
      SimX = ImsiX;
    }
  }

  // Conectadondo APN
  if (modem.isNetworkConnected()) {
    delay(1000);
    // Movistar
    if (ImsiX.substring(0, 5) == "21407") {
      SerialMon.println("Connecting to APN -> " + String(apn_1));
      if (modem.gprsConnect(apn_1, gprsUser_1, gprsPass_1)) {
        SerialMon.println("Network connected");
      }
    }

    // Tigo
    else if (ImsiX.substring(0, 5) == "73210") {
      SerialMon.println("Connecting to APN -> " + String(apn_2));
      if (modem.gprsConnect(apn_2, gprsUser_2, gprsPass_2)) {
        SerialMon.println("Network connected");
      }
    }

    // Mas Movil
    if (ImsiX.substring(0, 5) == "71401") {
      SerialMon.println("Connecting to APN -> " + String("+Movil"));
      if (modem.gprsConnect(apn_4, gprsUser_4, gprsPass_4)) {
        SerialMon.println("Network connected");
      }
    }

    // Tipo de Conexionn
    String res;
    modem.sendAT("+CPSI?");
    if (modem.waitResponse(1000L, res) == 1) {
      res.replace(GSM_NL "OK" GSM_NL, "");
      res.trim();
      res.remove(0, 7);
      Serial.println(res);
      res.remove(3, 80);
      ConexionX = res;
    }
  }

  SenalX = modem.getSignalQuality();
  delay(1000);
  Serial.println("Señal Lte:" + String(SenalX) + " -> Rango de 1 a 31");
}
//------------------------------------------
void Lora_Start() {
  delay(2000);

  SPI.begin(SCK, MISO, MOSI, SS);
  LoRa.setPins(SS, RST, DI0);

  while (!LoRa.begin(BAND)) {
    Serial.print(".");
    delay(500);
  }

  Serial.println("");
  Serial.println("LoRa Gateway iniciado...");
}
//----------------------------------------
void Http_Actualizacion() {
  LinkX = Vps_Lectura + "&sim=" + SimX + "&mac=" + ImeiX + "&fir=" + FirmwareX + "&tip=" + TipoX + "&set=" + SetupX;
  Serial.print("Validacion Ota:");
  Serial.println(LinkX);

  // Conectando GPRS
  if (!modem.isGprsConnected()) {
    modem.gprsDisconnect();  // Desconectar gprs
    SerialMon.println(F("GPRS disconnected"));
    Modem_Setup();
    return;
  } else {
    // Enviar Lectura
    SerialMon.print(F("Request HTTP GET... "));
    int err = http.get(LinkX);
    if (err != 0) {
      SerialMon.println(F("Error HTTP"));
    }

    // Respuesta del servidor
    int status = http.responseStatusCode();
    SerialMon.print(F("Response: "));
    SerialMon.println(status);

    // Determinar si hay que reintentar
    if (status == 200) {
      // Reincia el temporizador
      timerRestart(timer);
      Server_Disconnected();
    }

    http.stop();
    client.stop();
  }
  Serial.println(".........");
}
//----------------------------------------
void Log() {
  Serial.println("Tipo:" + String(TipoX));
  Serial.println("Setup:" + String(SetupX));
  Serial.print("Clave Lora:0x");
  Serial.println(PalabraSincronizadora, HEX);
  Serial.println("Identificados:" + String(IdentificadorX));
  SerialMon.println("SIM:" + SimX);
  SerialMon.println("IMEI:" + ImeiX);
  Serial.println("Señal:" + String(SenalX) + " -> Rango de 1 a 31");
  Serial.println("..........");
}
//----------------------------------------
void Server_Disconnected() {
  String body = http.responseBody();
  if (body.toInt() > 1577836800) TimeX = body;
  if (body.toInt() > 1 && body.toInt() < 1577836800) {  // unixtime => 20200101
    overTheAirURL += body;
    OTA_Setup();
  } else {
    SerialMon.print(F("Time:"));
    SerialMon.println(String(body));
    // Cerrar conexion con el servidor
    unsigned long timeout = millis();
    while (client.connected() && millis() - timeout < 2000L) {
      // Print available data (HTTP response
      while (client.available()) {
        char c = client.read();
        SerialMon.print(c);
        timeout = millis();
      }
    }
    client.stop();
    SerialMon.println(F("Server disconnected"));
  }
}

//---------------------------------------- OTA //----------------------------------------
void OTA_Setup() {
  timerRestart(timer);  // Reinicia el Temporizador
  SerialMon.println("Starting OTA update in 3 seconds...");
  delay(3000);
  startOtaUpdate(overTheAirURL);
}

//----------------------------------------
void startOtaUpdate(const String& ota_url) {
  String protocol, host, url;
  int port;
  if (!parseURL(ota_url, protocol, host, port, url)) {
    SerialMon.println(F("Cannot parse URL"));
  }
  SerialMon.println(String("Connecting to ") + host + ":" + port);
  Client* client = NULL;
  if (protocol == "http") {
    client = new TinyGsmClient(modem);
    if (!client->connect(host.c_str(), port)) {
      SerialMon.println(F("Client not connected"));
    }
  } else {
    SerialMon.println(String("Unsupported protocol: ") + protocol);
  }
  client->print(String("GET ") + url + " HTTP/1.0\r\n"
                + "Host: " + host + "\r\n"
                + "Connection: keep-alive\r\n"
                + "\r\n");
  long timeout = millis();
  while (client->connected() && !client->available()) {
    if (millis() - timeout > 10000L) {
      SerialMon.println("Response timeout");
    }
  }
  String md5;
  int contentLength = 0;
  while (client->available()) {
    String line = client->readStringUntil('\n');
    line.trim();
    line.toLowerCase();
    if (line.startsWith("content-length:")) {
      contentLength = line.substring(line.lastIndexOf(':') + 1).toInt();
    } else if (line.startsWith("x-md5:")) {
      md5 = line.substring(line.lastIndexOf(':') + 1);
    } else if (line.length() == 0) {
      break;
    }
  }
  if (contentLength <= 0) {
    SerialMon.println("Content-Length not defined");
  }
  bool canBegin = Update.begin(contentLength);
  if (!canBegin) {
    Update.printError(SerialMon);
    SerialMon.println("OTA begin failed");
  }
  if (md5.length()) {
    SerialMon.println(String("Expected MD5: ") + md5);
    if (!Update.setMD5(md5.c_str())) {
      SerialMon.println("Cannot set MD5");
    }
  }
  SerialMon.println("Flashing...");
  int written = 0;
  int progress = 0;
  uint8_t buff[256];
  while (client->connected() && written < contentLength) {
    timeout = millis();
    while (client->connected() && !client->available()) {
      delay(1);
      if (millis() - timeout > 20000L) {
        SerialMon.println("Timeout");
        ESP.restart();
      }
    }
    int len = client->read(buff, sizeof(buff));
    if (len <= 0) continue;
    Update.write(buff, len);
    written += len;
    int newProgress = (written * 100) / contentLength;
    if (newProgress - progress >= 5 || newProgress == 100) {
      progress = newProgress;
      SerialMon.print(String("\r ") + progress + "%");
    }
  }
  SerialMon.println();
  if (written != contentLength) {
    Update.printError(SerialMon);
    SerialMon.println(String("Write failed. Written ") + written + " / " + contentLength + " bytes");
  }
  if (!Update.end()) {
    Update.printError(SerialMon);
    SerialMon.println(F("Update not ended"));
  }
  if (!Update.isFinished()) {
    SerialMon.println(F("Update not finished"));
  }
  SerialMon.println("=== Update successfully completed. Rebooting.");
  ESP.restart();
}

bool parseURL(String url, String& protocol, String& host, int& port, String& uri) {
  int index = url.indexOf(':');
  if (index < 0) {
    return false;
  }
  protocol = url.substring(0, index);
  url.remove(0, (index + 3));
  index = url.indexOf('/');
  String server = url.substring(0, index);
  url.remove(0, index);
  index = server.indexOf(':');
  if (index >= 0) {
    host = server.substring(0, index);           // hostname
    port = server.substring(index + 1).toInt();  // port
  } else {
    host = server;
    if (protocol == "http") {
      port = 80;
    } else if (protocol == "https") {
      port = 443;
    }
  }
  if (url.length()) {
    uri = url;
  } else {
    uri = "/";
  }
  return true;
}
//---------------------------------------- OTA //----------------------------------------
