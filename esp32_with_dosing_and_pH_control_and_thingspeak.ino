#include <Arduino.h>
#include <Hash.h>

#include <WiFi.h>
#include <WebSocketsClient.h>
#include "SoftwareSerial.h"
#include <ArduinoJson.h>

//инициализация всего для работы ThingSpeak.com
#include <ThingSpeak.h>
const char* thingSpeakApiKey = "03IX7QQIDRH1P9OL"; //api key
unsigned long thingSpeakChannelId = 2391840; //channel ID
WiFiClient  client;
float lastThingSpeakWriteTime; //таймер для записи времени последней отправки данных на ThingSpeak
float ThingSpeakWriteInterval = 20000; //интервал отправки данных в облако 20сек (не менее 15 сек)
//переменные для отправки данных с датчиков на ThingSpeak.com
float sen_t_air = 0.0; 
float sen_hum_air = 0.0;
float sen_co2_air = 0.0;
float sen_ec = 0.0;
float sen_ph = 0.0;
float sen_t_h2o = 0.0;
bool dataReadyToSend = false; // Флаг готовности данных к отправке


#include <time.h>
#include <stdlib.h>
#define RXp2 16
#define TXp2 17

//новый код для дозировки
//назначение пинов для моторов, скоростей каждого их продолжительности работы цикла работы моторов
const int motorPins[3] = {27, 26, 25}; // Пины PWM, подключенные к моторам на ESP32
const int motorSpeeds[3] = {180, 174, 169}; // Скорость для каждого мотора от 1 до 255; эти скорости вы узнаете после калибровки своих перистальтических насосов
const int motorCycleTime = 600; // Время цикла работы мотора в миллисекундах (600мс) - за это время на заданной скорости мотор наливает 1 миллилитр

//переменные для контроля pH в заданных рамках
float ph; //TODO выпилить из кода эту переменную, заменить на уже рабочую sen_ph
bool isPhReceived = false; // Флаг для проверки, получено ли значение pH через serial2
unsigned long pumpUpTimer = 0; //таймеры для вкл/выключения моторов pH
unsigned long pumpDownTimer = 0;
const unsigned long pumpInterval = 5000; // Время работы насоса pH+ или pH- в миллисекундах (2 секунды)
const int pumpUpPin = 33; // Пин подключенный к мотору pH+
const int pumpDownPin = 32; // Пин подключенный к мотору pH-
unsigned long pumpUpStartTime = 0; //время включения насоса ph+
unsigned long pumpDownStartTime = 0; //время включения насоса ph-
bool pumpUpActive = false; //флаги активности моторов pH
bool pumpDownActive = false;
const long webInterval = 15000; //таймер запуска функции выравнивания pH (30сек)
unsigned long previouspHMillis = 0; //таймер отсечки работы моторов корректировки рН
//конец нового кода

const char* wlan_ssid             = "Xiaomi_7246";
const char* wlan_password         = "1258959v";
const char* ws_host               = "192.168.31.198";
const int   ws_port               = 8080; 

String roomName = "q";
String username = "esp32";
/** Flag if task should run */
bool tasksEnabled = false;
/** Task handle for the light value read task */
TaskHandle_t tempTaskHandle = NULL;

String softwareSerialData = "";
char character;

unsigned long previousMillis = 0;
unsigned long interval = 2000;

// base URL for SockJS (websocket) connection
// The complete URL will look something like this(cf. http://sockjs.github.io/sockjs-protocol/sockjs-protocol-0.3.3.html#section-36):
// ws://<ws_host>:<ws_port>/<ws_baseurl>/<3digits>/<randomstring>/websocket
// For the default config of Spring's SockJS/STOMP support, the default base URL is "/socketentry/".
const char* ws_baseurl            = "/chat/"; // don't forget leading and trailing "/" !!!

int lastPotValue = 0;

WebSocketsClient webSocket;

void setup() {
    tasksEnabled = true;
    Serial.begin(115200);

    connectToWifi();
    connectToWebSocket();
    srand(time(NULL));
    Serial2.begin(9600, SERIAL_8N1, RXp2, TXp2);


    //новый код для дозировки
    // Инициализация пинов моторов как выходов
    for (int i = 0; i < 3; i++) {
      pinMode(motorPins[i], OUTPUT);
      ledcSetup(i, 1000, 8); // Настройка канала PWM, частота 1000 Гц, разрешение 8 бит
      ledcAttachPin(motorPins[i], i); // Привязка пина мотора к каналу PWM
     }
    ThingSpeak.begin(client); //запуск клиента для отправки данных в облако ThingSpeak
    //конец нового кода
    pinMode(pumpUpPin, OUTPUT);
    pinMode(pumpDownPin, OUTPUT);
    
}
void loop() {
    webSocket.loop();
    //delay(150); зачем он тут нужен?

    unsigned long currentMillis = millis();
//убиваем симулятор
//     if (currentMillis - previousMillis > interval) {
//         previousMillis = currentMillis;


//         //“SEN_HUM_AIR”
//         int humidity = 50 + rand() % 5;
//         //sendMessage(roomName, username, String(humidity), "SEN_HUM_AIR"); // SIMULATOR
//         // Serial.println("[SIMULATOR]  SEN_HUM_AIR" + humidity);

//         //“SEN_T_AIR”
//         //int t_air = 20 + rand() % 35;
//         //sendMessage(roomName, username, Serial.readString(), "SEN_T_AIR"); // SIMULATOR
//         // Serial.println("[SIMULATOR]  SEN_T_AIR" + t_air);

//         //“SEN_CO2_AIR”
//         int co2_air = 450 + rand() % 50;
//         //sendMessage(roomName, username, String(co2_air), "SEN_CO2_AIR"); // SIMULATOR
//         // Serial.println("[SIMULATOR]  SEN_CO2_AIR" + co2_air);

//         //“SEN_T_H2O”
//         int t_h20 = 18 + rand() % 6;
//         //sendMessage(roomName, username, String(t_h20), "SEN_T_H2O"); // SIMULATOR
//         // Serial.println("[SIMULATOR]  SEN_T_H2O" + t_h20);

//         //“SEN_PH”
//         float ph = float_rand(5.5, 6.5);
//         //sendMessage(roomName, username, String(ph), "SEN_PH"); // SIMULATOR
//         // Serial.println("[SIMULATOR]  SEN_PH" + String(ph));

//         //“SEN_EC”
// //        float ec = float_rand(0.6, 2.2);
// //        sendMessage(roomName, username, String(ec), "SEN_EC"); // SIMULATOR
//         // Serial.println("[SIMULATOR]  SEN_EC" + String(ec));

//         //“SEN_LEVEL”
//         int level = rand() % 2;
//         //sendMessage(roomName, username, String(level), "SEN_LEVEL"); // SIMULATOR
//         // Serial.println("[SIMULATOR]  SEN_LEVEL" + level);
//     }
    serialGetData(currentMillis); //функция получения и парсинга данных с ардуино по Rx Tx

//новый код 
//вызов функции выравнивания pH каждые "webInterval" (30) секунд ТОЛЬКО при получении корректных данных с датчика рН (не nan и не inf)
  if (isPhReceived && (currentMillis - previouspHMillis >= webInterval)) {
      previouspHMillis = currentMillis;
      if (sen_ph > 0) { //этот if проверяет, что датчик pH не отвалился во время эксплуатации
      controlPumps(sen_ph, currentMillis); //смотрит, входит ли значение pH в заданные рамки и запускает ph+ или ph- если ph высокий или слишком низкий
      Serial.print("controlPumps-------------------->");Serial.println(sen_ph);
      } else {Serial.println("У вас pH отвалился, сделайте там чонить ");}
    }
    // Отключение насоса pH+ по истечении таймера pumpInterval (2000мс = 2сек)
  if (pumpUpActive && (currentMillis - pumpUpStartTime >= pumpInterval)) {
      digitalWrite(pumpUpPin, LOW); //тут выбрать HIGH или LOW в соответствии с типом реле - нормально включенное или нормально выключенное
      pumpUpActive = false;
      Serial.println("Отключение pH+ по таймеру");

    }
    // Отключение насоса pH- по истечении таймера pumpInterval (2000мс = 2сек)
  if (pumpDownActive && (currentMillis - pumpDownStartTime >= pumpInterval)) {
      digitalWrite(pumpDownPin, LOW);  //тут выбрать HIGH или LOW в соответствии с типом реле - нормально включенное или нормально выключенное
      pumpDownActive = false;
      Serial.println("Отключение pH- по таймеру");
  }
}

//код удержания pH в заданных рамках. Принимает переменную pH
void controlPumps(float ph, unsigned long currentMillis) {
  if (!isnan(ph) && !isinf(ph)) {  //TODO
    if (ph < 5.7 && !pumpUpActive) {
      digitalWrite(pumpUpPin, HIGH);
      pumpUpStartTime = currentMillis;
      pumpUpActive = true;
      Serial.print("Включение pH+ ");
      Serial.println(ph);
    }

    if (ph > 6.4 && !pumpDownActive) {
      digitalWrite(pumpDownPin, HIGH);
      pumpDownStartTime = currentMillis;
      pumpDownActive = true;
      Serial.print("Включение pH- ");
      Serial.println(ph);
    }
  }  
}
//конец нового кода

void serialGetData(unsigned long currentMillis) {


    while (Serial2.available() > 0) { //пока приходят данные через Serial2 и они не пустые, загоняем их в String line
        String line = Serial2.readStringUntil('\n'); //ищем в строке String line символ конца строки \n
        line.trim();  // Удаляем пробельные символы в начале и конце строки, если они есть
        Serial.println(line); //вывод в монитор порта строки, которую мы получили
        // Проверяем, что строка не пуста
        
        if (line.length() > 0) {
            char * sensor = strtok(&line[0], " "); //разделение строки на Sensor и Data
            char * data = strtok(NULL, " ");
                //новый код 
                //извлечение из строки данных с pH датчика, сохранение в переменную ph
                if (String(sensor) == "SEN_PH") {
                    String dataLower = String(data);
                    dataLower.toLowerCase();
                    // Проверка на специальные случаи перед преобразованием
                    if (dataLower == "inf" || dataLower == "nan") {
                        Serial.println("Некорректные данные с датчика pH: " + dataLower);

                    } else {
                        isPhReceived = true; //мы прошли проверку на корректность данных рН и начинает работать функция controlPumps
                        float ph = String(data).toFloat(); // Преобразование данных в float
                        sen_ph = ph;
                        Serial.print("Преобразованные данные с датчика pH: "); Serial.println(ph);
                    }
                } 
                else if (String(sensor) == "SEN_T_AIR") {
                  sen_t_air = String(data).toFloat();
                  dataReadyToSend = true;
                } else if (String(sensor) == "SEN_HUM_AIR") {
                  sen_hum_air = String(data).toFloat();
                  dataReadyToSend = true;
                } else if (String(sensor) == "SEN_CO2_AIR") {
                  sen_co2_air = String(data).toFloat();
                  dataReadyToSend = true;
                } else if (String(sensor) == "SEN_EC") {
                  sen_ec = String(data).toFloat();
                  dataReadyToSend = true;
                } else if (String(sensor) == "SEN_T_H2O") {
                  sen_t_h2o = String(data).toFloat();
                  dataReadyToSend = true;
                } //TODO add "else if" logic for SEN_H20_SWITCH_1

                      //отправка показаний всех датчиков на ThingSpeak, если прошло ThingSpeakWriteInterval секунд (20сек)
                      if (dataReadyToSend && (currentMillis - lastThingSpeakWriteTime >= ThingSpeakWriteInterval)) {
                          // Отправка данных
                          ThingSpeak.setField(1, sen_t_air);
                          ThingSpeak.setField(2, sen_hum_air);
                          ThingSpeak.setField(3, sen_co2_air);
                          ThingSpeak.setField(4, sen_ec);
                          ThingSpeak.setField(5, sen_ph);
                          ThingSpeak.setField(6, sen_t_h2o);
                          ThingSpeak.writeFields(thingSpeakChannelId, thingSpeakApiKey);

                          // Сброс флага и обновление таймера
                          dataReadyToSend = false;
                          lastThingSpeakWriteTime = currentMillis;
                      }



                      //было sendMessage(roomName, username, String(data), String(sensor));
                      if (sensor && data) {
                          String cleanedSensor = cleanString(sensor);
                          String cleanedData = cleanString(data);
                          //Serial.println("Cleaned Sensor: " + cleanedSensor); //отладочная информация для проверки отправленных данных на backednd
                          // Serial.println("Cleaned Data: " + cleanedData);
                          sendMessage(roomName, username, cleanedData, cleanedSensor); //TODO отправка данных на background
                      }
                    //конец нового кода
        } 
    }
}

//функция очистки данных от \r и \n перед отправкой на backend
String cleanString(String input) {
    String output = "";
    for (char c : input) {
        if (isPrintable(c) || c == '\n' || c == '\r') { // Проверяем, является ли символ печатным
            output += c;
        }
    }
    return output;
}

float float_rand( float min, float max ) //TODO попробовать загасить
{
    float scale = rand() / (float) RAND_MAX; /* [0, 1.0] */
    return min + scale * ( max - min );      /* [min, max] */
}

bool led1 = false;
bool led2 = false;
bool buzzer = false;

//“REL_PUMP1”
bool rel_pump1 = false;

//“REL_PUMP2”
bool rel_pump2 = false;

//“REL_PH”
bool rel_ph = false;

//“REL_MICRO”
bool rel_micro = false;

//“REL_GROW”
bool rel_grow = false;

//“REL_BLOOM”
bool rel_bloom = false;

//“REL_MIX”
bool rel_mix = false;

//“REL_LED1”
bool rel_led1 = false;

//“REL_LED2”
bool rel_led2 = false;

//“REL_FUN”
bool rel_fun = false;

//“REL_USM”
bool rel_usm = false;

//“REL_PELTIER”
bool rel_peltier = false;


void executeCommand(String command) {
    // Serial.println("Сommand: " + command);

        //“REL_PUMP1”
    if (command == "REL_PUMP1_ON") {
        rel_pump1 = true;
        Serial.printf("[SIMULATOR] REL_PUMP1 on!\n");
    } else if (command == "REL_PUMP1_OFF") {
        rel_pump1 = false;
        Serial.printf("[SIMULATOR] REL_PUMP1 off!\n");
    }

        //“REL_PUMP2”
    else if (command == "REL_PUMP2_ON") {
        rel_pump2 = true;
        Serial.printf("[SIMULATOR] REL_PUMP2 on!\n");
    } else if (command == "REL_PUMP2_OFF") {
        rel_pump2 = false;
        Serial.printf("[SIMULATOR] REL_PUMP2 off!\n");
    }

        //“REL_PH”
    else if (command == "REL_PH_ON") {
        rel_ph = true;
        Serial.printf("[SIMULATOR] REL_PH on!\n");
    } else if (command == "REL_PH_OFF") {
        rel_ph = false;
        Serial.printf("[SIMULATOR] REL_PH off!\n");
    }

        //“REL_MICRO”
    else if (command == "REL_MICRO_ON") {
        rel_micro = true;
        Serial.printf("[SIMULATOR] REL_MICRO on!\n");
    } else if (command == "REL_MICRO_OFF") {
        rel_micro = false;
        Serial.printf("[SIMULATOR] REL_MICRO off!\n");
    }

        //“REL_GROW”
    else if (command == "REL_GROW_ON") {
        rel_grow = true;
        Serial.printf("[SIMULATOR] REL_GROW on!\n");
    } else if (command == "REL_GROW_OFF") {
        rel_grow = false;
        Serial.printf("[SIMULATOR] REL_GROW off!\n");
    }

        //“REL_BLOOM”
    else if (command == "REL_BLOOM_ON") {
        rel_bloom = true;
        Serial.printf("[SIMULATOR] REL_BLOOM on!\n");
    } else if (command == "REL_BLOOM_OFF") {
        rel_bloom = false;
        Serial.printf("[SIMULATOR] REL_BLOOM off!\n");
    }

        //“REL_MIX”
    else if (command == "REL_MIX_ON") {
        rel_mix = true;
        Serial.printf("[SIMULATOR] REL_MIX on!\n");
    } else if (command == "REL_MIX_OFF") {
        rel_mix = false;
        Serial.printf("[SIMULATOR] REL_MIX off!\n");
    }

        //“REL_LED1”
    else if (command == "REL_LED1_ON") {
        rel_led1 = true;
        Serial.printf("[SIMULATOR] REL_LED1 on!\n");
    } else if (command == "REL_LED1_OFF") {
        rel_led1 = false;
        Serial.printf("[SIMULATOR] REL_LED1 off!\n");
    }

        //“REL_LED2”
    else if (command == "REL_LED2_ON") {
        rel_led2 = true;
        Serial.printf("[SIMULATOR] REL_LED2 on!\n");
    } else if (command == "REL_LED2_OFF") {
        rel_led2 = false;
        Serial.printf("[SIMULATOR] REL_LED2 off!\n");
    }

        //“REL_FUN”
    else if (command == "REL_FUN_ON") {
        rel_fun = true;
        Serial.printf("[SIMULATOR] REL_FUN on!\n");
    } else if (command == "REL_FUN_OFF") {
        rel_fun = false;
        Serial.printf("[SIMULATOR] REL_FUN off!\n");
    }

        //“REL_USM”
    else if (command == "REL_USM_ON") {
        rel_usm = true;
        Serial.printf("[SIMULATOR] REL_USM on!\n");
    } else if (command == "REL_USM_OFF") {
        rel_usm = false;
        Serial.printf("[SIMULATOR] REL_USM off!\n");
    }

        //“REL_PELTIER”
    else if (command == "REL_PELTIER_ON") {
        rel_peltier = true;
        Serial.printf("[SIMULATOR] REL_PELTIER on!\n");
    } else if (command == "REL_PELTIER_OFF") {
        rel_peltier = false;
        Serial.printf("[SIMULATOR] REL_PELTIER off!\n");
    }
}

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {

    switch (type) {
        case WStype_DISCONNECTED:
            Serial.printf("[WSc] Disconnected!\n");
            break;
        case WStype_CONNECTED:
        {
            Serial.printf("[WSc] Connected to url: %s\n",  payload);
        }
            break;
        case WStype_TEXT:
        {

            String text = (char*) payload;
            if (payload[0] == 'h') {

                Serial.println("Heartbeat!");

            } else if (payload[0] == 'o') {

                // on open connection
                char *msg = "[\"CONNECT\\naccept-version:1.1,1.0\\nheart-beat:10000,10000\\n\\n\\u0000\"]";
                webSocket.sendTXT(msg);
                delay(1000);


            } else if (text.startsWith("a[\"CONNECTED")) {

                // subscribe to some channels
                subscribeToChannel(roomName);
                delay(1000);
                sendMessage(roomName, username, username + " connected", "SYSTEM");
                sendMessage(roomName, username, "Hi there, this is esp32!", "USER");

            }
            else if (text.startsWith("a[\"MESSAGE")) {
                processJsonData(text);
            }
            break;
        }
        case WStype_BIN:
            Serial.printf("[WSc] get binary length: %u\n", length);
            //hexdump(payload, length);
            break;
    }

}

void subscribeToChannel(String _channelName) {
    String msg = "[\"SUBSCRIBE\\nid:sub-0\\ndestination:/topic/messages/" + _channelName + "\\n\\n\\u0000\"]";
    webSocket.sendTXT(msg);
}

void sendMessage(String _channelName, String _username, String _messageText, String _messageType) {
    String messageData =  "[\"SEND\\ndestination:/app/chat/" +
                          _channelName + "\\n\\n{\\\"username\\\":\\\"" +
                          _username + "\\\",\\\"message\\\":\\\"" +
                          _messageText + "\\\",\\\"messageType\\\":\\\"" +
                          _messageType + "\\\"}\\u0000\"]";
    // Serial.println(messageData);
    webSocket.sendTXT(messageData);
}

void processJsonData(String _received) {
    String json = extractString(_received);
    json.replace("\\", "");
    // Serial.println(json);
    DynamicJsonDocument doc(1024);
    deserializeJson(doc, json);
    JsonObject obj = doc.as<JsonObject>();
    String receivedMessage = obj["message"];
    String username = obj["username"];
    // Serial.println(receivedMessage);
    executeCommand(receivedMessage);
}

String extractString(String _received) {
    char startingChar = '{';
    char finishingChar = '}';

    String tmpData = "";
    bool _flag = false;
    for (int i = 0; i < _received.length(); i++) {
        char tmpChar = _received[i];
        if (tmpChar == startingChar) {
            tmpData += startingChar;
            _flag = true;
        }
        else if (tmpChar == finishingChar) {
            tmpData += finishingChar;
            break;
        }
        else if (_flag == true) {
            tmpData += tmpChar;
        }
    }

    return tmpData;

}

void connectToWifi() {
    delay(500);
    Serial.print("Logging into WLAN: "); Serial.print(wlan_ssid); Serial.print(" ...");
    WiFi.mode(WIFI_STA);
    WiFi.begin(wlan_ssid, wlan_password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println(" success.");
    Serial.print("IP: "); Serial.println(WiFi.localIP());
}

void connectToWebSocket() {
    Serial.println("0");

    String socketUrl = ws_baseurl;
    socketUrl += random(0, 999);
    socketUrl += "/";
    socketUrl += random(0, 999999); // should be a random string, but this works (see )
    socketUrl += "/websocket";
    Serial.println(ws_host + String(ws_port) + socketUrl);

    webSocket.begin(ws_host, ws_port, socketUrl);
    Serial.println("2");

    webSocket.setExtraHeaders();
    Serial.println("3");

    webSocket.onEvent(webSocketEvent);
    Serial.println("4");
}

//новый код для дозировки
//ниже - происходит активации моторов и одной из 5 функций дозировки, которые соответствуют каждому из стадии роста растений. 
//
//Вызов функции запускает дозировочные насосы в соответствии с текущей (выбранной) стадией роста растения
//Вызов функции добавляет удобрения в расчете на 1 литр раствора

//функция активации моторов. данная функция вызывается из функций "firstRoots()", firstTrueLeaves() и тд
void runMotor(int motorIndex, float cycles) {
  int duration = motorCycleTime * cycles; // Расчет продолжительности работы мотора
  ledcWrite(motorIndex, motorSpeeds[motorIndex]); // Установка скорости мотора с использованием PWM
  Serial.print("Мотор № ");Serial.print(motorIndex);Serial.print(". Объем (мл): ");Serial.println(cycles);
  delay(duration); // Поддержание работы мотора в течение рассчитанной продолжительности
  ledcWrite(motorIndex, 0); // Остановка мотора
}

//функция дозировки в соответствии с таблицей flora series (гидропоника) от производителя Terra Aquatica
//первые корешки
void firstRoots() {
  Serial.println("Первые корешки");
  runMotor(0, 0.5); //передаем в функцию запуска моторов runMotor номер мотора (0) и множитель cycles(0.5мл/л) для motorCycleTime (600мс)
  runMotor(1, 0.5); 
  runMotor(2, 0.5); 
}

//первые настоящие листья
void firstTrueLeaves() {
  Serial.println("Первые настоящие листья");
  runMotor(0, 1);
  runMotor(1, 1); 
  runMotor(2, 1); 
}

//рост
void growing() {
  Serial.println("Вегетация");
  runMotor(0, 1.8);
  runMotor(1, 1.2); 
  runMotor(2, 0.6); 
}

//предцвет
void preFlowering() {
  Serial.println("Предцвет");
  runMotor(0, 2);
  runMotor(1, 2); 
  runMotor(2, 1.5); 
}

//цветение
void flowering() {
  Serial.println("Цветение");
  runMotor(0, 0.8);
  runMotor(1, 1.6); 
  runMotor(2, 2.4); 
}


