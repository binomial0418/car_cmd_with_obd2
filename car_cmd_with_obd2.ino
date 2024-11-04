//1.GPIO 34連接無感藍牙模塊，當手機連接藍牙模塊時將GPIO電平拉高，如果車輛未啟動則觸發車門解鎖。
//2.藍牙模塊與手機斷線，GPIO34電平拉低，如果車輛未啟動則觸發車門上鎖。
//3.使用另外一塊esp8266與本板的GPIO 35連接，esp8266連接車輛acc電源，當車輛發動esp8266會將GPIO 35拉高，用來判斷車輛目前是否發動中。
//4.當GPIO 35電平高-->低，代表車輛關閉，則點亮LED燈。
//5.讀取OBD2(ESP32<-->藍牙elm327)判斷目前車門是否開啟，若開啟則開led。
//6.讀取OBD2(ESP32<-->藍牙elm327)判斷目前車門是否解鎖，若解鎖則開led。
//7.定時查詢server上是否有啟動車輛的命令，若有則進行啟動車輛。

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <base64.h>
#include "BluetoothSerial.h"
#include "ELMduino.h"

BluetoothSerial SerialBT;
#define ELM_PORT SerialBT
#define DEBUG_PORT Serial

#define checkBluePin      34 // 使用 GPIO34 作為偵測針腳(藍牙)
#define checkAccPin       35 // 使用 GPIO35 作為偵測針腳(汽車ACC ON)
#define RELAY_PIN_LOCK  5  // GPIO 5 鎖門
#define RELAY_PIN_BOOT  4  // GPIO 4 發車
#define RELAY_PIN_OPEN  15 // GPIO 15 開門
#define POWER_PIN       26 // GPIO 26 鑰匙的3.3v電源
#define R1_PIN          27 // GPIO 26 LED的3.3v電源
ELM327 myELM327;
RTC_DATA_ATTR int loopCounter = 0;  // 使用 RTC 記憶體保存深度睡眠次數
RTC_DATA_ATTR int preAct = 0;  // 使用 RTC 記憶體保存上次動作 1:unlock 0:lock
RTC_DATA_ATTR int preAccOn  = 0;  // 使用 RTC 記憶體保存汽車是否通電 0: off 1:on
RTC_DATA_ATTR int preLock  = 0;  // 使用 RTC 記憶體保存車門是否上鎖 0: unLock 1:Lock

// Wi-Fi 連接資訊
#include "wifi_info.h"

bool wakeFromBlue = false; //是否應由藍牙腳喚醒
bool wakeFromAcc = false; //是否應由Acc腳喚醒

//藍牙OBD2資訊
uint8_t remoteAddress[] = { 0x13, 0xE0, 0x2F, 0x8D, 0x5A, 0xF1 }; // ELM327 Bluetooth device MAC address
const char* pinCode = "1234"; // Pairing pin
int nb_query_state = SEND_COMMAND;  // Set the inital query state ready to send a command

//LED點亮設定
bool ledOn = false;
unsigned long ledOnStartTime = 0;
const unsigned long ledDuration = 30000; // 30秒後關閉

//定義讀取狀態，以door_open值開始讀
typedef enum { door_open,
               door_lock,
               SPEED} obd_pid_states;
obd_pid_states obd_state = door_open;
float mph = 0;

void setup() {
  Serial.begin(115200);
  // 檢查深度睡眠恢復的次數
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER) {
    loopCounter++;
    if (loopCounter == 999){
      loopCounter = 1;  
    }
    Serial.println(loopCounter);
  } else if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0) {
    Serial.println("藍牙喚醒");
    wakeFromBlue = true;
  }else if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT1) {
    Serial.println("ACC喚醒");
    wakeFromAcc = true;
  } else {
    loopCounter = 0;  // 如果是第一次啟動或非計時器喚醒，則重置計數器
  }

  // 設置 GPIO34 為輸入模式
  pinMode(checkBluePin, INPUT);  // GPIO34 是預設的輸入模式
  // 設置 GPIO35 為輸入模式
  pinMode(checkAccPin, INPUT);  // GPIO34 是預設的輸入模式

  // 設置為輸出模式，初始狀態為高電平  
  pinMode(RELAY_PIN_OPEN, OUTPUT);
  digitalWrite(RELAY_PIN_OPEN, HIGH);
  pinMode(RELAY_PIN_LOCK, OUTPUT);
  digitalWrite(RELAY_PIN_LOCK, HIGH);
  pinMode(RELAY_PIN_BOOT, OUTPUT);
  digitalWrite(RELAY_PIN_BOOT, HIGH);
  pinMode(POWER_PIN, OUTPUT); // for key power
  digitalWrite(POWER_PIN, LOW);  // 初始狀態下關閉電源輸出
  
  pinMode(R1_PIN, OUTPUT); // for key power
  digitalWrite(R1_PIN, LOW);  // 初始狀態下關閉電源輸出
  
  //汽車斷電時點燈解鎖 ,2024/11/04移到Loop section
  // if (digitalRead(checkAccPin) == LOW && preAccOn == 1){
  //   //digitalWrite(R1_PIN, HIGH); //先讓燈亮起再解鎖
  //   delay(100);
  //   unlockDoor();
  //   //openLightPower();
  // }
  //汽車未發動狀態下 才做發車或開鎖門動作
  if (digitalRead(checkAccPin) == LOW){
    //次數到達且無藍牙訊號且汽車目前未發動 發車
    if (digitalRead(checkBluePin) == LOW && wakeFromBlue == false) {
      bootCar();
    }
    //有藍牙且有偵測到人，開門
    if (checkOpenDoor() == true){ //preAct 1:unlock 0:lock
      unlockDoor();
    }
    //無藍牙訊號 關門
    if (digitalRead(checkBluePin) == LOW && preAct == 1){ 
      delay(3000); //避免藍牙訊號瞬間丟失
      if (digitalRead(checkBluePin) == LOW){
        lockDoor();
      }
    }
  }
  
  //連線OBD2 ,當汽車啟動時才連線OBD2
  if (digitalRead(checkAccPin) == HIGH){
    DEBUG_PORT.begin(115200);
    // Set Bluetooth name
    ELM_PORT.begin("ArduHUD", true);
    ConnectToElm327();
    if (!myELM327.connected){
      Serial.println("Connected to ELM327");
    }
  }
  
  if (digitalRead(checkAccPin) == HIGH){
    preAccOn = 1;
  } else {
    preAccOn = 0;
  }
  
  // 設置定時喚醒，汽車未啟動狀態才進入睡眠
  if (digitalRead(checkAccPin) != HIGH){
    setDeepSleep();
  }
}
/*********************************************
* loop setcion
**********************************************/
void loop() {
  ledOn = false;
  //汽車啟動時讀取OBD資料，汽車關閉後進入深度睡眠
  if (digitalRead(checkAccPin) == HIGH){
    //使用obd_state搭配switch循環讀取OBD2資料，順序為door_open-->door_lock-->tmps_p
    //若ELM327斷線 重新連線
    if (!myELM327.connected){
      Serial.println("reconnect elm327");
      ConnectToElm327();
    }
    if (myELM327.connected){
      switch (obd_state){
        case door_open:{
          checkDoorOpenStats();
          break;
        }
        case door_lock:{
          checkDoorLockStats();
          break;
        }
        case SPEED: //當車子速度到達X時關閉時LED
        {
          getCarSpeed();
          if (mph*1.65 > 10){
            ledOn = false;
            if (digitalRead(R1_PIN) == HIGH ){
              digitalWrite(R1_PIN, LOW);
              Serial.println("Led Off");
            }
          }
          break;
        }
      }
    }
    //set led on
    if (ledOn == true){
      ledOnStartTime = millis(); //若持續檢查到開燈註記，就重設時間讓他可以一直亮
    }
    setLedPowerNonWait();
  } else {
    //汽車斷電時點燈解鎖
    if (digitalRead(checkAccPin) == LOW && preAccOn == 1){
      //digitalWrite(R1_PIN, HIGH); //先讓燈亮起再解鎖
      delay(100);
      unlockDoor();
      //openLightPower();
    }
    //睡眠
    setDeepSleep();
  }
  delay(1000);
}
/************************************
* 睡眠設定
************************************/
void setDeepSleep(){
  Serial.println("進入深度睡眠模式(slow)...");
  // 設定 GPIO34 為外部喚醒源，當 GPIO34 高電平時喚醒
  if (digitalRead(checkBluePin) != HIGH){
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_34, 1);  // 1 表示高電平喚醒
  } else {
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_34, 0);  
  } 
  esp_sleep_enable_timer_wakeup(60 * 1000000);  // 60秒後喚醒
  esp_deep_sleep_start();  // 進入深度睡眠
}
/************************************
* 啟動引擎的主程式
************************************/
void bootCar(){
  //Serial.println("car boot");
  connectToWiFi();
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    WiFiClient client;
    String url = "http://www.inskychen.com/carcmd/checkcarboot.php";

    // 使用 WiFiClient 和 URL 初始化 HTTPClient
    http.begin(client, url);

    int httpCode = http.GET();  // 發送請求並獲取響應代碼

    if (httpCode > 0) {
      String payload = http.getString();  // 獲取網頁內容
      // 檢查 "boot" 標籤是否為 "enable"
      if (payload.indexOf("<boot>boot</boot>") != -1) {
        triggerRelays();
      }
    }
    http.end();  // 結束 HTTP 請求
    WiFi.disconnect(true);  // 斷開 Wi-Fi 連接     
  }
}
/************************************
* 連接WIFI
************************************/
void connectToWiFi() {
  int iCount = 0;

  // 掃描 Wi-Fi 網路
  int n = WiFi.scanNetworks();
  bool ssidFound = false;
  for (int i = 0; i < n; i++) {
    if (checkOpenDoor() == true){
      unlockDoor();
      ssidFound = false;
      break;
    }
    if (WiFi.SSID(i) == ssid) {
      ssidFound = true;
      break;
    }
  }

  // 如果未找到目標 SSID，跳過 Wi-Fi 動作
  if (!ssidFound) {
    Serial.println("未找到目標 SSID，跳過本次 WiFi 動作...");
    return;
  }
  // 連接 Wi-Fi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    iCount++;
    if (iCount > 20) {
      Serial.println("停止連接");
      break;
    }
  }

  if (iCount <= 20) {
    Serial.println("WiFi 連接成功");
  } else {
    Serial.println("WiFi 連接失敗");
  }
}
/************************************
* 連接汽車遙控器啟動引擎
************************************/
void triggerRelays() {
  Serial.println("引擎啟動");
  digitalWrite(POWER_PIN, HIGH);  // 打開3.3V電源輸出
  delay(100);

  // 觸發繼電器 A 一秒
  digitalWrite(RELAY_PIN_LOCK, LOW);
  delay(200);
  digitalWrite(RELAY_PIN_LOCK, HIGH);
  delay(100);
  // 觸發繼電器 B 4秒
  digitalWrite(RELAY_PIN_BOOT, LOW);
  delay(4000);
  digitalWrite(RELAY_PIN_BOOT, HIGH);
  //send_line("BVB-7980 啟動中...");
  digitalWrite(POWER_PIN, LOW);  // 關閉3.3V電源輸出
}
/************************************
* 連接汽車遙控器解鎖車門
************************************/
void unlockDoor() {
  Serial.println("open door..");
  digitalWrite(POWER_PIN, HIGH);  // 打開3.3V電源輸出
  delay(100);

  digitalWrite(RELAY_PIN_OPEN, LOW);
  delay(200);
  digitalWrite(RELAY_PIN_OPEN, HIGH);
  delay(100);
  preAct = 1;  //preAct 1:unlock 0:lock
  digitalWrite(POWER_PIN, LOW);  // 關閉3.3V電源輸出
  //openLightPower();
}
/************************************
* 是否觸發鎖車門條件
************************************/
void lockDoor() {
  Serial.println("lock door");
  digitalWrite(POWER_PIN, HIGH);  // 打開3.3V電源輸出
  delay(100);
  
  digitalWrite(RELAY_PIN_LOCK, LOW);
  delay(200);
  digitalWrite(RELAY_PIN_LOCK, HIGH);
  delay(100);
  preAct = 0;//preAct 1:unlock 0:lock
  digitalWrite(POWER_PIN, LOW);  // 關閉3.3V電源輸出
}
/************************************
* 是否觸發解鎖車門條件
************************************/
bool checkOpenDoor(){
  if (digitalRead(checkBluePin) == HIGH && preAct == 0){ //preAct 1:unlock 0:lock
    return true;
  } else {
    return false;
  }
}
/************************************
* 觸發點燈X秒，但是當藍牙離開範圍就熄燈
************************************/
void openLightPower() {
  int lightTime = 60; //預設點燈X秒
  Serial.println("點燈");
  digitalWrite(R1_PIN, HIGH);
  for (int i = 0; i < lightTime; i++){
    if (digitalRead(checkBluePin) != HIGH){
      break;
    } else {
      delay(1000);
    }
  }
  digitalWrite(R1_PIN, LOW);
  Serial.println("關燈");
}
/************************************
* 從OBD2取得門鎖狀態
************************************/
void checkDoorLockStats() {
  if (nb_query_state == SEND_COMMAND)  // We are ready to send a new command
  {
    myELM327.sendCommand_Blocking("ATSH 770");
    delay(200);
    myELM327.sendCommand("22bc04");           // Send the custom PID commnad
    nb_query_state = WAITING_RESP;            // Set the query state so we are waiting for response
  } else if (nb_query_state == WAITING_RESP)  // Our query has been sent, check for a response
  {
    myELM327.get_response();  // Each time through the loop we will check again
  }
  if (myELM327.nb_rx_state == ELM_SUCCESS)  // Our response is fully received, let's get our data
  {
    if (myELM327.payload[22] == '1'){
      Serial.println("Door Lock");  
      preLock = 1; //0: unLock 1:Lock
    } else {
      Serial.println("Door UnLock");
      if (preLock = 1){ //從鎖門-->解鎖才要開LED
        ledOn = true;//點亮Led
      }
      preLock = 0; //0: unLock 1:Lock
    }
    nb_query_state = SEND_COMMAND;                       // Reset the query state for the next command
    obd_state = SPEED;                                   // 切換下一個讀取的狀態
    delay(200);                                          // Wait 5 seconds until we query again
  } else if (myELM327.nb_rx_state != ELM_GETTING_MSG) {  // If state == ELM_GETTING_MSG, response is not yet complete. Restart the loop.
    nb_query_state = SEND_COMMAND;                       // Reset the query state for the next command
    myELM327.printError();
    obd_state = SPEED;                                   // 切換下一個讀取的狀態
    delay(200);  // Wait 5 seconds until we query again
  }
}
/************************************
* 從OBD2取得車門開啟狀態
************************************/
void checkDoorOpenStats() {
  if (nb_query_state == SEND_COMMAND)  // We are ready to send a new command
  {
    myELM327.sendCommand_Blocking("ATSH 770");
    delay(200);
    myELM327.sendCommand("22bc03");           
    nb_query_state = WAITING_RESP;            // Set the query state so we are waiting for response
  } else if (nb_query_state == WAITING_RESP)  // Our query has been sent, check for a response
  {
    myELM327.get_response();  // Each time through the loop we will check again
  }
  if (myELM327.nb_rx_state == ELM_SUCCESS)  // Our response is fully received, let's get our data
  {
    if (myELM327.payload[21] == '2' || myELM327.payload[21] == '3'){ //2 or 3 open ,0:close
      Serial.println("Front Door open");  
      ledOn = true;//點亮Led
    } 
    if (myELM327.payload[22] == 'B' || myELM327.payload[22] == 'E'){ //B or E OPEN ,A:close
      Serial.println("Rear Door Open");  
      ledOn = true;//點亮Led
    }
    nb_query_state = SEND_COMMAND;                       // Reset the query state for the next command
    obd_state = door_lock;                              // 切換下一個讀取的狀態
    delay(200);                                         // Wait 5 seconds until we query again
  } else if (myELM327.nb_rx_state != ELM_GETTING_MSG) {  // If state == ELM_GETTING_MSG, response is not yet complete. Restart the loop.
    nb_query_state = SEND_COMMAND;                       // Reset the query state for the next command
    myELM327.printError();
    obd_state = door_lock;                                  // 切換下一個讀取的狀態
    delay(200);  // Wait 5 seconds until we query again
  }
}
/************************************
* 從OBD2取得車速
************************************/
void getCarSpeed() {
  mph = myELM327.mph();
  if (myELM327.nb_rx_state == ELM_SUCCESS)
  {
    obd_state = door_open;
  }
  else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
  {
    myELM327.printError();
    obd_state = door_open;
  }
}
/************************************
* 連線藍牙elm327
************************************/
void ConnectToElm327(){
  // Set pairing pin
  SerialBT.setPin(pinCode, strlen(pinCode));
  // Connect to specific MAC address
  for (int i = 0; i < 10; i++) {
    if (!ELM_PORT.connect(remoteAddress)) {
      DEBUG_PORT.println("Couldn't connect to OBD scanner - Phase 1");
      delay(200);
    } else {
      break;
    }
  }
  // Initialize ELM327
  for (int i = 0; i < 10; i++) {
    if (!myELM327.begin(ELM_PORT, false, 2000,'0',50)) {
      Serial.println("Couldn't connect to OBD scanner - Phase 2");
      delay(200);
    } else {
      break;
    }
  }
}
/************************************
* 使用非阻塞式點亮LED並計時
*************************************/
void setLedPowerNonWait() {
  if (digitalRead(R1_PIN) != HIGH && ledOn == true){
    Serial.println("Led on");
    digitalWrite(R1_PIN, HIGH);
    ledOnStartTime = millis();
  }

  // 檢查是否該熄滅
  if (digitalRead(R1_PIN) == HIGH && (millis() - ledOnStartTime >= ledDuration)) {
    digitalWrite(R1_PIN, LOW);
    Serial.println("Led Off");
  }
}