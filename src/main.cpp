#include "config.h"
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Adafruit_INA219.h"
#include <WiFi.h>
#include "esp_sleep.h"
#include <rom/rtc.h>
#include <driver/rtc_io.h>
#include "Version.h"

#include <OneWire.h>
#include <DS18B20.h>
#define ONE_WIRE_BUS 15
OneWire oneWire(ONE_WIRE_BUS);
DS18B20 sensor(&oneWire);
bool sensorFound = false;

#include <esp_task_wdt.h>
//10 seconds Watchdog
#define WDT_TIMEOUT 10

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define uS_TO_mS_FACTOR 1000   /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  10       /* Time ESP32 will go to sleep (in seconds) */

String LoRa_incoming_Data;
float VBAT;  // battery voltage from ESP32 ADC read
const uint8_t vbatPin = 35;
const float vbatTreshold = 3.5;
//int txPwr = 0;

Adafruit_SSD1306 display(Lora_Screen_Width, LoRa_Screen_Height, &Wire, LoRa_Oled_RST);

RTC_DATA_ATTR long l_Timer_Bake = 0;
RTC_DATA_ATTR long l_Timer_Bake_Send = ((1000*60)*LoRa_Timer_Bake);

RTC_DATA_ATTR long l_Timer_Telemetry = (((1000*60)*LoRa_Timer_Telemetry)* -1);
RTC_DATA_ATTR long l_Timer_Telemetry_Send = ((1000*60)*LoRa_Timer_Telemetry);

RTC_DATA_ATTR int telSeqNr = 1; // survive deep Sleep

// Ina 219 Current/Voltage sensors. A=default 0x40
Adafruit_INA219 ina219_A;
Adafruit_INA219 ina219_B(0x41);
bool ina219_Afound = false;
bool ina219_Bfound = false;


void LoRa_send(String LoRa_str_Data, int LoRa_i_Header);
void LoRa_init_display();
void LoRa_init();
void LoRa_display(String LoRa_str_display, int LoRa_i_X, int LoRa_i_Y);
void send_telemetry();
void goto_sleep(int mseconds);
void onReceive(int packetSize);
void clockDown(bool par);
void sendBake(String beaconText);
float getBattV();
void sendStatus(String message);
bool getWakeupReason();


void setup() {
  
  Serial.println("Configuring WDT...");
  esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch

  LoRa_init_display();
  LoRa_init();

  Serial.begin(LoRa_Serial_Baudrate);
  pinMode(vbatPin, INPUT);

  if (getWakeupReason()){
    sendStatus("Wakeup");
  } else {
    sendStatus("Bootup");
  }
  esp_task_wdt_reset();

  //disable WiFi & BT
  WiFi.mode(WIFI_OFF);
  btStop();
  
  if (! ina219_A.begin()) {
    Serial.println("Failed to find INA219 chip 0x40");
    //while (1) { delay(10); }
    ina219_Afound = false;
  } else {
    ina219_Afound = true;
    ina219_A.setCalibration_16V_400mA();
  }
  
  if (! ina219_B.begin()) {
    Serial.println("Failed to find INA219 chip 0x41");
    //while (1) { delay(10); }
    ina219_Bfound = false;
  } else {
    ina219_Bfound = true;
    ina219_B.setCalibration_16V_400mA();
  }

  if (! sensor.begin()) {
    Serial.println("Failed to find DS18B20");
    //while (1) { delay(10); }
    sensorFound = false;
  } else {
    sensorFound = true;
    sensor.setResolution(12);
  }

  Serial.print("Version: "); Serial.println(VERSION);
  Serial.print("Build: "); Serial.println(BUILD_TIMESTAMP);

  if (getBattV() <= vbatTreshold && getBattV() > 2){
    goto_sleep(1000*60*30);
  }

  LoRa_display("Bootup OK, Display OFF",0,20);
  delay(1000);
  
  display.ssd1306_command(0x8D); //into charger pump set mode
  display.ssd1306_command(0x10); //turn off charger pump
  display.ssd1306_command(0xAE); //set OLED sleep
  display.ssd1306_command(SSD1306_DISPLAYOFF);
  clockDown(true);
}


void loop() {
  if (millis() > l_Timer_Bake_Send + l_Timer_Bake ) {
    l_Timer_Bake = millis();
    sendBake( "" );

    if (getBattV() <= vbatTreshold && getBattV() > 2){
      goto_sleep(1000*60*30);
    }
  }
  
  if (millis() > l_Timer_Telemetry_Send + l_Timer_Telemetry ) {
    l_Timer_Telemetry = millis();
    send_telemetry();    

    if (getBattV() <= vbatTreshold && getBattV() > 2){
      goto_sleep(1000*60*30);
    }

  }

  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    clockDown(false);
    String LoRaHeader;
    String sourceCall;
    String destCall;
    String message;
    String digiPath;
    String digis[8];

    while (LoRa.available()) {
      LoRa_incoming_Data = LoRa.readString();
      Serial.print("Rx:"); Serial.println(LoRa_incoming_Data);

      if (LoRa_incoming_Data.length() < 5)
        goto bad_packet;
      int posEndOfCall, posEndOfData;
      LoRaHeader = LoRa_incoming_Data.substring(0, 3);
      //if (dummy[0] != '<') {
      posEndOfCall = LoRa_incoming_Data.indexOf('>');
      if (posEndOfCall < 5)
        goto bad_packet;
      sourceCall = LoRa_incoming_Data.substring(3, posEndOfCall);
      posEndOfData = LoRa_incoming_Data.indexOf(':');
      if (posEndOfData < posEndOfCall)
        goto bad_packet;
      destCall = LoRa_incoming_Data.substring(posEndOfCall + 1, posEndOfData);
      message = LoRa_incoming_Data.substring(posEndOfData + 1);
      digiPath = "";
      posEndOfData = destCall.indexOf(',');
      if (posEndOfData > 0) {
        digiPath = destCall.substring(posEndOfData + 1);
        destCall = destCall.substring(0, posEndOfData);
      }
      if (destCall == "")
        goto bad_packet;

      // In LORA we don't support digipeating more than one time -> If somewhere in the path the digipeated marker '*' appears, don't repeat.
      if (digiPath.indexOf('*') != -1)
        goto already_repeated;
        
      bool via_digi_found;
      via_digi_found = false;
      posEndOfData = destCall.indexOf('-');
      if (posEndOfData > -1) {
        // DST Call digipeating?
        // DST-n>WIDEm is mutually exclusive
        if (digiPath.startsWith("WIDE") || digiPath.indexOf(",WIDE") > -1)
          goto bad_packet;
        // v this makes ^ this obsolete. If digiPath contains a real digi call for digipeating (and it has no repeated flad
        //                (but we well not repeat already repeated packets anywa..), it's also mutually exclusive to DST-Digipeating
        if (!(digiPath == "" || digiPath == "NOGATE" || digiPath == "RFONLY"))
          goto bad_packet;

        int ssid = destCall.substring(posEndOfData + 1).toInt();
        //if (ssid < 1 || ssid > 7) { // SSID 8-15: unsupported. < 1 and > 15 are invalid
        if (ssid < 1 || ssid > 15) { // yeah.. we won't be such restrictive 
          goto bad_packet;
        }
        // Decrement SSID; if SSID was -1, then remove "-1".
        destCall = destCall.substring(0, posEndOfData);
        if (ssid > 1)
          destCall = destCall + "-" + String(ssid - 1);
        via_digi_found = true;
      } else {
        // no DST Call Digipeating 
        if (!(digiPath.startsWith("WIDE") || digiPath.indexOf(",WIDE") > -1)){
          //Serial.println("No DST-1, no Wide, digipeating anyway");
          via_digi_found = true;
        }
      }


      int n_digis;
      int call_start;
      int call_end;
      bool at_last_digi;
      bool my_call_was_in_path;
      n_digis = 0;
      call_start = 0;
      call_end = 0;
      at_last_digi = false;
      my_call_was_in_path = false;

      while (true) {
        String digi;
        call_end = digiPath.indexOf(',', call_start);
        if (call_end == -1) {
          digi = digiPath.substring(call_start);
          at_last_digi = true;

        } else {
          if (n_digis == 6)
            goto max_digipeaters_reached;
          digi = digiPath.substring(call_start, call_end);
        }

        if (digi.startsWith("WIDE")) {
          // In LoRa, we repeat only once. Set WIDEn-m to WIDEn (-> cut off '-'), on every word "WIDEn" in the path
          int ssid_pos;
          if ((ssid_pos = digi.indexOf("-")) > -1)
            digi = digi.substring(0, ssid_pos);
          if (!via_digi_found)
            digi = digi + "*";
          via_digi_found = true;       
        } else {
          /* user specified a digipeater, i.e. ...>APRS,DL2BBB" or >APRS,DL2BBB,WIDE2-1, we must not repeat this. Except if our own call is addressed.
           * Thanks to !via_digi_found status we know we are in the position of digiPath before the first WIDE digi.
           */
          if (digi == String(LoRa_str_call)) {
            if (!via_digi_found) {
              digi = digi + "*";
              my_call_was_in_path = true;
            }
            via_digi_found = true;
          } else {
            if (!via_digi_found)
              goto no_via_digi_found;
          }
        }

        if (digi != "") {
          digis[n_digis] = digi;
          n_digis++;
        }

        if (at_last_digi)
          break;
          
        call_start = call_end + 1;
      }

      if (!via_digi_found)
        goto no_via_digi_found;

      // Now, look for the first reference of wide* and insert our callsign just before
      int i;
      int my_digi_pos;
      my_digi_pos = 0;
      if (!my_call_was_in_path) {
        for (i = n_digis-1; i >= 0; i--) {
          if (digis[i].endsWith("*") || i == 0) {
            my_digi_pos = i;
            int j;
            for (j = n_digis+1; j > i; j--)
              digis[j] = digis[j-1];
          }
        }

        // always add our repeater
        digis[my_digi_pos] = String(LoRa_str_call) + "*";
        n_digis++;
      }

      // rebuild digiPath
      digiPath = "";
      for (i = 0; i < n_digis; i++)
          digiPath = digiPath + "," + digis[i];

      //LoRa_display(String(sourceCall + " repeated!"), 0, 20);
      LoRa_incoming_Data = LoRaHeader + sourceCall + ">" + destCall + digiPath + ":" + message;
      Serial.print("Tx:"); Serial.println(LoRa_incoming_Data);
      LoRa_send(LoRa_incoming_Data,0);
      
bad_packet:
      // ignore bad packet
      //Serial.print("Bad");
no_via_digi_found:
      // no via digi found
      //Serial.print("No via Digi");
max_digipeaters_reached:
      // max. 8 digipeaters are allowed
      //Serial.print("Max Digis reached");
already_repeated:
      // already_repeated
      //Serial.print("Already repeated");
      ;

    }
    clockDown(true);
  }
  esp_task_wdt_reset();
}

void LoRa_send(String LoRa_str_Data, int LoRa_i_Header) {
  LoRa.setFrequency(LoRa_Frequency_TX);
  LoRa.setSpreadingFactor(LoRa_SpreadingFactor_TX);
  LoRa.setSignalBandwidth(LoRa_SignalBandwidth_TX);
  LoRa.setCodingRate4(LoRa_CodingRate4_TX);
  LoRa.beginPacket();
  if (LoRa_i_Header > 0) {
    LoRa.write('<');
    LoRa.write(0xFF);
    LoRa.write(0x01);
  }
  LoRa.write((const uint8_t *)LoRa_str_Data.c_str(), LoRa_str_Data.length());
  LoRa.endPacket();
  LoRa.setFrequency(LoRa_Frequency_RX); 
  LoRa.setSpreadingFactor(LoRa_SpreadingFactor_RX);
  LoRa.setSignalBandwidth(LoRa_SignalBandwidth_RX);
  LoRa.setCodingRate4(LoRa_CodingRate4_RX); 
}

void LoRa_init() {
  SPI.begin(LoRa_SCK, LoRa_MISO, LoRa_MOSI, LoRa_SS);
  LoRa.setPins(LoRa_SS, LoRa_RST, LoRa_DIO0);
  
  if (!LoRa.begin(LoRa_Frequency_RX)) {
    Serial.println(LoRa_str_failed);
    while (1);
  }

  LoRa.setSpreadingFactor(LoRa_SpreadingFactor_RX);
  LoRa.setSignalBandwidth(LoRa_SignalBandwidth_RX);
  LoRa.setCodingRate4(LoRa_CodingRate4_RX);
  LoRa.enableCrc();
  LoRa.setTxPower(LoRa_TxPower);
  //LoRa.onReceive(onReceive);
  delay(1000);  
}

void LoRa_display(String LoRa_str_display, int LoRa_i_X, int LoRa_i_Y) {
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0,0);
  display.println(LoRa_str_startup);
  display.setCursor(LoRa_i_X,LoRa_i_Y);
  display.print(LoRa_str_display);
  display.display();  
}

void LoRa_init_display() {
  pinMode(LoRa_Oled_RST, OUTPUT);
  digitalWrite(LoRa_Oled_RST, LOW);
  delay(20);
  digitalWrite(LoRa_Oled_RST, HIGH);

  Wire.begin(LoRa_Oled_SDA, LoRa_Oled_SCL);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3c, false, false)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); 
  }
  display.clearDisplay();
  LoRa_display(LoRa_str_display_ok,0,20);
  delay(1000);
}

void send_telemetry(){
  // read DS18B20 Temperature Sensor.
  // needs a while so start at first
  //if (sensorFound){
  //  sensor.requestTemperatures();
  //}
  
  // U/I from Battery
  float batt_shuntvoltage = 0;
  float batt_busvoltage = 0;
  float batt_current_mA = 0;
  float batt_loadvoltage = 0;
  //float batt_power_mW = 0;

  // U/I from PV
  float pv_shuntvoltage = 0;
  float pv_busvoltage = 0;
  float pv_current_mA = 0;
  float pv_loadvoltage = 0;
  //float pv_power_mW = 0;
  float tempC = 0;

  if (ina219_Bfound){
    batt_shuntvoltage = ina219_B.getShuntVoltage_mV();
    batt_busvoltage = ina219_B.getBusVoltage_V();
    batt_current_mA = ina219_B.getCurrent_mA() * -1;
    //batt_power_mW = ina219_B.getPower_mW();
    batt_loadvoltage = batt_busvoltage + (batt_shuntvoltage / 1000);
  } else {
    //VBAT from onboard Resistor Network  
    //VBAT = (float)(analogRead(vbatPin)) / 4095*2*3.3*1.1;
    batt_loadvoltage = (float)(analogRead(vbatPin)) *7.221/4096;
    //Serial.print("Vbat = "); Serial.print(VBAT); Serial.println(" Volts");
  }

  if (ina219_Afound){
    pv_shuntvoltage = ina219_A.getShuntVoltage_mV();
    pv_busvoltage = ina219_A.getBusVoltage_V();
    pv_current_mA = ina219_A.getCurrent_mA()  * -1;
    //pv_power_mW = ina219_A.getPower_mW();
    pv_loadvoltage = pv_busvoltage + (pv_shuntvoltage / 1000);
  }
  //LoRa_display("U=" + (String)pv_loadvoltage + "V " + "I=" + (String)pv_current_mA + "mA\nU=" + (String)batt_loadvoltage + "V " + "I=" + (String)batt_current_mA + "mA",0,20);

  clockDown(false);
  String loraOut;

  // calculate equated output to meet aprs specs
  int eqBattU = round((batt_loadvoltage - 2.5) / 0.010); // 0-255 eq: 0,0.010,2.5
  int eqPvU = round((pv_loadvoltage - 2.5) / 0.010); // 0-255 eq: 0,0.010,2.5
  float eqBattI = ((batt_current_mA + 500 ) / 1.25); // 0-999 eq: 0,1.001001001,-500 !! not in aprs spec.. but works !!
  //float eqBattI = ((batt_current_mA + 500 ) / 3.92157); // 0-255 eq: 0,3.92157,-500
  float eqPvI = ((pv_current_mA + 500 ) / 1.25); // 0-999 eq: 0,1.25,-500 !! not in aprs spec.. but works !!
  //float eqPvI = ((pv_current_mA + 500 ) / 1.001001001); // 0-999 eq: 0,1.001001001,-500 !! not in aprs spec.. but works !!
  //float eqPvI = ((pv_current_mA + 500 ) / 3.92157); // 0-255 eq: 0,3.92157,-500

  // get DS18B20 Reading
  if (sensorFound){
    sensor.requestTemperatures();
    while (!sensor.isConversionComplete());  // wait until sensor is ready
    tempC = ((sensor.getTempC() + 40) / 0.16016);
  }

  String teleCALL = String(LoRa_str_call);
	if (teleCALL.length() < 8){teleCALL += " ";}

  String teleHeader = String(LoRa_str_call) + 
          String(">") + String(LoRa_str_Dest)+
          String("-1:");

  String tele1 = "PARM.BattU,BattI,PvU,PvI,TempC,Temp,curA,curB,WiFi,5,6,7,8";
  String tele2 = "UNIT.Volt,mA,Volt,mA,deg.C,on,on,on,on,on,on,on,on";
  String tele3 = "EQNS.0,.01,2.5,0,1.25,-500,0,.01,2.5,0,1.25,-500,0,0.16016,-40";
  //String tele4 = "BITS.01001010,Solar Digi V" + String(VERSION);
  //String statusMessage = "SolarDigi";

  char tele0[255];
	sprintf(tele0, "T#%03d,%03d,%03.0f,%03d,%03.0f,%03.0f,%d%d%d00000", telSeqNr, eqBattU, eqBattI, eqPvU, eqPvI, tempC, sensorFound, ina219_Afound, ina219_Bfound); 
  //Serial.println(tele0);

  char tele4[255];
  sprintf(tele4, "BITS.11110000,SolarDigi V%s", VERSION);
  //Serial.println(tele4);

  loraOut = teleHeader + String(tele0);
  LoRa_send(loraOut, 1);
  Serial.println(loraOut);


  if (!(telSeqNr % 10)){ //10
    loraOut = teleHeader + String(":") + teleCALL + String(" :") + tele1;
    LoRa_send(loraOut, 1);
    Serial.println(loraOut);
  }

  if (!(telSeqNr % 12)){ //12
    loraOut = teleHeader + String(":") + teleCALL + String(" :") + tele2;
    LoRa_send(loraOut, 1);
    Serial.println(loraOut);
  }

  if (!(telSeqNr % 14)){ //14
    loraOut = teleHeader + String(":") + teleCALL + String(" :") + tele3;
    LoRa_send(loraOut, 1);
    Serial.println(loraOut);
  }

  if (!(telSeqNr % 16)){ //4
    loraOut = teleHeader + String(":") + teleCALL + String(" :") + String(tele4);
    LoRa_send(loraOut, 1);
    Serial.println(loraOut);
  }
  if (!(telSeqNr % 18)){ //18
    sendStatus("SolarDigi - dj1an.de/solardigi");
  }

	telSeqNr++;
	if ( telSeqNr > 999 ){telSeqNr = 1;}
  clockDown(true);
}

void goto_sleep(int mseconds){
  String loRa_str_Bake; 
  Serial.print("Going to Sleep for "); Serial.print(mseconds / 1000 ); Serial.println(" Seconds");
  if (mseconds >= 60000){
    loRa_str_Bake = "Sleeping for " + String(mseconds / 1000 / 60) + " min";
  } else {
    loRa_str_Bake = "Sleeping for " + String(mseconds / 1000 ) + " sec";
  }
  sendStatus(loRa_str_Bake);

  int ms = millis(); 
  pinMode(5,INPUT); 
  pinMode(14,INPUT); pinMode(15,INPUT); pinMode(16,INPUT); 
  pinMode(17,INPUT); pinMode(18,INPUT); pinMode(19,INPUT); 
  pinMode(26,INPUT); pinMode(27,INPUT); 
  //pinMode(GPS,INPUT); 
  delay(100); 

  LoRa.sleep();

  // works?!?
  display.ssd1306_command(0x8D); //into charger pump set mode
  display.ssd1306_command(0x10); //turn off charger pump
  display.ssd1306_command(0xAE); //set OLED sleep
  display.ssd1306_command(SSD1306_DISPLAYOFF);
  //esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  //esp_sleep_pd_config(ESP_PD_DOMAIN_MAX, ESP_PD_OPTION_OFF);
  //esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
  //esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
  //esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);

  esp_sleep_enable_timer_wakeup((mseconds - (millis() - ms)) * uS_TO_mS_FACTOR); 
 	//rtc_gpio_set_direction((gpio_num_t)33, RTC_GPIO_MODE_INPUT_ONLY);
	//rtc_gpio_pulldown_en((gpio_num_t)33);
  //rtc_gpio_pulldown_en((gpio_num_t)LORA_D1);
	//rtc_gpio_pullup_en((gpio_num_t)LORA_RST);
	//rtc_gpio_pullup_en((gpio_num_t)LORA_CS);
  //esp_sleep_enable_ext0_wakeup((gpio_num_t)LORA_D1,1);
  esp_deep_sleep_start();
  
} 

void clockDown(bool par){
  if ( par ) {
    setCpuFrequencyMhz(40);
  } else {
    setCpuFrequencyMhz(240);
  }
  return;
}

void sendBake(String beaconText){

  String loRa_str_Bake = (String)LoRa_str_call + ">" + String(LoRa_str_Dest) + ":!" + (String)LoRa_str_Lat + (String)LoRa_str_Overlay + (String)LoRa_str_Lon + (String)LoRa_str_Symbol+(String)LoRa_str_Comment + " " + (String)beaconText;
  LoRa_send(loRa_str_Bake, 1);
  //LoRa_display(loRa_str_Bake,0,20);
}

float getBattV(){
  float batt_loadvoltage = 0;
  float batt_busvoltage = 0;
  float batt_shuntvoltage = 0;
  if (ina219_Bfound){
    batt_shuntvoltage = ina219_B.getShuntVoltage_mV();
    batt_busvoltage = ina219_B.getBusVoltage_V();
    //batt_current_mA = ina219_B.getCurrent_mA() * -1;
    //batt_power_mW = ina219_B.getPower_mW();
    batt_loadvoltage = batt_busvoltage + (batt_shuntvoltage / 1000);
    Serial.print("Batt: "); Serial.println(batt_loadvoltage);
  } else {
    //VBAT from onboard Resistor Network  
    batt_loadvoltage = (float)(analogRead(vbatPin)) *7.221/4096;
    Serial.print("Vbat = "); Serial.print(batt_loadvoltage); Serial.println(" Volts");
  }
  return batt_loadvoltage;
}

void sendStatus(String message){

  String teleHeader = String(LoRa_str_call) + 
          String(">") + String(LoRa_str_Dest)+
          String(":");

  String loraOut = teleHeader + String(">") + message;
  LoRa_send(loraOut, 1);
  Serial.println(loraOut);
  return;

}

bool getWakeupReason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason){
    case ESP_SLEEP_WAKEUP_EXT0 : 
      Serial.println("Wakeup caused by external signal using RTC_IO"); 
      break;
    case ESP_SLEEP_WAKEUP_EXT1 : 
      Serial.println("Wakeup caused by external signal using RTC_CNTL"); 
      break;
    case ESP_SLEEP_WAKEUP_TIMER : 
      Serial.println("Wakeup caused by timer");
      return true; 
      break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : 
      Serial.println("Wakeup caused by touchpad"); 
      break;
    case ESP_SLEEP_WAKEUP_ULP : 
      Serial.println("Wakeup caused by ULP program"); 
      break;
    default : 
      Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); 
    break;
  }
  return false;
}
