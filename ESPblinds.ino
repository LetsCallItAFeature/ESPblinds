/*
   ESPblinds  V 1.0
   
   - shutters controlled according to a timetable
   - includes locking front shutters down in summer (summer-lock)
   - fabric shutters controlled according to a timetable and weather
   - all shutters up during the night unless summer-locked
   - no updating/controlling in the night (semi low-power)
   - OTA uploading of sketches supported
   - experimental webserver
   
   
   todo:
   - have a log of all IR transmissions on the server to better check if a transmission has failed
   - interface to change the timetable on the server
   
   
   by Kai Arnetzl

*/

/*
   note: autoformat in the Arduino IDE screws the timetable formatting up, need to adjust afterwards
   true = up, false = down
   pin 4 = D2
	 
	 RGB led colors:
	 - white:  booting up
   - green:  sending IR
   - cyan:   getting data from the internet
   - purple: sending a client page contents
   - blue:   recieving a new scetch via OTA
   - yellow: scheduled restart
   - red blinking: no WiFi connection
*/



#include <ESP8266mDNS.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <IRremoteESP8266.h>
#include <IRsend.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <JSON_Decoder.h>
#include <DarkSkyWeather.h>
#include <TimeLib.h>

#ifndef STASSID
#define STASSID "xxxxxxxx"    //add network name here
#define STAPSK  "xxxxxxxxxxxxx"  //add network password here
#endif
#define OTA_PASSWORD "xxxxxxxxxxx"  //add OTA password here
#define FRONT_ALL 7
#define BACK_MOST 8
#define BACK_LAST 9
#define ONE_WIRE_BUS 12
#define red_pin 14   //D5
#define green_pin 12 //D6
#define blue_pin 13  //D7

String api_key = "xxxxxxxxxxxxxxxxxxx";   //add DarkSky API key here
String latitude = "xx.xxxx";     //add latitude of the blinds here
String longitude = "xx.xxxx";    //add longitude of the blinds here
String units = "si";
String language = "";
const char* ssid = STASSID;
const char* password = STAPSK;
const long utcOffsetInSeconds = 3600;
static const char ntpServerName[] = "us.pool.ntp.org";
const int timeZone = 1;
const int NTP_PACKET_SIZE = 48;
byte packetBuffer[NTP_PACKET_SIZE];

bool first_time = true;
int last_shutter = 0;
int blink_time = 0;
int last;
bool blink_state = false;
bool error_notification = false;
bool r_state[3] = {true, true, true}; //0 = front shutters, 1 = fabric, 2 = back shutters
bool last_state[3] = {false, false, false};
int weather_data[3];
byte date;


//Compressed IR Codes
bool r1_1[24] =    {0, 0, 1, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1};
bool r1_2[24] =    {0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0};
bool r1_3[24] =    {0, 0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0};
bool r1_4[24] =    {0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0};
bool r1_5[24] =    {0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1};
bool r1_6[24] =    {0, 0, 1, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1};
bool r1_all[24] =  {0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0};
bool r2[24] =      {0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0};
bool r3[24] =      {0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0};

const byte front_lock_dates[2][2] = {{16, 5}, {16, 8}};  //Start(Day, Month), End(Day, Month)
const byte front_ignore_dates = 0;
/* 
   times ignore daylight saving time, all times are UTC-1
   timetable structure:
   down{Hour, Minute}, up{Hour, Minute}    for all of the r1 shutters (includes fabric)
   down{Hour, Minute}, up{Hour, Minute}    for the r2 and r3 shutters
   from{Day, Month}                        end date is the start date of the next section
   Setting a time to {0, 0} means that that action doesn't happen, so {{0, 0}, {0, 0}} means that the shutters will always stay up in that section.
*/
const byte times[2][8][2][2] =  {{{{10, 30}, {13, 0}}, {{9, 30}, {13, 30}}, {{8, 0}, {14, 0}}, {{6,  30}, {16, 30}}, {{8,   0}, {14, 0}}, {{8, 30}, {13, 30}}, {{9, 30}, {13, 0}}, {{10, 30}, {13, 0}}}, 
                                 {{{0,   0}, {0,  0}}, {{0,  0}, {0,   0}}, {{0, 0}, {0,  0}}, {{11, 30}, {20, 30}}, {{11, 30}, {19, 0}}, {{12, 0}, {18,  0}}, {{0,  0}, {0,  0}}, {{0,   0}, {0,  0}}}};
const byte dates[9][2] =          {{1, 4},              {15, 4},             {1, 5},            {16, 5},              {1, 8},              {1, 9},              {16, 9},            {1, 10},           {20, 10}};

byte current_times[2][2];

MDNSResponder mdns;
DS_Weather dsw;
ESP8266WebServer server(80);
IRsend irsend(4);
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
ADC_MODE(ADC_VCC);
WiFiUDP Udp;
EspClass ESPm;
unsigned int localPort = 8888;
time_t getNtpTime();

void setup() {    //set everything up
  Serial.begin(115200);
  Serial.println("Booting");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    delay(5000);
    ESP.restart();
  }
  ArduinoOTA.setHostname("rollo-esp8266");
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.onStart([]() {
  });
  ArduinoOTA.onEnd([]() {
     digitalWrite(blue_pin, LOW);
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
     analogWrite(blue_pin, map(progress, 0, total, 0, 255));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    error_notification = true;
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  if(!mdns.begin("rollo", WiFi.localIP())){
    Serial.println("MDNS error :(");
  }
  sensors.begin();
  irsend.begin();
  setSyncInterval(300);
  Udp.begin(localPort);
  setSyncProvider(getNtpTime);
  server.on("/", handleRoot);
  server.on("/log", handleLog);
  server.on("/table", handleTable);
  server.onNotFound(handleNotFound);
  server.begin();
  MDNS.addService("http","tcp",80);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  pinMode(red_pin, OUTPUT);
  pinMode(green_pin, OUTPUT);
  pinMode(blue_pin, OUTPUT);
  pinMode(4, OUTPUT);
  for(int i = 0; i<255; i++){
    analogWrite(red_pin , i);
    analogWrite(green_pin , i);
    analogWrite(blue_pin , i);
    delay(2);
  }
  delay(1000);
  digitalWrite(red_pin, LOW);
  digitalWrite(green_pin, LOW);
  digitalWrite(blue_pin, LOW);
}


//todo: progmem
String prepareRootPage(){     //HTML for the server
  String html_page =
     String("<!DOCTYPE HTML>") +
            "<html>" +
              "<head>" +
                "<title>ESP Rollo</title>" +
                "<meta http-equiv='refresh' content='150'/>" +
              "</head>" +
              "<body>" +
	  						"<CENTER>" +
                	"<h1>ESP Rollo - Server</h1>" +
	  						"</CENTER>" +
                "<p><sup>" + timeAsString() + "</sup></p>" +
                "<p>Roll&aumlden zur Stra&szlige: " + expressState(0) + "</p>" +
                "<p>" + showCurrentTimes(0) + "<p>" +
                "<p>Roll&aumlden zum Garten: " + expressState(2) + "</p>" +
                "<p>" + showCurrentTimes(2) + "<p>" +
                "<p>Stoffroll&aumlden: " + expressState(1) + "</p>" +
                "<p>" + showCurrentTimes(1) + "<p>" +
                "<p>&nbsp;</p>" +
                "<p>Temperatur Dachboden: " + temperatureString(getTemperature()) + "</p>" +
                "<p>&nbsp;</p>" +
                "<p>Wetter laut Darksky</p>" +
                "<p>Niederschlagswahrscheinlichkeit: " + getDarksky(0) + "&#037</p>" +
                "<p>Wolkenbedeckung: " + getDarksky(1) + "&#037</p>" +
                "<p>Luftfeuchtigkeit: " + getDarksky(2) + "&#037</p>" +
              "</body>" +
            "</html>" +
            "\r\n";
    return html_page;
}

String prepareLogPage(){     //HTML for the server
  String html_page =
     String("<!DOCTYPE HTML>") +
            "<html>" +
              "<head>" +
                "<title>ESP Rollo Log</title>" +
                "<meta http-equiv='refresh' content='150'/>" +
              "</head>" +
              "<body>" +
	  						"<CENTER>" +
                	"<h1>ESP Rollo - Log</h1>" +
	  						"</CENTER>" +
                "<p>Alle an die Rollos gesendeten Befehle seit Anfang des Monats.</p>" +
	  						"<p>Wird auch bei Stromverlust, Resets und Abstürzen geleert.</p>" +
              "</body>" +
            "</html>" +
            "\r\n";
    return html_page;
}

String prepareTablePage(){     //HTML for the server
  String html_page =
     String("<!DOCTYPE HTML>") +
            "<html>" +
              "<head>" +
                "<title>ESP Rollo Zeiten und Werte</title>" +
                "<meta http-equiv='refresh' content='150'/>" +
              "</head>" +
              "<body>" +
	  						"<CENTER>" +
                	"<h1>ESP Rollo - Zeitplan</h1>" +
	  						"</CENTER>" +
                "<p>Der Zeitplan sowie die Wetter-Schwellwerte.</p>" +
	  						"<p>Alle Änderungen bleiben auch bei Stromverlust gespeichert.</p>" +
              "</body>" +
            "</html>" +
            "\r\n";
    return html_page;
}

void handleRoot() {
  char temp[400];
  digitalWrite(red_pin, HIGH);
  digitalWrite(blue_pin, HIGH);
  server.send(200, "text/html", prepareRootPage());
  server.send(200, "text/html", temp);
  digitalWrite(red_pin, LOW);
  digitalWrite(blue_pin, LOW);
}

void handleNotFound() {
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";

  for (uint8_t i = 0; i < server.args(); i++) {
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  digitalWrite(red_pin, HIGH);
  digitalWrite(blue_pin, HIGH);
  server.send(404, "text/plain", message);
  digitalWrite(red_pin, LOW);
  digitalWrite(blue_pin, LOW);
}

uint16_t* translate(bool data[], int nr, bool dir) {
  uint16_t *real_data = new uint16_t[48];
  real_data[47] = 1000;
  if (dir == false) {
    data[1] = 1;
    if (nr == 1) {
      data[21] = 1;
      data[23] = 0;
    } else if (nr == 2 || nr == 3) {
      data[21] = 0;
      data[23] = 1;
    } else if (nr == 5 || nr == 6) {
      data[21] = 0;
      data[23] = 0;
    } else {
      data[21] = 1;
      data[23] = 1;
    }
  }
  for (int i = 0; i < 24; i++) {
    if (data[i] == true) {
      real_data[i * 2] = 1300;
      if (i < 23) {
        real_data[(i * 2) + 1] = 400;
      }
    } else {
      real_data[i * 2] = 400;
      if (i < 23) {
        real_data[(i * 2) + 1] = 1300;
      }
    }
  }
  return real_data;
}

void sendIr(uint16_t data[]) {
  digitalWrite(green_pin, HIGH);
  irsend.sendRaw(data , 48, 36);
  digitalWrite(green_pin, LOW);
}

int* getWeather() {
  int *weather = new int[3];
  DSW_current *current = new DSW_current;
  DSW_hourly *hourly = new DSW_hourly;
  DSW_daily *daily = new DSW_daily;
  time_t time;
  digitalWrite(green_pin, HIGH);
  digitalWrite(blue_pin, HIGH);
  dsw.getForecast(current, hourly, daily, api_key, latitude, longitude, units, language);
  digitalWrite(green_pin, LOW);
  digitalWrite(blue_pin, LOW);
  weather[0] = current->precipProbability;
  weather[1] = current->cloudCover;
  weather[2] = current->humidity;
  delete current;
  delete hourly;
  delete daily;
  return weather;
}

String timeAsString(){
  String real_time = String(hour()+1) + ":" + String(minute()) + ":" + String(second());
  return real_time;
}

void moveShutter(int nr, bool dir) {
  uint16_t *arr;
  switch (nr) {
    case 0:
      arr = translate(r1_2, 2, dir);
      sendIr(arr);
      delay(20);
      arr = translate(r1_3, 3, dir);
      sendIr(arr);
      delay(20);
      arr = translate(r1_5, 5, dir);
      sendIr(arr);
      delay(20);
      arr = translate(r1_6, 6, dir);
      sendIr(arr);
    case 1:
      arr = translate(r1_1, 1, dir);
      sendIr(arr);
      delay(20);
      arr = translate(r1_4, 4, dir);
      sendIr(arr);
    case 2:
      arr = translate(r2, BACK_MOST, dir);
      sendIr(arr);
      delay(20);
      arr = translate(r3, BACK_LAST, dir);
      sendIr(arr);
  }
  delete []arr;
}

void controllShutter() {    //also updates global weather array (weather_data)
  int *temp_weather = getWeather();
  weather_data[0] = temp_weather[0];
  weather_data[1] = temp_weather[1];
  weather_data[2] = temp_weather[2];
  delete []temp_weather;
  r_state[1] = scheduleState(1);
  if (weather_data[0] > 20 || weather_data[1] > 60 || weather_data[2] > 90) {
    r_state[1] = true;
  }
  r_state[0] = scheduleState(0);  //decide for normal shutters
  r_state[2] = scheduleState(2);

  for (int i = 0; i < 3; i++) {
    if (r_state[i] != last_state[i] || first_time == true) {
      moveShutter(i, r_state[i]);
      delay(20);
      last_state[i] = r_state[i];
    }
  }
  first_time = false;
}

bool scheduleState(int shutters) {
  int nr = shutters;
  bool result = true;
  if (shutters == 1) {
    nr = 0;
  }
  else if (shutters == 2) {
    nr = 1;
  }
  if (date != -1 && date != 8 && times[nr][date][0][0] != 0 && !(shutters == 0 && date == front_ignore_dates) && timeToMins(hour(), minute()) > current_times[nr][0] && timeToMins(hour(), minute()) < current_times[nr][1]) {
    result = false;
  }
  if (shutters == 0 && (front_lock_dates[0][1] < month() || (front_lock_dates[0][1] == month() && front_lock_dates[0][0] <= day())) && (front_lock_dates[1][1] > month() || (front_lock_dates[1][1] == month() && front_lock_dates[1][0] >= day()))) {
    result = false;
  }
  return result;
}

int getDarksky(int index){
  return weather_data[index];
}

void updateDate(){
   date = -1;
   for (int i = 0; i < 9; i++) {
    if (dates[i][1] < month() || (dates[i][1] == month() && dates[i][0] <= day())) {
      date = i;
    }
   }
}

String showCurrentTimes(int nr){
   int table_nr = 0
      String sentence;
   if (nr != 0) {
     table_nr = nr - 1;
   }
   if (times[table_nr][date][0][0] == 0 || date == -1 || date == 8){
      sentence = "Diese Rollos werden im aktuellen Zeitraum nicht heruntergefahren.";
   else if (nr == 0 && (front_lock_dates[0][1] < month() || (front_lock_dates[0][1] == month() && front_lock_dates[0][0] <= day())) && (front_lock_dates[1][1] > month() || (front_lock_dates[1][1] == month() && front_lock_dates[1][0] >= day()))){
      sentence = "Diese Rollos werden im aktuellen Zeitraum nicht hochgefahren.";
   else if (nr == 1){
      sentence = "Diese Rollos sind heute planmäßig von " + minsToTime(current_times[table_nr][0]) + " bis " + minsToTime(current_times[table_nr][1]) + " unten.";
   else{
      sentence = "Diese Rollos sind heute von " + minsToTime(current_times[table_nr][0]) + " bis " + minsToTime(current_times[table_nr][1]) + " unten.";
   }
}
      
String minsToTime(int _time){
   int _minutes = _time % 60;
   int _hours = (_times - _minutes) / 60;
   String result = "%02d:%02d", _hours, _minutes;
   return result;
}

int timeToMins(byte _hour, byte _minute){
   int minutes_since_0 = _hour * 60;
   minutes_since_0 += _minutes;
   return minutes_since_0;
}

int dailyTime(byte nr, bool direction){
   int time_diff;
   int date_diff;
   int og_date_diff;
   int new_time;
   
   og_date_diff = (dates[date + 1][0] - dates[date][0]) + (dates[date + 1][1] - dates[date][1])*30; //20
   date_diff = dates[date + 1][0] - day();
   date_diff += (dates[date + 1][1] - month()) * 30; //15  80
   time_diff = timeToMins(times[nr][date + 1][direction][0], times[nr][date + 1][direction][1]) - timeToMins(times[nr][date][direction][0], times[nr][date][direction][1]);
   time_diff -= (date_diff) * (time_diff / og_date_diff);
   new_time = timeToMins(times[nr][date][direction][0], times[nr][date][direction][1]) + time_diff;
   return new_time;
}

char* expressState(int index){
  if(r_state[index]){
    return "Hoch";
  }
  else{
    return "Runter";
  }
}

float getTemperature(){
  sensors.requestTemperatures();
  float tempC = sensors.getTempCByIndex(0);
  if(tempC == DEVICE_DISCONNECTED_C){
    error_notification = true;
  }
  return tempC;
}

String temperatureString(float temp){
  String result;
  if(temp == DEVICE_DISCONNECTED_C){
    result = "ERROR: Kein Temperatursensor erkannt";
  }
  else{
    
    result = String(temp) + "&deg;C";
  }
  return result;
}




time_t getNtpTime() {
  IPAddress ntpServerIP; // NTP server's ip address

  while (Udp.parsePacket() > 0) ; // discard any previously received packets
  // get a random server from the pool
  digitalWrite(green_pin, HIGH);
  digitalWrite(blue_pin, HIGH);
  WiFi.hostByName(ntpServerName, ntpServerIP);
  sendNTPpacket(ntpServerIP);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      digitalWrite(green_pin, LOW);
      digitalWrite(blue_pin, LOW);
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  }
  return 0; // return 0 if unable to get the time
}

void sendNTPpacket(IPAddress &address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}

void loop() {
  while (hour() >= 6 && hour() <= 24) {
    updateDate();
    current_times[0][0] = dailyTime(0, false);
    current_times[0][1] = dailyTime(0, true);
    current_times[1][0] = dailyTime(1, false);
    current_times[1][1] = dailyTime(1, true);
    last = millis();
    while (millis() - last < 50){
      ArduinoOTA.handle();
      server.handleClient();
      delay(1);
    }
    if (now() >= last_shutter + 300) {
      last_shutter = now();
      controllShutter();
    }
  }
  delay(1800000); //wait 0.5 hours
}
