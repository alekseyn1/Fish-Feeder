#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <NTPClient.h>
#include <Servo.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <time.h>

ESP8266WiFiMulti wifiMulti;
WiFiClient espClient;
PubSubClient client(espClient);
Servo servo;

time_t now;                         // this are the seconds since Epoch (1970) - UTC
tm tm;                              // the structure tm holds time information in a more convenient way

/* Configuration of NTP */
#define MY_NTP_SERVER "ca.pool.ntp.org"           
#define MY_TZ "PST8PDT,M3.2.0,M11.1.0"   //https://github.com/nayarsystems/posix_tz_db/blob/master/zones.csv

/* Define the WiFi credentials */
#define WIFI_SSID "Skywalker"
#define WIFI_PASSWORD "Luke2024$$$!"
#define WIFI_SSID2 "hassio"
#define WIFI_PASSWORD2 "126651203"

// MQTT Broker
const char *mqtt_broker = "192.168.10.2";
const char *mqtt_username = "admin";
const char *mqtt_password = "Luke2024$$$!";
const char *topic_state = "FishFeeder/state";
const char *topic_cmd = "FishFeeder/set";
const char *topic_avl = "FishFeeder/availability";
const char *topic_state_light = "FishFeeder/light/state";
const char *topic_cmd_light = "FishFeeder/light/set";
const char *topic_avl_light = "FishFeeder/light/availability";
const int mqtt_port = 1883;

//Scheduled times for feeding 24hr format. No leading zeros, i.e. 8:00 and 20:00 are both 8 am and 8 pm feeding times
//sample: const String times[]={"8:0:5","13:0:0","21:0:0"};
const String times[]={"8:0:0","20:0:0"};
const int arrayLength = sizeof(times) / sizeof(times[0]);

const String lights_on = "7"; //in the morning
const String lights_off = "23"; //in the evening

// Relay pin is controlled with D1. The active wire is connected to Normally Open and common
int relay = D1;

// End settings

bool servoState = false;
bool lightState = false;
const bool auto_discovery = true;
String client_id = "esp8266-";
String currentTime = "";
String currentHour = "";

long lastReconnectAttempt = 0;

void setup() {
  Serial.begin(115200);
  delay(1500);
  
  // Pin for relay module set as output
  pinMode(relay, OUTPUT); //OUTPUT_OPEN_DRAIN
  
  configTime(MY_TZ, MY_NTP_SERVER);
  TimeNow();
  Serial.println("Now is " + currentTime);

  client_id += ESP.getChipId();
  Serial.println(client_id);

  wifiMulti.addAP(WIFI_SSID, WIFI_PASSWORD);
  wifiMulti.addAP(WIFI_SSID2, WIFI_PASSWORD2);

  // WiFi.scanNetworks will return the number of networks found
  int n = WiFi.scanNetworks();
  delay(1500);
  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println("Scan done");
  if (n == 0) {
      Serial.println("No networks found");
  } 
  else {
    Serial.print(n);
    Serial.println(" networks found");
    for (int i = 0; i < n; ++i) {
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(WiFi.SSID(i));
      Serial.print(" (");
      Serial.print(WiFi.RSSI(i));
      Serial.print(")");
      Serial.println();
      delay(10);
    }
  }

  Serial.println("Connecting Wifi...");
  if(wifiMulti.run() == WL_CONNECTED) {
    Serial.println("");
    Serial.println("Wi-Fi CONNECTED!");
    Serial.println("IP address: ");
    Serial.print(WiFi.localIP());
    Serial.println("");
  }

  Serial.println("Scheduled feeds start at (24hr format) ");
  for (byte i = 0; i < arrayLength; i++) {
    Serial.println(times[i]);
  }

  //connecting to a mqtt broker
  client.setServer(mqtt_broker, mqtt_port);
  client.setBufferSize(512);
  client.setCallback(callback);
  reconnect();
  setupHaDiscovery();
  lastReconnectAttempt = 0;

}

void TimeNow() {
  time(&now);                       // read the current time
  localtime_r(&now, &tm);           // update the structure tm with the current time
  currentHour = String(tm.tm_hour);
  currentTime = String(tm.tm_hour)+":"+String(tm.tm_min)+":"+String(tm.tm_sec);
}

void setupHaDiscovery () {
  char topic[128];
  char topic1[128];

  strcpy(topic, "homeassistant/switch/");
  strcat(topic, client_id.c_str());
  strcat(topic, "/config");

  strcpy(topic1, "homeassistant/switch/");
  strcat(topic1, client_id.c_str());
  strcat(topic1, "-1/config");

  if (auto_discovery) {
    char buffer[1024]; //2048
    DynamicJsonDocument doc(1024);
    doc.clear();
    doc["name"] = "Fish Feeder";
    doc["unique_id"] = client_id;
    doc["state_topic"] = topic_state;
    doc["command_topic"] = topic_cmd;
    doc["availability_topic"] = topic_avl;
    doc["payload_on"] = "on";
    doc["payload_off"] = "off";
    doc["state_on"] = "on";
    doc["state_off"] = "off";
    doc["icon"] = "mdi:fish";
    JsonObject device = doc.createNestedObject("device");
    device["identifiers"] = client_id.c_str();
    //device["name"] = "Fish Feeder";
    device["model"] = "ESP8622";
    device["manufacturer"] = "relit.ca";
    serializeJson(doc, buffer);
    serializeJson(doc, Serial);
    client.publish(topic, buffer, true);
    delay(1000);
    client.publish(topic_cmd, "off");
    client.publish(topic_state, "off");
    client.publish(topic_avl, "online");
    delay(1000);

    
    char buffer1[1024]="";
    doc.clear();
    doc["name"] = "Fish Feeder Light";
    doc["unique_id"] = client_id+"_light";
    doc["state_topic"] = topic_state_light;
    doc["command_topic"] = topic_cmd_light;
    doc["availability_topic"] = topic_avl_light;
    doc["payload_on"] = "on";
    doc["payload_off"] = "off";
    doc["state_on"] = "on";
    doc["state_off"] = "off";
    doc["icon"] = "mdi:wall-sconce-flat";
    JsonObject device1 = doc.createNestedObject("device");
    device1["identifiers"] = client_id.c_str();
    //device1["name"] = "Fish Feeder";
    device1["model"] = "ESP8622";
    device1["manufacturer"] = "relit.ca";
    serializeJson(doc, buffer1);
    serializeJson(doc, Serial);
    client.publish(topic1, buffer1, true);
    delay(1000);
    client.publish(topic_cmd_light, "off");
    client.publish(topic_state_light, "off");
    client.publish(topic_avl_light, "online");    
  } else {
    //remove all entities by publishing empty payloads
    client.publish(topic, "");
    client.publish(topic1, "");
  }
}

boolean reconnect() {
      Serial.println("");
      Serial.println("Now is " + currentTime);
      Serial.printf("The client %s connects to the mqtt broker\n", client_id.c_str());
      if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
        Serial.println("Broker connected");
        // publish and subscribe
        client.publish(topic_avl, "online"); //servo
        client.subscribe(topic_cmd);
        client.publish(topic_avl_light, "online"); //led
        client.subscribe(topic_cmd_light);
      } else {
          Serial.print("failed with state ");
          Serial.print(client.state());
          delay(2000);
      }
  return client.connected();
}


void callback(char *topic_, byte *payload, unsigned int length) {
    Serial.print("Message arrived in topic: ");
    Serial.println(topic_);
    Serial.print("Message: ");
    
    String message;

    for (int i = 0; i < length; i++) {
        message += (char) payload[i];  // Convert *byte to string
    }
    Serial.print(message);

    if (strcmp(topic_,topic_cmd)==0){
      if (message == "on" && !servoState) {
          feeding(); 
          servoState = true;
      }
      if (message == "off" && servoState) {
          servoState = false;
      }
    }

    if (strcmp(topic_,topic_cmd_light)==0){
      if (message == "on" && !lightState) {
        digitalWrite(relay, HIGH); //lights on
        client.publish(topic_state_light, "on");
        client.publish(topic_cmd_light, "on");
        lightState = true;
      }
      if (message == "off"  && lightState) {
        digitalWrite(relay, LOW); //lights off
        client.publish(topic_state_light, "off");
        client.publish(topic_cmd_light, "off");
        lightState = false;
      }
    }
    Serial.println();
    Serial.println("-------- end of callback ---------------");
}

//Writes a value in microseconds (uS) to the servo, controlling the shaft accordingly. 
//On a standard servo, this will set the angle of the shaft. 
//On standard servos a parameter value of 1000 is fully counter-clockwise, 2000 is fully clockwise, and 1500 is in the middle.

void feeding() {
      client.publish(topic_state, "on");
      //String currentTime = String(tm.tm_hour)+":"+String(tm.tm_min)+":"+String(tm.tm_sec);
      Serial.println();
      Serial.println("Scheduled feeds start at (24hr format) ");
      for (byte i = 0; i < arrayLength; i++) {
        Serial.println(times[i]);
      }
      Serial.println();
      servo.attach(D3); // Pin attached to D3
      delay(1000);
      //servo.writeMicroseconds(1000); // rotate clockwise
      servo.write(0); 
      delay(1000); //wait for one second so all food can come out from the feeder
      //servo.writeMicroseconds(1500); // go to middle position
      servo.write(90);  // set servo to mid-point
      delay(1000);
      servo.detach(); //this is done to prevent servo noise. this disconnects the servo.
      Serial.println("Success. Fed at " + currentTime);
      Serial.println();
      Serial.println("Servo Disabled. Waiting for the next scheduled feed ");
      delay(2000); //wait for one minute, milliseconds
      client.publish(topic_state, "off");
      client.publish(topic_cmd, "off");
}

// timer_state function determines whether the current time in minutes past midnight 
// is within the timed interval start and start+duration, also in minutes past midnight
// range may span midnight.  duration must be less than one day (1440).
byte timer_state(unsigned int start, unsigned int duration, unsigned int now) {
  unsigned int time_on = (now - start + 2880) % 1440;  //multiply minutes per day by two for safety
  if (time_on < duration) return 1;  //within interval
  return 0;  //not within interval
}

void loop() {

  TimeNow();
  
  //Serial.println("Current Time");
  //Serial.println(currentTime);

  //Serial.println("Current Hour");
  //Serial.println(currentHour);

  //Serial.println("Now:");
  //Serial.println(now);

  //Serial.println("Time On");
  //Serial.println((now - (22*60+58) + 2880) % 1440);

  //Serial.println(timer_state((22*60+58), 60, now));

  //Serial.println(lights_on.toInt() < currentHour.toInt());
  //Serial.println(currentHour.toInt() < lights_off.toInt());
  
  //Serial.println(lights_on.toInt());
  //Serial.println(currentHour.toInt());
  //Serial.println(lights_off.toInt());

  if ( lights_on.toInt() < currentHour.toInt() && currentHour.toInt() < lights_off.toInt() ) {
    //digitalWrite(relay, HIGH); //lights on
    Serial.println("Relay ON");
    //client.publish(topic_state_light, "on");
    //client.publish(topic_cmd_light, "on");
  } else {
    //digitalWrite(relay, LOW); //lights off
    Serial.println("Relay OFF");
    //client.publish(topic_state_light, "off");
    //client.publish(topic_cmd_light, "off");
  }

  for (byte i = 0; i < arrayLength; i++) {
    if (times[i]==currentTime) {
      feeding();
    }
  }

  if (!client.connected()) {
    long now = millis();
    if (now - lastReconnectAttempt > 360000) { //attempt to reconnect once in hour
      lastReconnectAttempt = now;
      // Attempt to reconnect
      if (reconnect()) {
        lastReconnectAttempt = 0;
        client.publish(topic_avl, "online");
      }
    }
  } 
  else 
  {
    // Client connected
    client.loop();
    //Serial.println("now: " + currentTime);
    delay(1000);
  }
}