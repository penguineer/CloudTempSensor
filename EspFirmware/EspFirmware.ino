#include "Wire.h"

#include <EEPROM.h>

#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#include <Adafruit_MCP9808.h>
#include <Adafruit_NeoPixel.h>

#include <ESP8266HTTPClient.h>

#include <ArduinoJson.h>


/**********************
 *                    *
 * State LED handling *
 *                    *
 **********************/

// PIN of the State LED
#define STATUS_PIN 2
Adafruit_NeoPixel stateLED = Adafruit_NeoPixel(2, STATUS_PIN, NEO_GRB + NEO_KHZ800);

// Signalling colors for the status LED (WS2812)
// 00RRGGBB
const uint32_t state_connecting = 0x00664411;
const uint32_t state_connected  = 0x00006611;
const uint32_t state_logging    = 0x00002266;
const uint32_t state_error      = 0x00661100;

void setStateLED(uint32_t stateColor) {
  stateLED.setPixelColor(0, stateColor);
  stateLED.show();
}

/*************************************
 *                                   *
 * Configuration handling and EEPROM *
 *                                   *
 *************************************/

// Determine if we are in config mode, i.e. WiFi, MQTT and Temp Sensor are blocked not used
bool is_config_mode = true;

// Magic Word (6 bytes)
#define EEPROM_OFFSET_MAGIC           0
#define EEPROM_LENGTH_MAGIC           6
// Wifi ESSID (64 bytes)
#define EEPROM_OFFSET_WIFI_ESSID      (EEPROM_OFFSET_MAGIC + EEPROM_LENGTH_MAGIC)
#define EEPROM_LENGTH_WIFI_ESSID      64
// Wifi password (64 bytes)
#define EEPROM_OFFSET_WIFI_PASS       (EEPROM_OFFSET_WIFI_ESSID + EEPROM_LENGTH_WIFI_ESSID)
#define EEPROM_LENGTH_WIFI_PASS       64
// HTTP Config URL (256 bytes)
#define EEPROM_OFFSET_HTTP_URL        (EEPROM_OFFSET_WIFI_PASS + EEPROM_LENGTH_WIFI_PASS)
#define EEPROM_LENGTH_HTTP_URL        256
// HTTP User (32 bytes)
#define EEPROM_OFFSET_HTTP_USER       (EEPROM_OFFSET_HTTP_URL + EEPROM_LENGTH_HTTP_URL)
#define EEPROM_LENGTH_HTTP_USER       32
// HTTP Pass (32 bytes)
#define EEPROM_OFFSET_HTTP_PASS       (EEPROM_OFFSET_HTTP_USER + EEPROM_LENGTH_HTTP_USER)
#define EEPROM_LENGTH_HTTP_PASS       32


String read_from_eeprom(int offset, int length) {
  String result = "";

  char mem = 1; // anything other than 0
  int ofs = offset;
  while (mem && (ofs < offset + length)) {
    int addr = ofs++;
    mem = EEPROM.read(addr);
    if (mem)
      result += mem;
  }
  
  return result;
}

void update_eeprom(int offset, int length, String value) {
  for (int i = 0; i < length; i++) {
    int addr = offset + i;
    int mem = EEPROM.read(addr);

    int val = 0;
    if (i < value.length())
      val = value.charAt(i);

    if (val != mem)
      EEPROM.write(addr, val);
  }
  EEPROM.commit();
}

/*
 * Check if magic word is in EEPROM
 */
bool eeprom_check_magic() {
  // Check if we may have meaningful data in EEPROM
  // Read Magic word "TMPSNS" from offset 0
  String magic = read_from_eeprom(EEPROM_OFFSET_MAGIC, EEPROM_LENGTH_MAGIC);

  // If unsuccessful, go to config mode and exit
  if (!magic.equals("TMPSNS")) {
    is_config_mode = true;
    Serial.println("Magic word TMPSNS not found in EEPROM, going into config mode.");
    return false;
  }  else
    is_config_mode = false;

  return true;
}

/*
 * Write magic word to EEPROM
 */
void eeprom_set_magic() {
  update_eeprom(EEPROM_OFFSET_MAGIC, EEPROM_LENGTH_MAGIC, "TMPSNS");
}

String eeprom_read_wifi_essid() {
  return read_from_eeprom(EEPROM_OFFSET_WIFI_ESSID, EEPROM_LENGTH_WIFI_ESSID);
}

void eeprom_update_wifi_essid(String essid) {
  update_eeprom(EEPROM_OFFSET_WIFI_ESSID, EEPROM_LENGTH_WIFI_ESSID, essid);
}

String eeprom_read_wifi_password() {
  return read_from_eeprom(EEPROM_OFFSET_WIFI_PASS, EEPROM_LENGTH_WIFI_PASS);
}

void eeprom_update_wifi_password(String password) {
  update_eeprom(EEPROM_OFFSET_WIFI_PASS, EEPROM_LENGTH_WIFI_PASS, password);
}

String eeprom_read_http_url() {
  return read_from_eeprom(EEPROM_OFFSET_HTTP_URL, EEPROM_LENGTH_HTTP_URL);
}

void eeprom_update_http_url(String url) {
  update_eeprom(EEPROM_OFFSET_HTTP_URL, EEPROM_LENGTH_HTTP_URL, url);
}

String eeprom_read_http_user() {
  return read_from_eeprom(EEPROM_OFFSET_HTTP_USER, EEPROM_LENGTH_HTTP_USER);
}

void eeprom_update_http_user(String user) {
  update_eeprom(EEPROM_OFFSET_HTTP_USER, EEPROM_LENGTH_HTTP_USER, user);
}

String eeprom_read_http_pass() {
  return read_from_eeprom(EEPROM_OFFSET_HTTP_PASS, EEPROM_LENGTH_HTTP_PASS);
}

void eeprom_update_http_pass(String pass) {
  update_eeprom(EEPROM_OFFSET_HTTP_PASS, EEPROM_LENGTH_HTTP_PASS, pass);
}

void setup_config() {
  EEPROM.begin(512);

  if (!eeprom_check_magic())
    return;

  //is_config_mode = true;
}

/*****************************
 *                           *
 * Web-based config handling *
 *                           *
 *****************************/

// Example JSON file
/*

{
  "name" : "TmpSnsr",

  "update" : {
    "server" : "updateserver",
    "port"   : "80",
    "url"   : "/firmware/ESP-MAC"
  },

  "mqtt" : {
    "server" : "mqttserver",
    "event"  : "/MyHome/Things/TempSensor/1/Events",
    "state"  : "/MyHome/Things/TempSensor/1/State"
  }
}

 */

static String config_mqtt_server;
static String config_topic_event;
static String config_topic_state;

String config_val_mqtt_server() {
  return config_mqtt_server;
}

String config_val_topic_event() {
  return config_topic_event;
}

String config_val_topic_state() {
  return config_topic_state;
}

String configName() {
  String configName;
  configName += "ESP";
  
  uint8_t mac[6];
  WiFi.macAddress(mac);
  String macStr;
  for (int i = 0; i < 6; ++i) {
    macStr += "-";
    if (mac[i] < 0x10)
      macStr += "0";
    macStr += String(mac[i], 16);
  }
  macStr.toUpperCase();

  configName += macStr;

  return configName;
}

bool http_config_process(String cfg_json) {
    DynamicJsonBuffer  jsonBuffer;
    JsonObject& root = jsonBuffer.parseObject(cfg_json);

     if (!root.success()) {
      Serial.println("JSON parsing failed!");
      return false;
    }

    if (root.containsKey("mqtt")) {
      JsonObject& json_mqtt = root["mqtt"];

      if (json_mqtt.containsKey("server")) {
        config_mqtt_server = (const char*)json_mqtt["server"];

        Serial.println("MQTT server: " + config_mqtt_server);
      }

      if (json_mqtt.containsKey("event")) {
        config_topic_event = (const char*)json_mqtt["event"];

        Serial.println("Event topic: " + config_topic_event);
      }

      if (json_mqtt.containsKey("state")) {
        config_topic_state = (const char*)json_mqtt["state"];

        Serial.println("State topic: " + config_topic_state);
      }
    }


    return true;
}

bool http_config_load() {
  bool success = true;

  String url = eeprom_read_http_url();

  Serial.print("Loading web config from ");
  Serial.println(url);

  HTTPClient http;

  String user = eeprom_read_http_user();
  String pass = eeprom_read_http_pass();

  if ( (user.length() != 0) && (pass.length() != 0))
    http.setAuthorization(user.c_str(), pass.c_str());

  http.begin(url);

  int httpCode = http.GET();
  if (httpCode == HTTP_CODE_OK) {
    String cfg_json = http.getString();
    Serial.println("Web config:");
    Serial.println(cfg_json);

    http_config_process(cfg_json);

  } else {
    Serial.println("Error in HTTP request: " + HTTPClient::errorToString(httpCode));
    success = false;
  }

  http.end();

  return success;

}


/*
   Device State Machine
*/
// TODO  Decide if
//      a) one flat state machine
//      b) one hierarchical state machine
//      c) multiple state machine with subsumption artitecture

// * States *
/*
  /// Device is in initialization mode
  const uint8_t SETUP   = 1;
  /// Device is idle, waiting for event
  const uint8_t IDLE    = 2;
  /// Measurement is taking place
  const uint8_t MEASURE = 3;
  /// General error
  const uint8_t ERROR = 100;
  /// Temperature sensor cannot be found
  const uint8_t ERROR_NO_SENSOR = ERROR + 1;
  /// WiFi connection failed
  const uint8_t ERROR_WIFI   = ERROR + 2;
  /// Cloud connection failed
  const uint8_t ERROR_CLOUD  = ERROR + 3;
*/

/******************
 *                *
 * MQTT functions *
 *                *
 ******************/

WiFiClient wifiClient;
PubSubClient mqtt_client;

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  // handle message arrived
}

void mqtt_publish_event(const char* event) {
  if (mqtt_client.connected() && event) {
    if (!mqtt_client.publish(config_val_topic_event().c_str(), event))
      Serial.println("MQTT publish failed!");
  }
}

void mqtt_publish_temperature(float temperature) {
  if (mqtt_client.connected()) {
    String payload = "";
    payload += temperature;
    payload += " C";
    if (!mqtt_client.publish(config_val_topic_state().c_str(), (char*)payload.c_str()))
      Serial.println("MQTT publish failed!");
  }
}

/****************
 *              *
 * UART Console *
 *              *
 ****************/

void uart_announce() {
  Serial.println();

  if (is_config_mode)
    Serial.println("Config mode is active!");

  Serial.println();
  Serial.println("Type command, \"help\" for a list of available commands.");
}

void uart_print_wifi_info() {
  if (is_config_mode)
    Serial.println("Currently in config mode, no reliable WiFi information available!");
  
  Serial.print("ESSID: ");
  Serial.println(eeprom_read_wifi_essid());
  Serial.print("Password: ");
  Serial.println(eeprom_read_wifi_password());
}

void uart_handle_wifi(String cmd, String remain) {
  cmd.toLowerCase();

  if (cmd.equals("info"))
    uart_print_wifi_info();
  else if (cmd.equals("essid")) {
    eeprom_set_magic();
    eeprom_update_wifi_essid(remain);
    uart_print_wifi_info();
  } else if (cmd.equals("password")) {
    eeprom_set_magic();
    eeprom_update_wifi_password(remain);
    uart_print_wifi_info();
  } else
    Serial.println("Unknown sub-command!");
}

void uart_print_http_info() {
  if (is_config_mode)
    Serial.println("Currently in config mode, no reliable WiFi information available!");

  Serial.print("URL: ");
  Serial.println(eeprom_read_http_url());

  String user = eeprom_read_http_user();
  if (user.length() == 0)
    Serial.println("User is not set.");
  else {
    Serial.print("User: ");
    Serial.println(user);
  }

  String pass = eeprom_read_http_pass();
  if ( pass.length() == 0)
    Serial.println("Password is not set");
  else
    Serial.println("Password is set but will not be printed!");
}

void uart_handle_http(String cmd, String remain) {
  cmd.toLowerCase();

  if (cmd.equals("info"))
    uart_print_http_info();
  else if (cmd.equals("load"))
    http_config_load();
  else if (cmd.equals("url")) {
    eeprom_set_magic();
    eeprom_update_http_url(remain);
    uart_print_http_info();
  } else if (cmd.equals("user")) {
    eeprom_set_magic();
    eeprom_update_http_user(remain);
    uart_print_http_info();
  } else if (cmd.equals("pass")) {
    eeprom_set_magic();
    eeprom_update_http_pass(remain);
    uart_print_http_info();
  } else
    Serial.println("Unknown sub-command!");
}

void uart_print_mqtt_info() {
  if (is_config_mode)
    Serial.println("Currently in config mode, no reliable MQTT information available!");

  Serial.print("MQTT Server: ");
  Serial.println(config_val_mqtt_server());
}

void uart_handle_mqtt(String cmd, String remain) {
  cmd.toLowerCase();

  if (cmd.equals("info"))
    uart_print_mqtt_info();
  else
    Serial.println("Unknown sub-command!");
}

void uart_print_topic_info() {
  if (is_config_mode)
    Serial.println("Currently in config mode, no reliable MQTT topic information available!");

  Serial.print("Event topic: ");
  Serial.println(config_val_topic_event());
  Serial.print("State topic: ");
  Serial.println(config_val_topic_state());
}

void uart_handle_topic(String cmd, String remain) {
  cmd.toLowerCase();

  if (cmd.equals("info"))
    uart_print_topic_info();
  else
    Serial.println("Unknown sub-command!");
}

void uart_dispatch_input(String topic, String cmd, String remain) {
  if (topic.equals("wifi"))
    uart_handle_wifi(cmd, remain);
  else if (topic.equals("http"))
    uart_handle_http(cmd, remain);
  else if (topic.equals("mqtt"))
    uart_handle_mqtt(cmd, remain);
  else if (topic.equals("topic"))
    uart_handle_topic(cmd, remain);
  else
    Serial.println("Unknown command!");
}

void uart_handle_input(String input) {
  // get first keyword delimiter
  int idx = input.indexOf(' ');

  // get the command topic
  String topic;
  if (idx > 0)
    topic = input.substring(0, idx);
  else
    topic = input;
  topic.toLowerCase();

  // get a command, if available
  String cmd = "";
  if (idx > 0) {
    int idx2 = input.indexOf(' ', idx+1);
    
    if (idx2 > 0)
      cmd = input.substring(idx+1, idx2);
    else
      cmd = input.substring(idx+1);

    idx = idx2;
  }

  // extract the remainder
  String remain = "";
  if (idx > 0)
      remain = input.substring(idx+1);

  uart_dispatch_input(topic, cmd, remain);
  uart_announce();
}

void loop_handle_uart() {
  // Check for input on the UART
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    Serial.print("> ");
    Serial.println(input);

    uart_handle_input(input);
  } // Serial available
}

/**********************
 *                    *
 * Temperature Sensor *
 *                    *
 **********************/

Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();

void setup_temp_sensor() {
  Serial.println("Temperature Sensor initialization.");

  // Make sure the sensor is found, you can also pass in a different i2c
  // address with tempsensor.begin(0x18) for example
  if (!tempsensor.begin(0x18)) {
    Serial.println("Couldn't find MCP9808!");
    setStateLED(state_error);
    mqtt_publish_event("Couldn't find MCP9808!");
    while (1) delay(10000);
  }
}

float read_temperature() {
  // Read and print out the temperature, then convert to *C
  tempsensor.shutdown_wake(0);   // Don't remove this line! required before reading temp
  delay(250);              // wait for a quarter second
  float temperature = tempsensor.readTempC();
  tempsensor.shutdown_wake(1);

  return temperature;
}


/*********
 *       *
 * Setup *
 *       *
 *********/

String clientname() {
  String clientName;
  clientName += "ESP-TmpSnsr-";
  
  uint8_t mac[6];
  WiFi.macAddress(mac);
  String macStr;
  for (int i = 3; i < 6; ++i) {
    if (mac[i] < 0x10)
      macStr += "0";
    macStr += String(mac[i], 16);
  }
  macStr.toUpperCase();

  clientName += macStr;

  return clientName;
}

void setup_wifi() {
  String clientName = clientname();

  String essid = eeprom_read_wifi_essid();
  String password = eeprom_read_wifi_password();

  Serial.print("Connecting to ");
  Serial.println(essid);

  WiFi.hostname(clientName);
  WiFi.begin(essid.c_str(), password.c_str());

  int timeout = 20;
  while ((WiFi.status() != WL_CONNECTED) && --timeout) {
    delay(500);
    Serial.print(".");
  }

  if (timeout == 0) {
    Serial.println("Could not connect to Wifi, entering config mode!");
    is_config_mode = true;
  } else {
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  }
}

void setup_mqtt() {
  String clientName = clientname();

  String server = config_val_mqtt_server();
  Serial.print("Connecting to MQTT server " + server);

  mqtt_client.setServer(server.c_str(), 1883);
  mqtt_client.setCallback(mqtt_callback);
  mqtt_client.setClient(wifiClient);

  int timeout = 20;

  while (timeout && !mqtt_client.connected()) {
    Serial.print(".");
    if (!mqtt_client.connect((char*) clientName.c_str())) {
      timeout--;
      delay(500);
    }
  }

  if (mqtt_client.connected()) {
    Serial.println(" Connected.");
    Serial.print("Event Topic is: ");
    Serial.println(config_val_topic_event());
    Serial.print("State Topic is: ");
    Serial.println(config_val_topic_state());
  } else {
    Serial.println("MQTT connect error, entering config mode.");
    is_config_mode = true;
  }
}


void setup() {
  // Start UART console
  Serial.begin(115000);

  Serial.println();
  Serial.println("Setting up...");
  
  // Check config
  setup_config();

  // Status LED initialization
  stateLED.begin();
  setStateLED(state_connecting);

  // TWI initialization
  Wire.begin(4, 5); // SDA, SCL


  String clientName = clientname();
  Serial.print("Client name ");
  Serial.println(clientName);

  if (!is_config_mode)
    setup_wifi();

  if (!is_config_mode)
    http_config_load();

  if (!is_config_mode)
    setup_mqtt();

  if (!is_config_mode) {
    setup_temp_sensor();
    
    mqtt_publish_event("start-up");
  }

  uart_announce();
}

/*************
 *           *
 * Main Loop *
 *           *
 *************/

void loop_check_connection() {
  // Connection Check (indirectly also checks WiFi)
  if (!mqtt_client.connected()) {
    setStateLED(state_error);

    // reboot if not in config mode
    if (!is_config_mode) {
      Serial.println("Lost connection, reboot.");
      abort();
    }
  }
}

#define SENSOR_WAIT_CYCLES 100
int sensor_cycles = 0;

void loop() {
  // always handle uart
  loop_handle_uart();


  // the rest: only when not in config mode

  if (!is_config_mode && !sensor_cycles--) {
    loop_check_connection();
  
  
    setStateLED(state_logging);
  
    float temperature = read_temperature();
  
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" C");
  
    mqtt_publish_temperature(temperature);
  
    setStateLED(state_connected);
  
    mqtt_client.loop();

    sensor_cycles = SENSOR_WAIT_CYCLES;
  } // if !is_config_mode

  // Wait before next loop
  delay(50);
}


