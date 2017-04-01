#include "Wire.h"

#include <EEPROM.h>

#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#include <Adafruit_MCP9808.h>
#include <Adafruit_NeoPixel.h>

Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();

/***********************
 *                     *
 * Status LED handling *
 *                     *
 ***********************/

// PIN of the Status Pixel
#define STATUS_PIN 2
Adafruit_NeoPixel statusLED = Adafruit_NeoPixel(2, STATUS_PIN, NEO_GRB + NEO_KHZ800);

// Signalling colors for the status LED (WS2812)
// 00RRGGBB
const uint32_t status_connecting = 0x00664411;
const uint32_t status_connected  = 0x00006611;
const uint32_t status_logging    = 0x00002266;
const uint32_t status_error      = 0x00661100;

void setStatusLEDs(uint32_t mcpColor, uint32_t wifiColor) {
  statusLED.setPixelColor(0, mcpColor);
  statusLED.setPixelColor(1, wifiColor);
  statusLED.show();
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
// MQTT server name or IP (128 bytes)
#define EEPROM_OFFSET_MQTT_SERVER     (EEPROM_OFFSET_WIFI_PASS + EEPROM_LENGTH_WIFI_PASS)
#define EEPROM_LENGTH_MQTT_SERVER     128
// MQTT Event channel (128 bytes)
#define EEPROM_OFFSET_MQTT_EVENT      (EEPROM_OFFSET_MQTT_SERVER + EEPROM_LENGTH_MQTT_SERVER)
#define EEPROM_LENGTH_MQTT_EVENT      128
// MQTT Status channel (128 bytes)
#define EEPROM_OFFSET_MQTT_STATE      (EEPROM_OFFSET_MQTT_EVENT + EEPROM_LENGTH_MQTT_EVENT)
#define EEPROM_LENGTH_MQTT_STATE      128


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

String eeprom_read_mqtt_server() {
  return read_from_eeprom(EEPROM_OFFSET_MQTT_SERVER, EEPROM_LENGTH_MQTT_SERVER);
}

void eeprom_update_mqtt_server(String server) {
  update_eeprom(EEPROM_OFFSET_MQTT_SERVER, EEPROM_LENGTH_MQTT_SERVER, server);
}

String eeprom_read_topic_event() {
  return read_from_eeprom(EEPROM_OFFSET_MQTT_EVENT, EEPROM_LENGTH_MQTT_EVENT);
}

void eeprom_update_topic_event(String event) {
  update_eeprom(EEPROM_OFFSET_MQTT_EVENT, EEPROM_LENGTH_MQTT_EVENT, event);
}

String eeprom_read_topic_state() {
  return read_from_eeprom(EEPROM_OFFSET_MQTT_STATE, EEPROM_LENGTH_MQTT_STATE);
}

void eeprom_update_topic_state(String state) {
  update_eeprom(EEPROM_OFFSET_MQTT_STATE, EEPROM_LENGTH_MQTT_STATE, state);
}

void setup_config() {
  EEPROM.begin(512);
  
  if (!eeprom_check_magic())
    return;

  //is_config_mode = true;
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
    if (!mqtt_client.publish(eeprom_read_topic_event().c_str(), event))
      Serial.println("MQTT publish failed!");
  }
}

void mqtt_publish_temperature(float temperature) {
  if (mqtt_client.connected()) {
    String payload = "";
    payload += temperature;
    payload += " C";
    if (!mqtt_client.publish(eeprom_read_topic_state().c_str(), (char*)payload.c_str()))
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
    eeprom_update_wifi_essid(remain);
    uart_print_wifi_info();
  } else if (cmd.equals("password")) {
    eeprom_update_wifi_password(remain);
    uart_print_wifi_info();
  } else
    Serial.println("Unknown sub-command!");
}

void uart_print_mqtt_info() {
  if (is_config_mode)
    Serial.println("Currently in config mode, no reliable MQTT information available!");

  Serial.print("MQTT Server: ");
  Serial.println(eeprom_read_mqtt_server());
}

void uart_handle_mqtt(String cmd, String remain) {
  cmd.toLowerCase();

  if (cmd.equals("info"))
    uart_print_mqtt_info();
  else if (cmd.equals("server")) {
    eeprom_update_mqtt_server(remain);
    uart_print_mqtt_info();
  } else
    Serial.println("Unknown sub-command!");
}

void uart_print_topic_info() {
  if (is_config_mode)
    Serial.println("Currently in config mode, no reliable MQTT topic information available!");

  Serial.print("Event topic: ");
  Serial.println(eeprom_read_topic_event());
  Serial.print("State topic: ");
  Serial.println(eeprom_read_topic_state());
}

void uart_handle_topic(String cmd, String remain) {
  cmd.toLowerCase();

  if (cmd.equals("info"))
    uart_print_topic_info();
  else if (cmd.equals("event")) {
    eeprom_update_topic_event(remain);
    uart_print_topic_info();
  } else if (cmd.equals("state")) {
    eeprom_update_topic_state(remain);
    uart_print_topic_info();
  } else
    Serial.println("Unknown sub-command!");
}

void uart_dispatch_input(String topic, String cmd, String remain) {
  if (topic.equals("wifi"))
    uart_handle_wifi(cmd, remain);
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

  String server = eeprom_read_mqtt_server();
  Serial.println("Connecting to MQTT broker " + server);

  mqtt_client.setServer(server.c_str(), 1883);
  mqtt_client.setCallback(mqtt_callback);
  mqtt_client.setClient(wifiClient);

  if (mqtt_client.connect((char*) clientName.c_str())) {
    Serial.print("Event Topic is: ");
    Serial.println(eeprom_read_topic_event());
    Serial.print("State Topic is: ");
    Serial.println(eeprom_read_topic_state());
  } else {
    Serial.println("MQTT connect error, activating config mode.");
    is_config_mode = true;
    //Serial.println("MQTT connect error, reboot.");
    //abort();
  }
}

void setup_temp_sensor() {
  // Make sure the sensor is found, you can also pass in a different i2c
  // address with tempsensor.begin(0x18) for example
  Serial.println("Temperature Sensor initialization.");
  if (!tempsensor.begin(0x18)) {
    Serial.println("Couldn't find MCP9808!");
    setStatusLEDs(status_error, status_connecting);
    mqtt_publish_event("Couldn't find MCP9808!");
    while (1) delay(10000);
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
  statusLED.begin();
  setStatusLEDs(status_connecting, status_connecting);

  // TWI initialization
  Wire.begin(4, 5); // SDA, SCL


  String clientName = clientname();
  Serial.print("Client name ");
  Serial.println(clientName);

  if (!is_config_mode)
    setup_wifi();

  if (!is_config_mode)
    setup_mqtt();

  if (!is_config_mode) {
    setStatusLEDs(status_connecting, status_connected);
  
    setup_temp_sensor();
  
    setStatusLEDs(status_connected, status_connected);
    
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
    setStatusLEDs(status_connected, status_error);

    is_config_mode = true;
    //abort();
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

#define SENSOR_WAIT_CYCLES 100
int sensor_cycles = 0;

void loop() {
  // always handle uart
  loop_handle_uart();


  // the rest: only when not in config mode

  if (!is_config_mode && !sensor_cycles--) {
    loop_check_connection();
  
  
    setStatusLEDs(status_logging, status_connected);
  
    float temperature = read_temperature();
  
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" C");
  
    mqtt_publish_temperature(temperature);
  
    setStatusLEDs(status_connected, status_connected);
  
    mqtt_client.loop();

    sensor_cycles = SENSOR_WAIT_CYCLES;
  } // if !is_config_mode

  // Wait before next loop
  delay(50);
}


