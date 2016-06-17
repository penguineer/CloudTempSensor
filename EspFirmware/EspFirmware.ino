#include "Wire.h"
//#include <Adafruit_Sensor.h>
#include <Adafruit_MCP9808.h>

Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();

#include <Adafruit_NeoPixel.h>
#define WSPIN 13
#define WSLEN 30

Adafruit_NeoPixel strip = Adafruit_NeoPixel(WSLEN, WSPIN, NEO_GRB + NEO_KHZ800);

// 00RRGGBB
uint32_t same_color   = 0x00448800;
uint32_t higher_color = 0x00882200;
uint32_t lower_color  = 0x002200AA;

uint32_t measure_delay = 1000;

void setup() {
  // put your setup code here, to run once:

  // initialize digital pin 13 as an output.
  pinMode(2, OUTPUT);

  Wire.begin(5, 4); // SDA, SCL

  Serial.begin(115000);

  // Make sure the sensor is found, you can also pass in a different i2c
  // address with tempsensor.begin(0x18) for example
  if (!tempsensor.begin(0x1f)) {
    Serial.println("Couldn't find MCP9808!");
    while (1);
  }


//  strip.begin();
//  strip.show();

  measure_delay = 60000 / WSLEN;
  Serial.print("Measure delay is ");
  Serial.print(measure_delay);
  Serial.println("ms");
}

float prev_temperature = 0;
uint8_t pos = 0;

void loop() {
  digitalWrite(2, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(250);              // wait for a quarter second


  //Serial.println("wake up MCP9808.... "); // wake up MSP9808 - power consumption ~200 mikro Ampere


  // Read and print out the temperature, then convert to *F
  tempsensor.shutdown_wake(0);   // Don't remove this line! required before reading temp
  delay(250);              // wait for a quarter second
  float temperature = tempsensor.readTempC();
  delay(250);              // wait for a quarter second
  tempsensor.shutdown_wake(1);   // Don't remove this line! required before reading temp

  //Serial.println("Shutdown MCP9808.... ");
  //tempsensor.shutdown_wake(1); // shutdown MSP9808 - power consumption ~0.1 mikro Ampere

  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" C");

/*
  uint32_t color = same_color;
  if (temperature - prev_temperature < -0.05)
    color = lower_color;
  if (temperature - prev_temperature >  0.05)
    color = higher_color;
  prev_temperature = temperature;

  strip.setPixelColor(pos, color);
  if (pos-- == 0) pos = WSLEN - 1;

  strip.show();
*/
  digitalWrite(2, LOW);    // turn the LED off by making the voltage LOW
  delay(measure_delay - 250);      // wait for delay to end
}


