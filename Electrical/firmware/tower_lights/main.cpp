#include <Arduino.h>
#include "driver/twai.h"
#include <Adafruit_NeoPixel.h>

#define CAN_TX_PIN GPIO_NUM_5
#define CAN_RX_PIN GPIO_NUM_4
#define Light5V GPIO_NUM_19
#define Light24V GPIO_NUM_21
#define LED_PIN 33
#define NUM_LEDS 18

Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

bool light24VState = false; // tracks current state

struct CANMessage {
    uint32_t id;
    uint32_t node_id;
    uint32_t cmd_id;
    uint8_t len;
    uint8_t data[8];
};

bool sendCANMessage(uint32_t id, uint8_t len, uint8_t* data);
uint32_t can_make_id(uint32_t node_id, uint32_t cmd_id);
bool receiveCANMessage(CANMessage &msg);
void CANCommand(CANMessage &msg);
void toggle24VLight(CANMessage &msg);
void flashGreenNonBlocking();
void showRed();
void showBlue();
void showWhite();
void printCANMessage(CANMessage &msg);

bool greenFlashEnabled = false;
bool greenOn = false;       // tracks current on/off phase of the flash
unsigned long lastToggle = 0;
const unsigned long flashInterval = 500; // ms

uint32_t ESP_CAN_ID = 12;

void setup() {
  Serial.begin(115200);
  delay(100);
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS(); 
  twai_filter_config_t f_config = {
    .acceptance_code = (ESP_CAN_ID << 26),
    .acceptance_mask = (1UL << 26) - 1,
    .single_filter = true
  };
    
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
      twai_start();
      Serial.println(">>> CAN Bus Online (1Mbps) <<<");
  }

  pinMode(Light24V, OUTPUT);
  digitalWrite(Light24V, HIGH);
  pinMode(Light5V, OUTPUT);
  digitalWrite(Light5V, HIGH);
  light24VState = false;
  greenFlashEnabled = false;
  
  strip.begin();
  strip.setBrightness(64);  // 0-255
  strip.show();  // Initialize all pixels to 'off'
}

void loop() {
  CANMessage msg;
  if(receiveCANMessage(msg)){
    printCANMessage(msg);
    CANCommand(msg);
  }
  if(greenFlashEnabled){
    flashGreenNonBlocking();
  }
  delay(50);
}

void CANCommand(CANMessage &msg) {
  if(msg.node_id == ESP_CAN_ID){
    if(msg.cmd_id == 1){
      toggle24VLight(msg);
    }
    if(msg.cmd_id == 2){
      showRed();
      greenFlashEnabled = false;
    }
    if(msg.cmd_id == 3){
      showBlue();
      greenFlashEnabled = false;
    }
    if(msg.cmd_id == 4){
      digitalWrite(Light5V, LOW);
      delay(100);
      greenFlashEnabled = true;
    }
    if(msg.cmd_id == 5){
      showWhite();
      greenFlashEnabled = false;
    }
    if(msg.cmd_id == 6){
      digitalWrite(Light5V, HIGH);
      delay(100);
      strip.clear();
      strip.show();
      greenFlashEnabled = false;
    }
  }
}

bool sendCANMessage(uint32_t id, uint8_t len, uint8_t* data) {
  twai_message_t message;
  message.identifier = id;
  message.extd = 0;
  message.data_length_code = len;
  for (int i = 0; i < len; i++) message.data[i] = data[i];

  esp_err_t res = twai_transmit(&message, pdMS_TO_TICKS(10));
  if (res == ESP_ERR_INVALID_STATE) {
      twai_stop();
      twai_start();
      return false;
  }
  return (res == ESP_OK);
}

uint32_t can_make_id(uint32_t node_id, uint32_t cmd_id) {
  return (node_id << 5) | cmd_id;
}

bool receiveCANMessage(CANMessage &msg) {
  twai_message_t rx_msg;
    
  // Check for message with 0 timeout (non-blocking)
  esp_err_t result = twai_receive(&rx_msg, 0);
    
  if (result == ESP_OK) {
    msg.id = rx_msg.identifier;
    msg.node_id = rx_msg.identifier >> 5;  // ODrive Node ID
    msg.cmd_id = rx_msg.identifier & 0x1F; // ODrive Command ID
    msg.len = rx_msg.data_length_code;
    memcpy(msg.data, rx_msg.data, rx_msg.data_length_code);
    return true;
  }
  return false;
}

void toggle24VLight(CANMessage &msg) {
  if (msg.data[0] == false) {
    digitalWrite(Light24V, HIGH);  // turn off
    light24VState = false;
  } else if(msg.data[0] == true){
    digitalWrite(Light24V, LOW); // turn on
    light24VState = true;
  }
}

void showRed() {
  digitalWrite(Light5V, LOW);
  delay(100);
  strip.fill(strip.Color(255, 0, 0), 0, NUM_LEDS);
  strip.show();
}

void showBlue() {
  digitalWrite(Light5V, LOW);
  delay(100);
  strip.fill(strip.Color(0, 0, 255), 0, NUM_LEDS);
  strip.show();
}

void showWhite() {
  digitalWrite(Light5V, LOW);
  delay(100);
  strip.fill(strip.Color(255, 255, 255), 0, NUM_LEDS);
  strip.show();
}

void flashGreenNonBlocking() {
  unsigned long now = millis();
  if (now - lastToggle >= flashInterval) {
    lastToggle = now;
    greenOn = !greenOn;
    if (greenOn) {
      strip.fill(strip.Color(0, 255, 0), 0, NUM_LEDS);
    } else {
      strip.clear();
    }
    strip.show();
  }
}

void printCANMessage(CANMessage &msg) {
  Serial.print("ID: 0x");
  Serial.print(msg.id, HEX);
  Serial.print("  Node: ");
  Serial.print(msg.node_id);
  Serial.print("  Cmd: ");
  Serial.print(msg.cmd_id);
  Serial.print("  Len: ");
  Serial.print(msg.len);
  Serial.print("  Data: ");
  for (int i = 0; i < msg.len; i++) {
    if (msg.data[i] < 0x10) Serial.print("0"); // leading zero for alignment
    Serial.print(msg.data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}
