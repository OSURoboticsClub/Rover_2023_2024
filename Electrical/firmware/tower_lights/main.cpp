#include <Arduino.h>
#include "driver/twai.h"

#define CAN_TX_PIN GPIO_NUM_5
#define CAN_RX_PIN GPIO_NUM_4
#define Light5V GPIO_NUM_19
#define Light24V GPIO_NUM_21

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
void toggle24VLight();

uint32_t ESP_CAN_ID = 12;

void setup() {
  Serial.begin(115200);
  delay(100);
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS(); 
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
      twai_start();
      Serial.println(">>> CAN Bus Online (1Mbps) <<<");
  }

  pinMode(Light24V, OUTPUT);
  digitalWrite(Light24V, HIGH);
  light24VState = false;
}

void loop() {
  CANMessage msg;
  if(receiveCANMessage(msg)){
    CANCommand(msg);
  }
  delay(100);
}

void CANCommand(CANMessage &msg){
  if(msg.node_id == ESP_CAN_ID){
    if(msg.cmd_id == 1){
      toggle24VLight();
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

void toggle24VLight(){
  if (light24VState) {
    digitalWrite(Light24V, HIGH);  // turn off
    light24VState = false;
  } else {
    digitalWrite(Light24V, LOW); // turn on
    light24VState = true;
  }
}
