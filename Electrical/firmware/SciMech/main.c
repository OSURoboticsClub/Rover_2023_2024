#include <Arduino.h>
#include <ESP32Servo.h>
#include "driver/twai.h"

#define CAN_TX_PIN GPIO_NUM_5
#define CAN_RX_PIN GPIO_NUM_4

//Servo Motor Code
Servo myServo;
int servoPin = 22;

struct CANMessage {
    uint32_t id;
    uint32_t node_id;
    uint32_t cmd_id;
    uint8_t len;
    uint8_t data[8];
};

bool sendCANMessage(uint32_t id, uint8_t len, uint8_t* data);
bool receiveCANMessage(CANMessage &cmsg);


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

    // Necessary for ESP32 to manage PWM timers properly
    ESP32PWM::allocateTimer(0);
    
    myServo.setPeriodHertz(50);           // Standard 50Hz frequency
    myServo.attach(servoPin, 500, 2400);  // Pin 22, min pulse, max pulse
}

void loop() {
    CANMessage msg;

    if(receiveCANMessage(msg)){ //check for messages
        if (msg.node_id == 1){
            //esp is node 1
            if(msg.cmd_id == 1){
                //turning servo is command id 1
                if(msg.data[0]){ 
                    //data bit 0 is a boolean, true = 50 degrees, false = 0 degrees
                    myServo.write(50);
                    delay(1000);
                }
                else{
                    myServo.write(0);
                    delay(1000);
                }
            }
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


/*
//ESC motor Code
const int escPin = 22;
Servo myESC;

void setup() {
    Serial.begin(115200);

    // Prepare PWM Timers
    ESP32PWM::allocateTimer(0);
    myESC.setPeriodHertz(50); // ESCs MUST have 50Hz

    // Attach the ESC
    // Most ESCs use 1000us for STOP and 2000us for FULL THROTTLE
    myESC.attach(escPin, 1000, 2000);

    // --- ESC ARMING SEQUENCE ---
    Serial.println("Arming ESC... Sending Stop Signal");
    myESC.write(0);  // Send "Zero" throttle
    delay(5000);     // Wait 5 seconds for the ESC to beep and arm
    Serial.println("ESC should be armed now.");
}

void loop() {
    // Slowly speed up
    Serial.println("Increasing Throttle...");
    for (int speed = 0; speed <= 15; speed++) { // Stay low (30%) for safety
        myESC.write(speed);
        delay(100);
    }

    delay(10000);

    // Shut down
    Serial.println("Stopping...");
    myESC.write(0);
    delay(5000);
}
    */
