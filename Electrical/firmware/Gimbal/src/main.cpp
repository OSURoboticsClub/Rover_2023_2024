#include <Arduino.h>
#include "driver/twai.h"
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <cstring>

#define CAN_TX_PIN GPIO_NUM_5
#define CAN_RX_PIN GPIO_NUM_4

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

struct CANMessage {
    uint32_t id;
    uint32_t node_id;
    uint32_t cmd_id;
    uint8_t len;
    uint8_t data[8];
};

bool Toggle_Stabilization = true;
bool Toggle_IMU_Feedback = false;

float pitch_offset = 0.0f;
float yaw_offset = 0.0f;
unsigned long last_update = 0;
const int update_interval = 10;

float camera_pos_yaw = 0;
float camera_pos_roll = 0;
float camera_pos_pitch = 0;

float yaw_target = 0;
float pitch_target = 0;
float roll_target = 0;

bool sendCANMessage(uint32_t id, uint8_t len, uint8_t* data);
void setODriveState(uint32_t node_id, uint32_t state);
void setODrivePosition(uint32_t node_id, float position);
uint32_t can_make_id(uint32_t node_id, uint32_t cmd_id);
void setODriveInputMode(uint32_t node_id, uint32_t mode);
void clearODriveErrors(uint32_t node_id);
void checkODriveErrors();
void setODriveGains(uint32_t node_id, float pos_gain, float vel_gain);
void forceODriveConfiguration(uint32_t node_id);
void setODriveControlMode(uint32_t node_id);
bool receiveCANMessage(CANMessage &cmsg);
void IMU_Stabilization_2axis_quaternion(float qw, float qx, float qy, float qz);
void IMU_Stabilization_Gyro(float yaw_rad, float pitch_rad, float roll_rad);
void Jetson_position_control(CANMessage &msg);
float applyLowPass(float *hist, float new_sample);
float wrapAngle(float error);
void FeedbackIMUData();

#define FILTER_TAPS 5

static float yaw_hist[FILTER_TAPS] = {0};
static float pitch_hist[FILTER_TAPS] = {0};
static float roll_hist[FILTER_TAPS] = {0};

static const float kernel[FILTER_TAPS] = {0.2f, 0.2f, 0.2f, 0.2f, 0.2f};

void setup() {
    Serial.begin(115200); // baud rate = 115200
    delay(100);

    // 1. Initialize CAN (1Mbps)
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS(); 
    // Target Node ID: 0x01
    // Shifted left by 5 to align with the ODrive protocol in the 11-bit ID
    uint32_t node_id = 0x01;
    uint32_t filter_id = (node_id << 5) << 21; // Shifted for TWAI register alignment

    // Mask: We care about the top 6 bits (Node ID), ignore the bottom 5 (Command ID)
    // 0x7E0 shifted left for alignment
    uint32_t filter_mask = ~(0x1F << 21); 

    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        twai_start();
        Serial.println(">>> CAN Bus Online (1Mbps) <<<");
    } else {
        Serial.println("!!! CAN Driver Failed !!!");
    }

    delay(1000); // Wait for bus to stabilize (Dominic doubled the wait
    // time here from .5 to 1 seconds)

    // For Node 3 (roll)
    clearODriveErrors(3);
    delay(100);
    setODriveControlMode(3); // Force Position Mode
    delay(100);
    setODriveState(3, 8);    // Engage Closed Loop
    delay(100);
  
    // For Node 4 (yaw)
    clearODriveErrors(4);
    delay(100);
    setODriveControlMode(4); // Force Position Mode
    delay(100);
    setODriveState(4, 8);    // Engage Closed Loop
    delay(100);

    // For Node 5 (pitch)
    clearODriveErrors(5);
    delay(100);
    setODriveControlMode(5); // Force Position Mode
    delay(100);
    setODriveState(5, 8);    // Engage Closed Loop
    delay(100);

    Wire.begin(21, 22);
    Wire.setClock(100000); // Super slow for long wires
    Wire.setTimeOut(10);  // CRITICAL: If I2C fails, it will time out in 10ms instead of hanging
    
    if (!bno.begin()) {
        Serial.println("BNO055 failed to start.");
    }

}

void loop() {
    CANMessage msg;
    
    if (millis() - last_update >= update_interval) {
        last_update = millis();
        
        imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
        IMU_Stabilization_Gyro(gyro.x(), gyro.y(), gyro.z()); //stabilize using velocity feed forward
        
    }
    
    if(receiveCANMessage(msg)){ //check for messages
        if (msg.node_id == 2){
            if(msg.cmd_id == 1){
                Serial.println("CAN Message Received");
                Jetson_position_control(msg); //if jetson sends CAN command to change position, 
            }
            if(msg.cmd_id == 2){
                memcpy(&Toggle_Stabilization, &msg.data[0], 1);
            }
            if(msg.cmd_id == 3)
            {
                memcpy(&Toggle_IMU_Feedback, &msg.data[0], 1);
            }
        }
    }

    if(Toggle_IMU_Feedback){
        FeedbackIMUData();
    }
}


void FeedbackIMUData(){
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    uint8_t data[8] = {0};
    float Accel = 0;
    float Gyro = 0;
    
    Accel = accel.x();
    Gyro = gyro.x();
    memcpy(&data[0], &Accel, 4);
    memcpy(&data[4], &Gyro, 4);
    sendCANMessage(can_make_id(1, 0x04), 8, data);

    std::memset(data, 0, sizeof(data));
    Accel = accel.y();
    Gyro = gyro.y();
    memcpy(&data[0], &Accel, 4);
    memcpy(&data[4], &Gyro, 4);
    sendCANMessage(can_make_id(1, 0x05), 8, data);

    std::memset(data, 0, sizeof(data));
    Accel = accel.z();
    Gyro = gyro.z();
    memcpy(&data[0], &Accel, 4);
    memcpy(&data[4], &Gyro, 4);
    sendCANMessage(can_make_id(1, 0x06), 8, data);
}

float wrapAngle(float error)
{
    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;
    return error;
}

float applyLowPass(float *hist, float new_sample)
{
    float result = 0.0f;

    // shift history
    for(int i = FILTER_TAPS-1; i > 0; i--)
        hist[i] = hist[i-1];

    hist[0] = new_sample;

    // convolution
    for(int i = 0; i < FILTER_TAPS; i++)
        result += hist[i] * kernel[i];
        Serial.println(result);

    
    return result;


}

void IMU_Stabilization_Gyro(float yaw_rad, float pitch_rad, float roll_rad)
{
    uint8_t data[8] = {0};
    int16_t vel_ff = 0;
    int16_t torque_ff = 0;

    float kp = 1.0f;
    float ki = 0.2f;
    float dt = 0.01f;  //10 ms

    // --- Apply convolution low-pass filter ---
    float yaw_filtered   = applyLowPass(yaw_hist, yaw_rad);
    float pitch_filtered = applyLowPass(pitch_hist, pitch_rad);
    float roll_filtered  = applyLowPass(roll_hist, roll_rad);


    //PI Loop    
    float yaw_error   = -yaw_filtered;
    float pitch_error = -pitch_filtered;
    float roll_error  = -roll_filtered;

    static float yaw_integral = 0;
    static float pitch_integral = 0;
    static float roll_integral = 0;

    yaw_integral   += yaw_error * dt;
    pitch_integral += pitch_error * dt;
    roll_integral  += roll_error * dt;

    // --- Anti-windup ---
    float integral_limit = 50.0f;
    yaw_integral   = constrain(yaw_integral, -integral_limit, integral_limit);
    pitch_integral = constrain(pitch_integral, -integral_limit, integral_limit);
    roll_integral  = constrain(roll_integral, -integral_limit, integral_limit);


    // --- ROLL (Node 3) ---
    float roll_output = kp * roll_error + ki * roll_integral;
    memcpy(&data[0], &camera_pos_roll, 4);
    vel_ff = (int16_t)(roll_output * 159.155f);
    memcpy(&data[4], &vel_ff, 2);
    memcpy(&data[6], &torque_ff, 2);
    sendCANMessage(can_make_id(3, 0x0c), 8, data);

    // --- YAW (Node 4) ---
    float yaw_output = kp * yaw_error + ki * yaw_integral;
    memcpy(&data[0], &camera_pos_yaw, 4);
    vel_ff = (int16_t)(yaw_output * 159.155f);
    memcpy(&data[4], &vel_ff, 2);
    memcpy(&data[6], &torque_ff, 2);
    sendCANMessage(can_make_id(4, 0x0c), 8, data);

    // --- PITCH (Node 5) ---
    float pitch_output = kp * pitch_error + ki * pitch_integral;
    memcpy(&data[0], &camera_pos_pitch, 4);
    vel_ff = (int16_t)(pitch_output * 159.155f);
    memcpy(&data[4], &vel_ff, 2);
    memcpy(&data[6], &torque_ff, 2);
    sendCANMessage(can_make_id(5, 0x0c), 8, data);
}

void Jetson_position_control(CANMessage &msg) {
    // msg.data[0] = Axis ID (3, 4, or 5)
    //msg.data[1] = direction (0 left, 1 right)
    // msg.data[2-5] = Float Position (Little Endian)
    
    if(msg.cmd_id == 1){
        float received_pos;
        // Extract bytes 1 through 5 into the float variable
        //memcpy(&direction, &msg.data[1], 1);
        memcpy(&received_pos, &msg.data[1], 4);

        if (msg.data[0] == 3) {
            setODrivePosition(3, received_pos);
            Serial.printf("Jetson set Roll to: %.3f revolutions \n", received_pos);
        }
        else if (msg.data[0] == 4) {
            setODrivePosition(4, received_pos);
            Serial.printf("Jetson set Yaw to: %.3f revolutions \n", received_pos);
        }
        else if (msg.data[0] == 5) {
            setODrivePosition(5, received_pos);
            Serial.printf("Jetson set Pitch to: %.3f revolutions \n", received_pos);
        }
    }
}

void setODrivePosition(uint32_t node_id, float position) {
    uint8_t data[8] = {0};
    memcpy(&data[0], &position, 4); // Position (Float)
    // data[4-5] Vel FF, data[6-7] Torque FF (set to 0)
    uint32_t id = (node_id << 5) | 0x0c; // 0x0c = Set_Input_Pos
    sendCANMessage(id, 8, data);
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

void setODriveInputMode(uint32_t node_id, uint32_t mode) {
 
    uint8_t data[8] = {0};

    memcpy(&data[0], &mode, sizeof(uint32_t));

    uint32_t id = can_make_id(node_id, 0x0b);

    sendCANMessage(id, 8, data);

}

// 0x07: Set Axis State

void setODriveState(uint32_t node_id, uint32_t state) {

    uint8_t data[8] = {0};

    memcpy(&data[0], &state, sizeof(uint32_t));

    uint32_t id = can_make_id(node_id, 0x07);

    sendCANMessage(id, 8, data);

}



void clearODriveErrors(uint32_t node_id) {

    uint8_t data[8] = {0}; // No data needed for this command

    uint32_t id = can_make_id(node_id, 0x18); // 0x18: Clear Errors

    sendCANMessage(id, 8, data);

}



void checkODriveErrors() {

    twai_message_t rx_msg;

    // Non-blocking check for messages

    if (twai_receive(&rx_msg, 0) == ESP_OK) {

        uint32_t node_id = rx_msg.identifier >> 5;

        uint32_t cmd_id = rx_msg.identifier & 0x1F;



        if (cmd_id == 0x01) { // Heartbeat Message

            uint32_t axis_error;

            uint8_t current_state;

            memcpy(&axis_error, &rx_msg.data[0], 4);

            current_state = rx_msg.data[4];



            if (axis_error != 0 || current_state != 8) {

                Serial.printf("Node %d: State %d, Error 0x%08X\n", node_id, current_state, axis_error);

            }

        }

    }

}



void setODriveGains(uint32_t node_id, float pos_gain, float vel_gain) {

    uint8_t data[8];

    // Command 0x1A: Set Gains (This may vary by firmware, check 0x1A or 0x01B)

    // For now, let's try sending a large position command to test

    Serial.printf("Setting Node %d gains...\n", node_id);

   

    // If you can't set gains via CAN easily, we ensure it's in Position Control

    setODriveInputMode(node_id, 1); // 1 = Passthrough

}



void forceODriveConfiguration(uint32_t node_id) {

    uint8_t data[8] = {0};

   

    // 1. Ensure we are in Position Control (3)

    // Command 0x00b: Set Input Mode

    // Data: [Mode (4 bytes), 1 (Passthrough)]

    uint32_t mode = 1; // Passthrough

    memcpy(&data[0], &mode, 4);

    sendCANMessage(can_make_id(node_id, 0x0b), 8, data);

    delay(50);



    // 2. Set the Control Mode to Position Control (3)

    // Some firmware versions require this explicitly via CAN

    // Note: If your ODrive is in Velocity mode, it ignores pos commands.

}



void setODriveControlMode(uint32_t node_id) {

    uint8_t data[8] = {0};

    int32_t control_mode = 3; // 3 = POS_CONTROL

    int32_t input_mode = 1;   // 1 = PASSTHROUGH



    memcpy(&data[0], &control_mode, 4);

    memcpy(&data[4], &input_mode, 4);



    // Command 0x00b is "Set Controller Modes"

    sendCANMessage(can_make_id(node_id, 0x0b), 8, data);

    Serial.printf("Node %d forced to Position Control Mode\n", node_id);

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

void IMU_Stabilization_2axis(float qw, float qx, float qy, float qz) {
    // Remap for gimbal mount
    float W = qw; float X = qz; float Y = qy; float Z = -qx;

    float pitch = asinf(2.0f * (W * Y - Z * X));
    float yaw = atan2f(2.0f * (W * Z + X * Y), 1.0f - 2.0f * (Y * Y + Z * Z));

    float target_p = (pitch - pitch_offset) / (2.0f * PI);
    float target_y = (yaw - yaw_offset) / (2.0f * PI);

    // Send to ODrives (ID 0x0c is Input_Pos)
    uint8_t data[8] = {0};
    memcpy(&data[0], &target_p, 4);
    sendCANMessage(can_make_id(4, 0x0c), 8, data);
    
    memcpy(&data[0], &target_y, 4);
    sendCANMessage(can_make_id(5, 0x0c), 8, data);
}
