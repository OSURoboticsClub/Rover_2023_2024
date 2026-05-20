#include <Arduino.h>
#include "driver/twai.h"
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <math.h>
#include <cstring>

#define CAN_TX_PIN GPIO_NUM_2
#define CAN_RX_PIN GPIO_NUM_15
#define I2C_SDA 21
#define I2C_SCL 22


Adafruit_BNO08x bno08x(-1);
sh2_SensorValue_t sensorValue;

// --- LIVE TUNING GAINS ---
float kp_yaw = 5.0f;    
float kp_pitch = 5.0f;  
float kp_roll = 5.0f;
float ki = 10.0f;         

// --- CONTINUOUS IMU TRACKING ---
float continuous_imu_yaw = 0.0f;
float continuous_imu_pitch = 0.0f;
float continuous_imu_roll = 0.0f;

// --- TARGET ANCHORS (Set by Jetson) ---
float target_camera_yaw = 0.0f;
float target_camera_pitch = 0.0f;
float target_camera_roll = 0.0f;

struct CANMessage {
    uint32_t id;
    uint32_t node_id;
    uint32_t cmd_id;
    uint8_t len;
    uint8_t data[8];
};

bool Toggle_Stabilization = true;
bool Toggle_IMU_Feedback = false;
bool imu_data_fresh = false;

unsigned long last_update = 0;
const int update_interval = 10; // 10ms (100Hz)

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
void Jetson_position_control(CANMessage &msg);
float applyLowPass(float *hist, float new_sample);
float wrapAngle(float error);
void FeedbackIMUData();
void setODriveVelocityMode(uint32_t node_id);
void sendODriveVelocity(uint32_t node_id, float velocity_turns_sec);
void checkSerialTuning();
void setReports(void);
void IMU_Stabilization_Velocity();

#define FILTER_TAPS 5

static float yaw_hist[FILTER_TAPS] = {0};
static float pitch_hist[FILTER_TAPS] = {0};
static float roll_hist[FILTER_TAPS] = {0};

static const float kernel[FILTER_TAPS] = {0.2f, 0.2f, 0.2f, 0.2f, 0.2f};

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

    

    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(60000); // force slow 100kHz


    if (!bno08x.begin_I2C(0x4A, &Wire)) {
        Serial.println("BNO08x failed to start.");
        while(1) { delay(10); } 
    }
    
    setReports();
    
    Serial.println("Waiting 3 seconds for ODrives to fully wake up...");
    delay(3000); 
    
    // Set to Velocity Mode
    setODriveVelocityMode(3);
    setODriveVelocityMode(4);
    setODriveVelocityMode(5);
    delay(50);

    Serial.println(">>> AUTO-WAKING MOTORS <<<");
    clearODriveErrors(3);
    clearODriveErrors(4);
    clearODriveErrors(5);
    delay(50);
    setODriveState(3, 8);
    setODriveState(4, 8);
    setODriveState(5, 8);
}

void loop() {
    CANMessage msg;

    checkSerialTuning();

    // Check for incoming Jetson Commands
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

    if (bno08x.wasReset()) {
        setReports();
    }

    // --- IMU PROCESSING & UNWRAP ALGORITHM ---
    if (bno08x.getSensorEvent(&sensorValue)) {
        if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
            imu_data_fresh = true; // Mark that we got fresh data
            float qw = sensorValue.un.gameRotationVector.real;
            float qx = sensorValue.un.gameRotationVector.i; 
            float qy = sensorValue.un.gameRotationVector.j; 
            float qz = sensorValue.un.gameRotationVector.k; 

            float sqx = qx * qx;
            float sqy = qy * qy;
            float sqz = qz * qz;

            float roll_rad  = asin( 2.0f * (qw * qy - qz * qx));  
            float pitch_rad = atan2(2.0f * (qw * qx + qy * qz), 1.0f - 2.0f * (sqx + sqy));
            float yaw_rad = atan2(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (sqy + sqz));

            float raw_pitch = pitch_rad / (2.0f * PI);
            float raw_yaw = yaw_rad / (2.0f * PI); 
            float raw_roll = roll_rad / (2.0f * PI); 

            static float prev_raw_pitch = 0.0f;
            static float prev_raw_yaw = 0.0f;
            static float prev_raw_roll = 0.0f;
            static bool first_loop = true;

            if (first_loop) {
                // Treat the very first IMU string reading as the structural zero
                prev_raw_pitch = raw_pitch;
                prev_raw_yaw   = raw_yaw;
                prev_raw_roll  = raw_roll;
                
                continuous_imu_pitch = 0.0f; 
                continuous_imu_yaw   = 0.0f;
                continuous_imu_roll  = 0.0f;

                // CRITICAL: Bind targets dynamically to 0.0 relative to its current boot spot
                target_camera_pitch = 0.0f;
                target_camera_yaw   = 0.0f;
                target_camera_roll  = 0.0f;
                
                first_loop = false;
                Serial.println("SENSORS ZEROED");
            }

            float delta_pitch = raw_pitch - prev_raw_pitch;
            float delta_yaw = raw_yaw - prev_raw_yaw;
            float delta_roll = raw_roll - prev_raw_roll;

            if (delta_pitch > 0.5f) delta_pitch -= 1.0f;
            else if (delta_pitch < -0.5f) delta_pitch += 1.0f;

            if (delta_yaw > 0.5f) delta_yaw -= 1.0f;
            else if (delta_yaw < -0.5f) delta_yaw += 1.0f;

            if (delta_roll > 0.5f) delta_roll -= 1.0f;
            else if (delta_roll < -0.5f) delta_roll += 1.0f;

            continuous_imu_pitch += delta_pitch;
            continuous_imu_yaw += delta_yaw;
            continuous_imu_roll += delta_roll;

            prev_raw_pitch = raw_pitch;
            prev_raw_yaw = raw_yaw;
            prev_raw_roll = raw_roll;
        }
    }

    // --- 100Hz CONTROL LOOP ---
    if (millis() - last_update >= update_interval) {
        last_update = millis();
        
        if (Toggle_Stabilization && imu_data_fresh) {
            IMU_Stabilization_Velocity(); 
        } else {
            // Safety: if stabilization is toggled off, stop the motors
            sendODriveVelocity(3, 0.0f);
            sendODriveVelocity(4, 0.0f);
            sendODriveVelocity(5, 0.0f);
        }

        imu_data_fresh = false;

        if (Toggle_IMU_Feedback) {
            FeedbackIMUData();
        }

        static int print_counter = 0;
        if (++print_counter >= 10) { 
            Serial.printf("Pitch Err: %.3f | Yaw Err: %.3f\n | Roll Err: %.3f\n",  
                          (target_camera_pitch - continuous_imu_pitch), 
                          (target_camera_yaw - continuous_imu_yaw),
                          (target_camera_roll - continuous_imu_roll));
            print_counter = 0;
        }
    }

    if(Toggle_IMU_Feedback){
        FeedbackIMUData();
    }
}

// --- VELOCITY STABILIZATION LOGIC ---
void IMU_Stabilization_Velocity() {
    float dt = update_interval / 1000.0f; // 0.01s
    
    // 1. Calculate Absolute Error vs Jetson Targets
    float pitch_error = target_camera_pitch - continuous_imu_pitch;
    float yaw_error = target_camera_yaw - continuous_imu_yaw;
    float roll_error = target_camera_roll - continuous_imu_roll;

    // 2. PI Loop Accumulator
    static float pitch_integral = 0.0f;
    static float yaw_integral = 0.0f;
    static float roll_integral = 0.0f;

    pitch_integral += pitch_error * dt;
    yaw_integral += yaw_error * dt;
    roll_integral += roll_error * dt;

    // Anti-Windup
    float integral_limit = 5.0f; 
    pitch_integral = constrain(pitch_integral, -integral_limit, integral_limit);
    yaw_integral = constrain(yaw_integral, -integral_limit, integral_limit);
    roll_integral = constrain(roll_integral, -integral_limit, integral_limit);

    // 3. Final Velocity Command Math
    float pitch_velocity = (pitch_error * kp_pitch) + (pitch_integral * ki);
    float yaw_velocity = (yaw_error * kp_yaw) + (yaw_integral * ki);
    float roll_velocity = (roll_error * kp_roll) + (roll_integral * ki);

    roll_velocity  = constrain(roll_velocity,  -1.0f, 1.0f);
    yaw_velocity  = constrain(yaw_velocity,  -1.0f, 1.0f);
    pitch_velocity  = constrain(pitch_velocity,  -1.0f, 1.0f);

    // 4. Fire to CAN Bus
    sendODriveVelocity(3, roll_velocity);
    sendODriveVelocity(4, yaw_velocity);
    sendODriveVelocity(5, pitch_velocity);
}

void FeedbackIMUData(){
    uint8_t data[8] = {0};
    memcpy(&data[0], &continuous_imu_roll, 4);
    sendCANMessage(can_make_id(1, 0x04), 8, data);
    memcpy(&data[0], &continuous_imu_yaw, 4);
    sendCANMessage(can_make_id(1, 0x05), 8, data);
    memcpy(&data[0], &continuous_imu_pitch, 4);
    sendCANMessage(can_make_id(1, 0x06), 8, data);
}

float wrapAngle(float error)
{
    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;
    return error;
}

void setODriveVelocityMode(uint32_t node_id){
    uint8_t data[8] = {0};
    int32_t control_mode = 2; // VELOCITY_CONTROL
    int32_t input_mode = 1;   // PASSTHROUGH
    memcpy(&data[0], &control_mode, 4);
    memcpy(&data[4], &input_mode, 4);
    sendCANMessage(can_make_id(node_id, 0x0B), 8, data);
}

void sendODriveVelocity(uint32_t node_id, float velocity_turns_sec) {
    uint8_t data[8] = {0};
    float torque_ff = 0.0f;
    memcpy(&data[0], &velocity_turns_sec, 4);
    memcpy(&data[4], &torque_ff, 4);
    sendCANMessage(can_make_id(node_id, 0x0D), 8, data);
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

void Jetson_position_control(CANMessage &msg) {
    // msg.data[0] = Axis ID (3, 4, or 5)
    // msg.data[1-4] = Float Position (Little Endian)
    
    if(msg.cmd_id == 1){
        float received_pos;
        // Extract bytes 1 through 5 into the float variable
        //memcpy(&direction, &msg.data[1], 1);
        memcpy(&received_pos, &msg.data[1], 4);

        if (msg.data[0] == 3) {
            setODrivePosition(3, received_pos);
        }
        else if (msg.data[0] == 4) {
            setODrivePosition(4, received_pos);
        }
        else if (msg.data[0] == 5) {
            setODrivePosition(5, received_pos);
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

void checkSerialTuning() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim(); 
        
        if (input.equalsIgnoreCase("GO")) {
            Serial.println(">>> MANUALLY WAKING UP MOTORS <<<");
            clearODriveErrors(3);
            clearODriveErrors(4);
            clearODriveErrors(5);
            delay(50);
            setODriveState(3, 8);
            setODriveState(4, 8);
            setODriveState(5, 8);
        }
        else if (input.startsWith("py") || input.startsWith("PY")) {
            kp_yaw = input.substring(2).toFloat();
            Serial.printf(">>> Yaw Kp: %.1f <<<\n", kp_yaw);
        } 
        else if (input.startsWith("pp") || input.startsWith("PP")) {
            kp_pitch = input.substring(2).toFloat();
            Serial.printf(">>> Pitch Kp: %.1f <<<\n", kp_pitch);
        }
        else if (input.startsWith("i") || input.startsWith("I")) {
            ki = input.substring(1).toFloat();
            Serial.printf(">>> Ki: %.3f <<<\n", ki);
        }
    }
}

void setReports(void) {
    if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 10000)) {
        Serial.println("Could not enable Game Rotation Vector");
    }
}
