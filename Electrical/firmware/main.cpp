#include <Arduino.h>
#include "driver/twai.h"
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <math.h>

// --- HARDWARE PINS ---
#define CAN_TX_PIN GPIO_NUM_32
#define CAN_RX_PIN GPIO_NUM_33
#define I2C_SDA 21
#define I2C_SCL 22

Adafruit_BNO08x bno08x(-1);
sh2_SensorValue_t sensorValue;

// --- LIVE TUNING GAINS ---
float kp_yaw = 15.0f;    
float kp_pitch = 15.0f;  
float ki = 25.0f;         

// --- CONTINUOUS IMU TRACKING ---
float continuous_imu_yaw = 0.0f;
float continuous_imu_pitch = 0.0f;

// --- TARGET ANCHORS (Set by Jetson) ---
float target_camera_yaw = 0.0f;
float target_camera_pitch = 0.0f;

struct CANMessage {
    uint32_t id;
    uint32_t node_id;
    uint32_t cmd_id;
    uint8_t len;
    uint8_t data[8];
};

bool Toggle_Stabilization = false;
bool Toggle_IMU_Feedback = false;

unsigned long last_update = 0;
const int update_interval = 10; // 10ms (100Hz)

// --- FUNCTION PROTOTYPES ---
void checkSerialTuning();
void setReports(void);
void IMU_Stabilization_Velocity();
void Jetson_position_control(CANMessage &msg);
void FeedbackIMUData();
bool sendCANMessage(uint32_t id, uint8_t len, uint8_t* data);
bool receiveCANMessage(CANMessage &cmsg);
void setODriveState(uint32_t node_id, uint32_t state);
void setODriveVelocityMode(uint32_t node_id);
void sendODriveVelocity(uint32_t node_id, float velocity_turns_sec);
void clearODriveErrors(uint32_t node_id);
uint32_t can_make_id(uint32_t node_id, uint32_t cmd_id);
void setODriveTrapTrajMode(uint32_t node_id);
void setODrivePositionMode(uint32_t node_id);
void setODrivePosition(uint32_t node_id, float position,float velff);

void checkSerialTuning() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim(); 
        
        if (input.equalsIgnoreCase("GO")) {
            Serial.println(">>> MANUALLY WAKING UP MOTORS <<<");
            clearODriveErrors(0);
            clearODriveErrors(1);
            delay(50);
            setODriveState(0, 8);
            setODriveState(1, 8);
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
    if (!bno08x.begin_I2C(0x4A, &Wire)) {
        Serial.println("BNO08x failed to start.");
        while(1) { delay(10); } 
    }
    
    setReports();
    
    Serial.println("Waiting 3 seconds for ODrives to fully wake up...");
    delay(3000); 
    
    // Set to position Mode
    setODrivePositionMode(0);
    setODrivePositionMode(1);
    delay(50);

    Serial.println(">>> AUTO-WAKING MOTORS <<<");
    clearODriveErrors(0);
    clearODriveErrors(1);
    delay(50);
    setODriveState(0, 8);
    setODriveState(1, 8);
}

void loop() {
    CANMessage msg;

    checkSerialTuning();

    // Check for incoming Jetson Commands
    if (receiveCANMessage(msg)) {
        Jetson_position_control(msg);
    }

    if (bno08x.wasReset()) {
        setReports();
    }

    // --- IMU PROCESSING & UNWRAP ALGORITHM ---
    if (bno08x.getSensorEvent(&sensorValue)) {
        if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
            float qw = sensorValue.un.gameRotationVector.real;
            float qx = sensorValue.un.gameRotationVector.i; 
            float qy = sensorValue.un.gameRotationVector.j; 
            float qz = sensorValue.un.gameRotationVector.k; 

            float sqx = qx * qx;
            float sqy = qy * qy;
            float sqz = qz * qz;

            float pitch_rad = atan2(2.0f * (qw * qx + qy * qz), 1.0f - 2.0f * (sqx + sqy));
            float yaw_rad = atan2(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (sqy + sqz));

            float raw_pitch = pitch_rad / (2.0f * PI);
            float raw_yaw = yaw_rad / (2.0f * PI); 

            static float prev_raw_pitch = 0.0f;
            static float prev_raw_yaw = 0.0f;
            static bool first_loop = true;

            if (first_loop) {
                prev_raw_pitch = raw_pitch;
                prev_raw_yaw = raw_yaw;
                continuous_imu_pitch = 0.0f; 
                continuous_imu_yaw = 0.0f;
                // Sync Jetson anchors to Tare point
                target_camera_pitch = 0.0f;
                target_camera_yaw = 0.0f;
                first_loop = false;
                Serial.println(">>> IMU ZEROED! <<<");
            }

            float delta_pitch = raw_pitch - prev_raw_pitch;
            float delta_yaw = raw_yaw - prev_raw_yaw;

            if (delta_pitch > 0.5f) delta_pitch -= 1.0f;
            else if (delta_pitch < -0.5f) delta_pitch += 1.0f;

            if (delta_yaw > 0.5f) delta_yaw -= 1.0f;
            else if (delta_yaw < -0.5f) delta_yaw += 1.0f;

            continuous_imu_pitch += delta_pitch;
            continuous_imu_yaw += delta_yaw;

            prev_raw_pitch = raw_pitch;
            prev_raw_yaw = raw_yaw;
        }
    }

    // --- 100Hz CONTROL LOOP ---
    if (millis() - last_update >= update_interval) {
        last_update = millis();
        
        if (Toggle_Stabilization) {
            IMU_Stabilization_Velocity(); 
        } 
        else {}

        if (Toggle_IMU_Feedback) {
            FeedbackIMUData();
        }

        /*
        static int print_counter = 0;
        if (++print_counter >= 10) { 
            Serial.printf("Pitch Err: %.3f | Yaw Err: %.3f\n", 
                          (target_camera_pitch - continuous_imu_pitch), 
                          (target_camera_yaw - continuous_imu_yaw));
            print_counter = 0;
        }
        */
    }
}

// --- VELOCITY STABILIZATION LOGIC ---
void IMU_Stabilization_Velocity() {
    float dt = update_interval / 1000.0f; // 0.01s
    
    // 1. Calculate Absolute Error vs Jetson Targets
    float pitch_error = target_camera_pitch - continuous_imu_pitch;
    float yaw_error = target_camera_yaw - continuous_imu_yaw;

    // 2. PI Loop Accumulator
    static float pitch_integral = 0.0f;
    static float yaw_integral = 0.0f;

    pitch_integral += pitch_error * dt;
    yaw_integral += yaw_error * dt;

    // Anti-Windup
    float integral_limit = 5.0f; 
    pitch_integral = constrain(pitch_integral, -integral_limit, integral_limit);
    yaw_integral = constrain(yaw_integral, -integral_limit, integral_limit);

    // 3. Final Velocity Command Math
    float pitch_velocity = (pitch_error * kp_pitch) + (pitch_integral * ki);
    float yaw_velocity = (yaw_error * kp_yaw) + (yaw_integral * ki);

    // --- DIRECTION FIXES ---
    yaw_velocity = -yaw_velocity;

    // 4. Fire to CAN Bus
    setODrivePosition(1, target_camera_pitch, pitch_velocity * 1000);
    setODrivePosition(0, target_camera_yaw, yaw_velocity * 1000);
}

// --- JETSON COMMAND HANDLER ---
void Jetson_position_control(CANMessage &msg) {
    if (msg.cmd_id == 1) {
        float received_pos;
        memcpy(&received_pos, &msg.data[1], 4);

        if (msg.data[0] == 0) {
            target_camera_yaw = received_pos;
            setODrivePosition(0, target_camera_yaw, 0); 
        }
        else if (msg.data[0] == 1) {
            target_camera_pitch = received_pos;
            setODrivePosition(1, target_camera_pitch, 0); 
        }
    }
    if(msg.cmd_id == 2){
        memcpy(&Toggle_Stabilization, &msg.data[0], 1);
    }
}

void setODrivePosition(uint32_t node_id, float position, float velff) {
    uint8_t data[8] = {0};
    memcpy(&data[0], &position, 4); // Position (Float)
    memcpy(&data[4], &velff, 2); // Velocity FF (Float)
    // data[4-5] Vel FF, data[6-7] Torque FF (set to 0)
    uint32_t id = (node_id << 5) | 0x0c; // 0x0c = Set_Input_Pos
    sendCANMessage(id, 8, data);
}

// --- IMU FEEDBACK ---
void FeedbackIMUData() {
    uint8_t data[8] = {0};
    memcpy(&data[0], &continuous_imu_yaw, 4);
    memcpy(&data[4], &continuous_imu_pitch, 4);
    sendCANMessage(can_make_id(1, 0x03), 8, data);
}

// --- CAN / ODRIVE HELPERS ---
uint32_t can_make_id(uint32_t node_id, uint32_t cmd_id) {
    return (node_id << 5) | cmd_id;
}

bool sendCANMessage(uint32_t id, uint8_t len, uint8_t* data) {
    twai_message_t message;
    message.identifier = id;
    message.extd = 0;
    message.data_length_code = len;
    for (int i = 0; i < len; i++) message.data[i] = data[i];

    esp_err_t res = twai_transmit(&message, pdMS_TO_TICKS(2));
    if (res == ESP_ERR_INVALID_STATE) {
        twai_stop();
        twai_start();
        return false;
    }
    return (res == ESP_OK);
}

bool receiveCANMessage(CANMessage &msg) {
    twai_message_t rx_msg;
    if (twai_receive(&rx_msg, 0) == ESP_OK) {
        msg.id = rx_msg.identifier;
        msg.node_id = rx_msg.identifier >> 5;  
        msg.cmd_id = rx_msg.identifier & 0x1F; 
        msg.len = rx_msg.data_length_code;
        memcpy(msg.data, rx_msg.data, rx_msg.data_length_code);
        return true;
    }
    return false;
}

void setODriveState(uint32_t node_id, uint32_t state) {
    uint8_t data[4];
    memcpy(data, &state, 4);
    sendCANMessage(can_make_id(node_id, 0x07), 4, data);
}

void setODriveVelocityMode(uint32_t node_id) {
    uint8_t data[8] = {0};
    int32_t control_mode = 2; // VELOCITY_CONTROL
    int32_t input_mode = 1;   // PASSTHROUGH
    memcpy(&data[0], &control_mode, 4);
    memcpy(&data[4], &input_mode, 4);
    sendCANMessage(can_make_id(node_id, 0x0B), 8, data);
}

void setODrivePositionMode(uint32_t node_id) {
    uint8_t data[8] = {0};
    int32_t control_mode = 3; // VELOCITY_CONTROL
    int32_t input_mode = 5;   // PASSTHROUGH
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

void clearODriveErrors(uint32_t node_id) {
    uint8_t data[1] = {0};
    sendCANMessage(can_make_id(node_id, 0x18), 0, data); 
}

void setODriveTrapTrajMode(uint32_t node_id){
    uint8_t data[8] = {0};
    int32_t control_mode = 3; // Position_CONTROL
    int32_t input_mode = 5;   // PASSTHROUGH
    memcpy(&data[0], &control_mode, 4);
    memcpy(&data[4], &input_mode, 4);
    sendCANMessage(can_make_id(node_id, 0x0B), 8, data);
}
