#include <SimpleFOC.h>
#include <MPU6050_tockn.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "esp_adc_cal.h"
#include <SPI.h>
#include "Bitcraze_PMW3901.h"
#include "Servo_STS3032.h"

// ====================================================================
// 光流配置
// ====================================================================
#define PIN_SCK  4
#define PIN_MISO 36
#define PIN_MOSI 0
#define PIN_CS   21

Bitcraze_PMW3901 flow(PIN_CS);

long last_flow_time = 0;
float flow_vel_y = 0.0;
float fused_vel = 0.0;
float FLOW_SCALE_FACTOR = 0.015; 
float FLOW_WEIGHT = 0.3;

// ====================================================================
// 电池检测
// ====================================================================
int BAT_PIN = 35; 
static esp_adc_cal_characteristics_t adc_chars;

// ====================================================================
// 舵机
// ====================================================================
SMS_STS sms_sts;
const s16 LEFT_ZERO_POS  = 1690;
const s16 RIGHT_ZERO_POS = 2370;

// ====================================================================
// 电机与传感器
// ====================================================================
BLDCMotor motor1 = BLDCMotor(7);
BLDCMotor motor2 = BLDCMotor(7);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(32, 33, 25, 22);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(26, 27, 14, 12);

TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor2 = MagneticSensorI2C(AS5600_I2C);
MPU6050 mpu6050(I2Ctwo);

// ====================================================================
// WiFi & UDP
// ====================================================================
const char* ssid = "17-6-302";
const char* password = "Qwe30268";
const char* udpAddress = "192.168.32.48";
const int udpPort = 9870;

WiFiUDP udp;
unsigned int localPort = 8888;
char packetBuffer[255];

// ====================================================================
// 控制参数(全部改成 float 全局变量,可远程修改)
// ====================================================================
float K_speed       = -1.0524;
float K_pitch_rate  = -0.1424;
float K_yaw_rate    = -0.1291;
float K_pitch       = -4.0751;
float K_speed_int   = -0.15;
float K_yaw_int     = 0.1054;

// 视觉指令
float target_v = 0.0;
float target_w = 0.0;
float target_height = 100.0;

// 状态变量
float ANGLE_OFFSET = 3.4;
float filtered_spd = 0.0;
float dx = 0.0;
float int_dx = 0.0;
unsigned long last_lqr_time = 0;
float actual_x = 0.0;
float target_x = 0.0;
float filtered_dtheta = 0.0;
unsigned long last_print_time = 0;
float global_u_total = 0.0;

// 👑 阶段 0 新增:变形冷却时间记录
unsigned long last_height_change_time = 0;

// 👑 阶段 0 新增:CALIBRATION_MODE 开关(默认关闭)
bool CALIBRATION_MODE = false;

// 👑 阶段 0 新增:在线补偿(为阶段 4 准备,默认关闭)
float u_compensation = 0.0;
float comp_learn_rate = 0.0;  // 默认 0 = 不学习,等阶段 4 再启用
bool ENABLE_COMPENSATION = false;

// 👑 阶段 0 新增:状态枚举(为阶段 5 准备,目前简单使用)
enum RobotState { STATE_IDLE, STATE_DRIVE, STATE_TRANSITION };
int current_state = STATE_IDLE;  // 用 int 方便 UDP 上报

// 👑 阶段 0 新增:陀螺仪零偏学习(为阶段 1 准备,默认关闭)
float gyro_y_bias = 0.0;
bool ENABLE_GYRO_BIAS_LEARN = false;


// 👑 阶段 5:状态机辅助变量
unsigned long transition_start_time = 0;
int transition_target_state = STATE_IDLE;
float int_dx_at_transition_start = 0.0;
unsigned long drive_to_idle_timer = 0;

// 参数(可改)
const float IDLE_DEAD_ZONE = 0.15;
const unsigned long TRANSITION_DURATION_MS = 200;
const unsigned long DRIVE_TO_IDLE_DEBOUNCE_MS = 100;

// UDP 调试用
float transition_progress = 0.0;  // 0-1,只在 TRANSITION 中有意义
float KD_POS = 0.5;

// 👑 阶段 5:级联外环(位置 → 速度)
float OUTER_KP = 0.3;              // 外环比例增益
float OUTER_KD = 0.3;  // 外环微分增益,起始值
float OUTER_MAX_V = 0.3;          // 外环输出速度限幅(m/s)
const float OUTER_DEAD_ZONE = 0.01;  // 外环死区(米),小偏差不响应

float outer_target_v = 0.0;        // 外环输出
float effective_target_v = 0.0;    // 内环实际跟踪的速度
unsigned long last_outer_loop_time = 0;
const unsigned long OUTER_LOOP_PERIOD_MS = 20;  // 50Hz

// ====================================================================
// IK 逆解
// ====================================================================
const int IK_POINTS = 5;
const float ik_axle_dist[IK_POINTS]  = {46.35,56.35,66.35,76.35,86.35};
const float ik_servo_angle[IK_POINTS] = {17.13,22.57,28.77,35.72,43.43};
const float REAL_WHEEL_RADIUS = 26.5;

float calculate_ik_servo_angle(float target_height_from_ground) {
    float target_dist = target_height_from_ground - REAL_WHEEL_RADIUS;
    if (target_dist <= ik_axle_dist[0]) return ik_servo_angle[0];
    if (target_dist >= ik_axle_dist[IK_POINTS - 1]) return ik_servo_angle[IK_POINTS - 1];
    for (int i = 0; i < IK_POINTS - 1; i++) {
        if (target_dist >= ik_axle_dist[i] && target_dist <= ik_axle_dist[i+1]) {
            float ratio = (target_dist - ik_axle_dist[i]) / (ik_axle_dist[i+1] - ik_axle_dist[i]);
            return ik_servo_angle[i] + ratio * (ik_servo_angle[i+1] - ik_servo_angle[i]);
        }
    }
    return ik_servo_angle[0]; 
}

// ====================================================================
// 增益调度
// ====================================================================
const float sched_heights[3] = {80.0, 100.0, 125.0};
const float sched_k_global[3] = {3.5, 4.0, 5.0};
const float sched_offset[3]   = {9.0, 3.4, 4.0};

float get_dynamic_K_global(float h) {
    if (h <= sched_heights[0]) return sched_k_global[0];
    if (h >= sched_heights[2]) return sched_k_global[2];
    if (h < sched_heights[1]) {
        float ratio = (h - sched_heights[0]) / (sched_heights[1] - sched_heights[0]);
        return sched_k_global[0] + ratio * (sched_k_global[1] - sched_k_global[0]);
    }
    float ratio = (h - sched_heights[1]) / (sched_heights[2] - sched_heights[1]);
    return sched_k_global[1] + ratio * (sched_k_global[2] - sched_k_global[1]);
}

float get_dynamic_Offset(float h) {
    if (h <= sched_heights[0]) return sched_offset[0];
    if (h >= sched_heights[2]) return sched_offset[2];
    if (h < sched_heights[1]) {
        float ratio = (h - sched_heights[0]) / (sched_heights[1] - sched_heights[0]);
        return sched_offset[0] + ratio * (sched_offset[1] - sched_offset[0]);
    }
    float ratio = (h - sched_heights[1]) / (sched_heights[2] - sched_heights[1]);
    return sched_offset[1] + ratio * (sched_offset[2] - sched_offset[1]);
}

// ====================================================================
// 👑 阶段 0 新增:封装变形函数(自动记录时间)
// ====================================================================
void execute_height_change(float new_height) {
    if (new_height < 80.0 || new_height > 125.0) {
        Serial.print("⚠️ 变形被拒绝(超范围): ");
        Serial.println(new_height);
        return;
    }
    if (abs(new_height - target_height) < 1.0) return;
    
    target_height = new_height;
    float target_angle_deg = calculate_ik_servo_angle(target_height);
    int angle_step = target_angle_deg * (4096.0 / 360.0);
    
    byte move_ID[2] = {1, 2};    
    byte move_ACC[2] = {30, 30};    
    u16 move_Speed[2] = {400, 400}; 
    s16 move_Position[2] = {(s16)(LEFT_ZERO_POS + angle_step), 
                            (s16)(RIGHT_ZERO_POS - angle_step)};
    sms_sts.SyncWritePosEx(move_ID, 2, move_Position, move_Speed, move_ACC);
    
    last_height_change_time = millis();
    
    Serial.print("🦿 变形完成 -> 高度: ");
    Serial.println(target_height);
}

// ====================================================================
// 👑 阶段 0 新增:远程指令解析器
// 支持的指令格式:
//   H100      → 设置高度为 100mm
//   O5.83     → 设置 ANGLE_OFFSET 为 5.83
//   I-0.5     → 设置 K_speed_int 为 -0.5
//   L0.05     → 设置 comp_learn_rate 为 0.05
//   CAL1      → 启用 CALIBRATION_MODE
//   CAL0      → 禁用 CALIBRATION_MODE
//   COMP1     → 启用在线补偿
//   COMP0     → 禁用在线补偿
//   GBIAS1    → 启用陀螺仪零偏学习
//   GBIAS0    → 禁用陀螺仪零偏学习
//   RESET     → 重置 int_dx, u_compensation, actual_x
//   ?         → 打印当前所有参数
// ====================================================================
void parse_remote_command(String cmd) {
    cmd.trim();
    if (cmd.length() == 0) return;
    
    if (cmd.startsWith("H")) {
        float val = cmd.substring(1).toFloat();
        execute_height_change(val);
    }
    else if (cmd.startsWith("OK")) {
        float val = cmd.substring(2).toFloat();
        OUTER_KP = val;
        Serial.print("✅ OUTER_KP = "); Serial.println(OUTER_KP);
    }
    else if (cmd.startsWith("OD")) {
        float val = cmd.substring(2).toFloat();
        OUTER_KD = val;
        Serial.print("✅ OUTER_KD = "); Serial.println(OUTER_KD);
    }
    else if (cmd.startsWith("OM")) {
        float val = cmd.substring(2).toFloat();
        OUTER_MAX_V = val;
        Serial.print("✅ OUTER_MAX_V = "); Serial.println(OUTER_MAX_V);
    }
    else if (cmd.startsWith("O")) {
        float val = cmd.substring(1).toFloat();
        ANGLE_OFFSET = val;
        Serial.print("✅ ANGLE_OFFSET = ");
        Serial.println(ANGLE_OFFSET);
    }
    else if (cmd.startsWith("I")) {
        float val = cmd.substring(1).toFloat();
        K_speed_int = val;
        Serial.print("✅ K_speed_int = ");
        Serial.println(K_speed_int);
    }
    else if (cmd.startsWith("L")) {
        float val = cmd.substring(1).toFloat();
        comp_learn_rate = val;
        Serial.print("✅ comp_learn_rate = ");
        Serial.println(comp_learn_rate);
    }
    else if (cmd == "CAL1") {
        CALIBRATION_MODE = true;
        int_dx = 0.0;
        u_compensation = 0.0;
        Serial.println("✅ 进入标定模式(纯姿态环)");
    }
    else if (cmd == "CAL0") {
        CALIBRATION_MODE = false;
        Serial.println("✅ 退出标定模式");
    }
    else if (cmd == "COMP1") {
        ENABLE_COMPENSATION = true;
        Serial.println("✅ 启用在线补偿");
    }
    else if (cmd == "COMP0") {
        ENABLE_COMPENSATION = false;
        u_compensation = 0.0;
        Serial.println("✅ 禁用在线补偿,清零");
    }
    else if (cmd == "GBIAS1") {
        ENABLE_GYRO_BIAS_LEARN = true;
        Serial.println("✅ 启用陀螺仪零偏学习");
    }
    else if (cmd == "GBIAS0") {
        ENABLE_GYRO_BIAS_LEARN = false;
        Serial.println("✅ 禁用陀螺仪零偏学习");
    }
    else if (cmd == "RESET") {
        int_dx = 0.0;
        u_compensation = 0.0;
        actual_x = 0.0;
        target_x = 0.0;
        Serial.println("✅ 重置状态");
    }
    else if (cmd == "?") {
        Serial.println("===== 当前参数 =====");
        Serial.print("  CAL_MODE = "); Serial.println(CALIBRATION_MODE);
        Serial.print("  COMP_EN  = "); Serial.println(ENABLE_COMPENSATION);
        Serial.print("  GBIAS_EN = "); Serial.println(ENABLE_GYRO_BIAS_LEARN);
        Serial.print("  ANGLE_OFFSET = "); Serial.println(ANGLE_OFFSET);
        Serial.print("  K_speed_int  = "); Serial.println(K_speed_int);
        Serial.print("  comp_learn_rate = "); Serial.println(comp_learn_rate);
        Serial.print("  target_height = "); Serial.println(target_height);
        Serial.print("  gyro_y_bias = "); Serial.println(gyro_y_bias);
        Serial.println("====================");
    }
    else if (cmd == "SHOW") {
        Serial.println("===== 状态机状态 =====");
        Serial.print("  current_state = ");
        if (current_state == STATE_IDLE) Serial.println("IDLE");
        else if (current_state == STATE_DRIVE) Serial.println("DRIVE");
        else Serial.println("TRANSITION");
        Serial.print("  target_v = "); Serial.println(target_v);
        Serial.print("  target_w = "); Serial.println(target_w);
        Serial.print("  int_dx = "); Serial.println(int_dx, 3);
        Serial.print("  actual_x = "); Serial.println(actual_x, 3);
        Serial.print("  target_x = "); Serial.println(target_x, 3);
        Serial.print("  pos_error = "); Serial.println(target_x - actual_x, 3);
        if (current_state == STATE_TRANSITION) {
            Serial.print("  TRANSITION → ");
            Serial.println(transition_target_state == STATE_IDLE ? "IDLE" : "DRIVE");
            Serial.print("  Progress = "); Serial.println(transition_progress, 2);
        }
        Serial.println("======================");
    }
    else if (cmd == "TV0") {
        target_v = 0.0;
        Serial.println("✅ target_v = 0 (触发 IDLE)");
    }
    else if (cmd.startsWith("TV")) {
        float val = cmd.substring(2).toFloat();
        target_v = val;
        Serial.print("✅ target_v = "); Serial.println(target_v);
    }
    else if (cmd.startsWith("D")) {
        float val = cmd.substring(1).toFloat();
        KD_POS = val;
        Serial.print("✅ KD_POS = "); Serial.println(KD_POS);
    }
    else {
        // 兼容老格式:纯数字 = 设置高度
        float val = cmd.toFloat();
        if (val >= 80.0 && val <= 125.0) {
            execute_height_change(val);
        } else {
            Serial.print("⚠️ 未知指令: ");
            Serial.println(cmd);
        }
    }
}

// ====================================================================
// SETUP
// ====================================================================
void setup() {
    Serial.begin(1000000); 
    delay(1000); 
    Serial.println("\n\n=== 🤖 机甲系统启动 - 阶段0调试版 ===");

    Serial.print("正在连接 WiFi: ");
    Serial.print(ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) { 
        delay(500); 
        Serial.print(".");
    }
    Serial.println("\n✅ WiFi连接成功!");
    Serial.print("👉 IP: ");
    Serial.println(WiFi.localIP());

    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 0, &adc_chars);

    Serial2.begin(1000000);
    Serial1.begin(115200, SERIAL_8N1, 34, 22);

    I2Cone.begin(19, 18, 400000UL); 
    I2Ctwo.begin(23, 5, 400000UL);

    sensor1.init(&I2Cone);
    sensor2.init(&I2Ctwo);
    mpu6050.begin();
    mpu6050.calcGyroOffsets(true);

    SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
    if (!flow.begin()) {
        Serial.println("⚠️ 光流掉线");
        FLOW_WEIGHT = 0.0;
    }

    // 初始变形(用封装函数,自动记录时间)
    sms_sts.pSerial = &Serial2;
    {
        float target_angle_deg = calculate_ik_servo_angle(target_height);
        int angle_step = target_angle_deg * (4096.0 / 360.0);
        byte init_ID[2] = {1, 2};    
        byte init_ACC[2] = {30, 30};    
        u16 init_Speed[2] = {300, 300}; 
        s16 init_Position[2] = {(s16)(LEFT_ZERO_POS + angle_step), 
                                (s16)(RIGHT_ZERO_POS - angle_step)};
        sms_sts.SyncWritePosEx(init_ID, 2, init_Position, init_Speed, init_ACC);
        last_height_change_time = millis();
        Serial.print("🦿 初始变形 -> 高度: ");
        Serial.println(target_height);
    }
    delay(1000);

    motor1.linkSensor(&sensor1);
    motor2.linkSensor(&sensor2);
    driver1.voltage_power_supply = 8.0;
    driver2.voltage_power_supply = 8.0;
    driver1.init();
    driver2.init();
    motor1.linkDriver(&driver1);
    motor2.linkDriver(&driver2);
    motor1.torque_controller = TorqueControlType::voltage;
    motor2.torque_controller = TorqueControlType::voltage;   
    motor1.controller = MotionControlType::torque;
    motor2.controller = MotionControlType::torque;
    motor1.init();
    motor1.initFOC(); 
    motor2.init();
    motor2.initFOC();

    udp.begin(localPort);
    
    Serial.println("✅ 系统就绪!");
    Serial.println("💡 发送 '?' 查看所有可调参数");
    Serial.println("💡 发送 'CAL1' 进入标定模式");
}

// ====================================================================
// 👑 阶段 5:级联外环
// 调用频率:50Hz
// 职责:根据位置误差,算出目标速度
// ====================================================================
void update_outer_loop() {
    if (current_state == STATE_IDLE) {
        float pos_error = target_x - actual_x;
        
        if (abs(pos_error) < OUTER_DEAD_ZONE) {
            outer_target_v = 0.0;
            return;
        }
        
        // PD 外环
        float p_term = OUTER_KP * pos_error;
        float d_term = -OUTER_KD * fused_vel;  // 注意负号
        outer_target_v = p_term + d_term;
        
        if (outer_target_v > OUTER_MAX_V) outer_target_v = OUTER_MAX_V;
        if (outer_target_v < -OUTER_MAX_V) outer_target_v = -OUTER_MAX_V;
    }
    else {
        outer_target_v = 0.0;
    }
}

// ====================================================================
// 👑 阶段 5:状态机决策
// 调用时机:每个 LQR 周期(500Hz)
// 职责:根据 target_v 和当前状态,决定是否切换状态
// ====================================================================
void update_state_machine() {
    unsigned long now_ms = millis();
    
    switch (current_state) {
        case STATE_IDLE:
            // IDLE → 检测到速度指令 → TRANSITION → DRIVE
            /* if (abs(target_v) > 0.02) {
                // 立即响应,无防抖
                int_dx_at_transition_start = int_dx;
                transition_target_state = STATE_DRIVE;
                transition_start_time = now_ms;
                current_state = STATE_TRANSITION;
            } */
            break;
        
        case STATE_DRIVE:
            // DRIVE → target_v 归零 → 防抖 100ms → TRANSITION → IDLE
            if (abs(target_v) < 0.02) {
                if (drive_to_idle_timer == 0) {
                    drive_to_idle_timer = now_ms;  // 启动计时
                } else if (now_ms - drive_to_idle_timer > DRIVE_TO_IDLE_DEBOUNCE_MS) {
                    // 持续 100ms 都是 0,确认要停
                    int_dx_at_transition_start = int_dx;
                    transition_target_state = STATE_IDLE;
                    transition_start_time = now_ms;
                    current_state = STATE_TRANSITION;
                    drive_to_idle_timer = 0;
                    // 进入 IDLE 时锚定目标位置
                    target_x = actual_x;
                }
            } else {
                drive_to_idle_timer = 0;  // 复位计时
            }
            break;
        
        case STATE_TRANSITION:
            // TRANSITION → 200ms 后进入目标状态
            if (now_ms - transition_start_time > TRANSITION_DURATION_MS) {
                current_state = transition_target_state;
                transition_progress = 0.0;  // 清零调试值
            }
            break;
    }
}

// ====================================================================
// LOOP
// ====================================================================
void loop() {
    motor1.loopFOC();
    motor2.loopFOC();
    mpu6050.update();

    // -------- Serial1: 视觉指令 --------
    if (Serial1.available()) {
        String receivedCmd = Serial1.readStringUntil('\n');
        int firstComma = receivedCmd.indexOf(',');
        int secondComma = receivedCmd.indexOf(',', firstComma + 1);

        if (firstComma > 0 && secondComma > 0) {
            target_v = receivedCmd.substring(0, firstComma).toFloat();
            target_w = receivedCmd.substring(firstComma + 1, secondComma).toFloat();
            float new_height = receivedCmd.substring(secondComma + 1).toFloat();

            Serial.print("✅ [视觉] V="); Serial.print(target_v);
            Serial.print(" W="); Serial.print(target_w);
            Serial.print(" H="); Serial.println(new_height);
            
            execute_height_change(new_height);
        }
    }

    // -------- 光流读取 --------
    if (millis() - last_flow_time >= 10) {
        int16_t dX, dY;
        flow.readMotionCount(&dX, &dY);
        float raw_forward_pixel_speed = dY; 
        float raw_flow_vel = raw_forward_pixel_speed * FLOW_SCALE_FACTOR;
        flow_vel_y = flow_vel_y * 0.9 + raw_flow_vel * 0.1;
        last_flow_time = millis();
    }

    // -------- 50Hz 外环更新 --------
    if (millis() - last_outer_loop_time >= OUTER_LOOP_PERIOD_MS) {
        last_outer_loop_time = millis();
        update_outer_loop();
    }

    // -------- 500Hz LQR 控制环 --------
    unsigned long now = micros();
    float dt = (now - last_lqr_time) / 1000000.0f;
    
    if (dt >= 0.002) { 
        last_lqr_time = now;

        // 传感器读取
        float theta = (mpu6050.getAngleY() - ANGLE_OFFSET) * (PI / 180.0);
        
        // 👑 陀螺仪零偏(阶段 1 启用,阶段 0 透传)
        float raw_dtheta = (mpu6050.getGyroY() - gyro_y_bias) * (PI / 180.0);
        filtered_dtheta = filtered_dtheta * 0.8 + raw_dtheta * 0.2; 
        float dtheta = filtered_dtheta;

        float raw_spd = -0.5 * (motor1.shaft_velocity + motor2.shaft_velocity);
        filtered_spd = filtered_spd * 0.9 + raw_spd * 0.1;
        dx = filtered_spd * 0.0265; 

        fused_vel = (FLOW_WEIGHT * flow_vel_y) + ((1.0 - FLOW_WEIGHT) * dx);

        float current_k_global = get_dynamic_K_global(target_height);
        float base_offset = get_dynamic_Offset(target_height);

        float v_error = fused_vel - target_v;
        actual_x += fused_vel * dt;

        // ============================================================
        // 👑 阶段 0 关键设计:CALIBRATION_MODE 分支
        // ============================================================
        float u_total = 0.0;
        
        if (CALIBRATION_MODE) {
            // 1. 彻底切断外部干预与在线补偿
            target_v = 0.0;
            target_w = 0.0;
            u_compensation = 0.0; 
            
            float cal_v_error = fused_vel - 0.0;

            // 2. 移除严苛的重置逻辑，死死锚定 target_x
            // 仅当机甲面临彻底摔倒（倾角 > 25度）时才松开船锚，防止积分暴走危险
            if (abs(mpu6050.getAngleY()) > 60.0) {
                target_x = actual_x;
                int_dx = 0.0;
            } else {
                // 纯粹的物理弹簧逻辑，只要没摔倒，就绝不更新 target_x
                float pos_error = target_x - actual_x; 
                if (abs(pos_error) < 0.1) pos_error = 0;
                int_dx = pos_error ;
            }
            
            // 3. 放宽积分（弹簧拉力）限幅，允许机甲在标定期间产生更大的对抗力矩
            if (int_dx > 1.0) int_dx = 1.0;     // ← 限幅从 5 降到 1
            if (int_dx < -1.0) int_dx = -1.0;

            // 4. 执行带有强位置约束的 LQR 计算
            float raw_u = -(K_speed * cal_v_error + K_pitch_rate * dtheta 
                        + K_pitch * theta + K_speed_int * int_dx);
            u_total = raw_u * current_k_global;
        }
        else {
            // ============================================================
            // 正常模式:阶段 5 状态机
            // ============================================================
            
            // 1. 更新状态机
            update_state_machine();
            
            // 2. 根据当前状态决定 int_dx 的计算方式
            if (current_state == STATE_IDLE) {
                // IDLE: 级联控制 - 跟踪外环算出的 outer_target_v
                if (abs(mpu6050.getAngleY()) > 60.0) {
                    // 安全保护:姿态太大,不工作
                    int_dx = 0.0;
                    target_x = actual_x;
                } else {
                    // 🐰 虚拟兔子:target_x 以 target_v 速度匀速爬
                    target_x += target_v * dt;

                    // 内环逻辑:和 DRIVE 完全一样,只是 target_v 来源不同
                    effective_target_v = outer_target_v;
                    float idle_v_error = fused_vel - effective_target_v;
                    int_dx += (0.0 - idle_v_error) * dt;
                }
            }
            else if (current_state == STATE_DRIVE) {
                // DRIVE: 速度积分,跟踪端侧 target_v
                target_x = actual_x;  // 跟随,不留位置目标
                effective_target_v = target_v;
                float drive_v_error = fused_vel - effective_target_v;
                int_dx += (0.0 - drive_v_error) * dt;
            }
            else if (current_state == STATE_TRANSITION) {
                // TRANSITION: effective_target_v 在两个目标速度之间插值
                transition_progress = (millis() - transition_start_time) 
                                    / (float)TRANSITION_DURATION_MS;
                if (transition_progress > 1.0) transition_progress = 1.0;
                
                float v_source, v_target;
                if (transition_target_state == STATE_IDLE) {
                    // DRIVE → IDLE:从端侧 target_v 过渡到外环 outer_target_v
                    v_source = target_v;       // 这时 target_v 应该接近 0
                    v_target = outer_target_v;
                } else {
                    // IDLE → DRIVE:从外环 outer_target_v 过渡到端侧 target_v
                    v_source = outer_target_v;
                    v_target = target_v;
                }
                
                effective_target_v = (1.0 - transition_progress) * v_source 
                                + transition_progress * v_target;
                
                // int_dx 继续做速度积分(像 DRIVE 那样)
                // float trans_v_error = fused_vel - effective_target_v;
                // int_dx += (0.0 - trans_v_error) * dt;

                float trans_v_error = fused_vel - effective_target_v;
                int_dx += trans_v_error * dt;

                //int_dx = int_dx_at_transition_start * (1.0 - transition_progress);
            }
            
            // 3. 统一限幅
            if (int_dx > 1.0) int_dx = 1.0;
            if (int_dx < -1.0) int_dx = -1.0;
            
            // 4. LQR 主计算:用 effective_target_v 算 v_error
            float lqr_v_error;
            if (current_state == STATE_IDLE) {
                lqr_v_error = fused_vel - effective_target_v;
            } else if (current_state == STATE_DRIVE) {
                lqr_v_error = fused_vel - target_v;
            } else {
                // TRANSITION:用插值过的 effective_target_v
                lqr_v_error = fused_vel - effective_target_v;
            }

            float raw_u = -(K_speed * lqr_v_error + K_pitch_rate * dtheta 
                            + K_pitch * theta + K_speed_int * int_dx);
            float lqr_effort = raw_u * current_k_global;
            
            // 5. 在线补偿(改了 is_truly_stationary 条件)
            bool is_truly_stationary = 
                current_state == STATE_IDLE                          // 新增:只在 IDLE
                && abs(target_v) < 0.02
                && abs(mpu6050.getAngleY() - ANGLE_OFFSET) < 8.0
                && abs(fused_vel) < 0.1
                && (millis() - last_height_change_time > 2000);
            
            u_total = lqr_effort;
            
            if (ENABLE_COMPENSATION) {
                if (is_truly_stationary && comp_learn_rate > 0) {
                    //float pos_loop_effort = -K_speed_int * int_dx * current_k_global;
                    // u_compensation += pos_loop_effort * dt * comp_learn_rate;
                    float comp_signal = outer_target_v * K_speed * current_k_global ;
                    u_compensation += comp_signal * dt * comp_learn_rate;
                    
                    
                    if (u_compensation > 1.5) u_compensation = 1.5;
                    if (u_compensation < -1.5) u_compensation = -1.5;
                }
                u_total = lqr_effort + u_compensation;
            }
            
            // 6. 陀螺仪零偏在线学习(不变)
            if (ENABLE_GYRO_BIAS_LEARN && is_truly_stationary) {
                gyro_y_bias = gyro_y_bias * 0.999 + mpu6050.getGyroY() * 0.001;
            }
            
        }

        // 限幅 & 安全
        if (u_total > 7.4) u_total = 7.4;
        if (u_total < -7.4) u_total = -7.4;
        if (abs(mpu6050.getAngleY()) > 30.0) {
            u_total = 0.0;
            u_compensation = 0.0;
        }

        global_u_total = u_total;
        motor1.target = -u_total + target_w; 
        motor2.target = -u_total - target_w;
    }

    motor1.move();
    motor2.move();

    // -------- 50Hz UDP 上报 --------
    if (now - last_print_time >= 20000) { 
        last_print_time = now;
        
        uint32_t raw_adc = analogRead(BAT_PIN);
        uint32_t cal_voltage_mv = esp_adc_cal_raw_to_voltage(raw_adc, &adc_chars); 
        float actual_voltage = (cal_voltage_mv * 3.97) / 1000.0;

        udp.beginPacket(udpAddress, udpPort);
        udp.print("{");
        udp.print("\"Angle\":");    udp.print(mpu6050.getAngleY(), 3);
        udp.print(",\"Gyro\":");    udp.print(mpu6050.getGyroY(), 3);
        udp.print(",\"Theta\":");   udp.print((mpu6050.getAngleY() - ANGLE_OFFSET), 3);
        udp.print(",\"Torque\":");  udp.print(global_u_total, 3);
        udp.print(",\"Offset\":");  udp.print(ANGLE_OFFSET, 3);
        udp.print(",\"Integral\":");udp.print(- K_speed_int * int_dx, 3);
        udp.print(",\"IntDx\":");   udp.print(int_dx, 3);
        udp.print(",\"uComp\":");   udp.print(u_compensation, 3);
        udp.print(",\"Voltage\":"); udp.print(actual_voltage, 2);
        udp.print(",\"EncVel\":");  udp.print(dx, 3);
        udp.print(",\"OptVel\":");  udp.print(flow_vel_y, 3);
        udp.print(",\"FusedVel\":");udp.print(fused_vel, 3);
        udp.print(",\"ActualX\":"); udp.print(actual_x, 3);
        udp.print(",\"TargetX\":"); udp.print(target_x, 3);
        udp.print(",\"PosErr\":");  udp.print(target_x - actual_x, 3);
        udp.print(",\"GBias\":");   udp.print(gyro_y_bias, 3);
        udp.print(",\"State\":");   udp.print(current_state);
        udp.print(",\"CalMode\":"); udp.print(CALIBRATION_MODE ? 1 : 0);
        udp.print(",\"SinceMorph\":");udp.print((millis() - last_height_change_time) / 1000.0, 1);
        udp.print(",\"TransProg\":"); udp.print(transition_progress, 2);
        udp.print(",\"TargetV\":");   udp.print(target_v, 3);  // 方便看指令
        udp.print(",\"Damping\":"); udp.print(KD_POS * fused_vel, 3);
        udp.print(",\"OuterTargetV\":"); udp.print(outer_target_v, 3);
        udp.print(",\"EffectiveV\":");   udp.print(effective_target_v, 3);
        udp.print(",\"OUTER_MAX_V\":");   udp.print(OUTER_MAX_V, 3);
        udp.print("}");
        udp.println();
        udp.endPacket();
    }

    // -------- UDP 指令监听 --------
    int packetSize = udp.parsePacket();
    if (packetSize) {
        int len = udp.read(packetBuffer, 254);
        if (len > 0) {
            packetBuffer[len] = 0;
            String receivedCmd = String(packetBuffer);
            parse_remote_command(receivedCmd);
        }
    }
}