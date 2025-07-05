/*
 * SH5001 六轴惯性测量单元 ESP32-C3 Arduino 示例
 * 
 * 功能：读取SH5001的加速度计和陀螺仪数据及位姿角度
 * 硬件：ESP32-C3 + SH5001 IMU
 * 通信：I2C接口
 * 
 * 接线：
 * ESP32-C3    SH5001
 * GPIO8  -->  SDA
 * GPIO9  -->  SCL
 * 3.3V   -->  VDD
 * GND    -->  GND
 * 
 * 作者：少司命  基于Senodia SH5001驱动修改
 * 日期：2025/06/26 23:39
 */

#include <Wire.h>

// SH5001 I2C地址
#define SH5001_ADDRESS 0x37  // SDO接VDD时为0x37，接GND时为0x36

// SH5001寄存器地址
#define SH5001_CHIP_ID      0x1F
#define SH5001_ACC_XL       0x00
#define SH5001_ACC_XH       0x01
#define SH5001_ACC_YL       0x02
#define SH5001_ACC_YH       0x03
#define SH5001_ACC_ZL       0x04
#define SH5001_ACC_ZH       0x05
#define SH5001_GYRO_XL      0x06
#define SH5001_GYRO_XH      0x07
#define SH5001_GYRO_YL      0x08
#define SH5001_GYRO_YH      0x09
#define SH5001_GYRO_ZL      0x0A
#define SH5001_GYRO_ZH      0x0B
#define SH5001_TEMP_ZL      0x0C
#define SH5001_TEMP_ZH      0x0D

// 配置寄存器
#define SH5001_ACC_CONF0    0x20
#define SH5001_ACC_CONF1    0x21
#define SH5001_ACC_CONF2    0x22
#define SH5001_GYRO_CONF0   0x23
#define SH5001_GYRO_CONF1   0x24
#define SH5001_GYRO_CONF2   0x25
#define SH5001_TEMP_CONF0   0x28
#define SH5001_POWER_MODE   0x30

// 配置参数
#define SH5001_ACC_ODR_125HZ    0x03
#define SH5001_ACC_RANGE_16G    0x30
#define SH5001_ACC_ODRX040      0x00
#define SH5001_ACC_FILTER_EN    0x01
#define SH5001_ACC_BYPASS_EN    0x02

#define SH5001_GYRO_ODR_125HZ   0x03
#define SH5001_GYRO_RANGE_2000  0x60
#define SH5001_GYRO_ODRX040     0x00
#define SH5001_GYRO_FILTER_EN   0x01
#define SH5001_GYRO_BYPASS_EN   0x02

#define SH5001_TEMP_ODR_63HZ    0x06
#define SH5001_TEMP_EN          0x01

// ESP32-C3 I2C引脚定义
#define SDA_PIN 8
#define SCL_PIN 9

// 数据结构
struct IMU_Data {
  int16_t acc_x, acc_y, acc_z;    // 加速度计数据 (原始值)
  int16_t gyro_x, gyro_y, gyro_z; // 陀螺仪数据 (原始值)
  float acc_x_g, acc_y_g, acc_z_g;       // 加速度计数据 (g)
  float gyro_x_dps, gyro_y_dps, gyro_z_dps; // 陀螺仪数据 (度/秒)
  float temperature;               // 温度 (摄氏度)
  float pitch, roll, yaw;          // 欧拉角 (度)
  float q[4];                      // 四元数 (w, x, y, z)
};

IMU_Data imu_data;
unsigned long last_time = 0;
float gyro_angle[3] = {0.0f, 0.0f, 0.0f};
float alpha = 0.98; // 互补滤波系数

// 函数声明
bool SH5001_Init();
bool SH5001_ReadByte(uint8_t reg, uint8_t* data);
bool SH5001_WriteByte(uint8_t reg, uint8_t data);
bool SH5001_ReadBytes(uint8_t reg, uint8_t* data, uint8_t len);
void SH5001_GetIMUData(IMU_Data* data);
float SH5001_GetTemperature();
void SH5001_SoftReset();
void SH5001_ConfigAccelerometer();
void SH5001_ConfigGyroscope();
void SH5001_ConfigTemperature();
void SH5001_CalculateAttitude(IMU_Data* data);
void normalizeQuaternion(float q[4]);
void updateQuaternionFromGyro(IMU_Data* data, float dt);
void correctQuaternionFromAccel(IMU_Data* data);


void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("SH5001 ESP32-C3 six-axis IMU example");
  Serial.println("I2C initialization...");
  
  // 初始化I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000); // 400kHz
  
  // 初始化SH5001
  if (SH5001_Init()) {
    Serial.println("SH5001 initialized successed!");
  } else {
    Serial.println("SH5001 initialized failed!");
    while(1) {
      delay(1000);
    }
  }
  
  Serial.println("reading IMU data now...");
  Serial.println("format: AccX(g) AccY(g) AccZ(g) GyroX(°/s) GyroY(°/s) GyroZ(°/s) Pitch(°) Roll(°) Yaw(°) Temp(°C)");
  
  // 初始化四元数为单位四元数
  imu_data.q[0] = 1.0f;  // w
  imu_data.q[1] = 0.0f;  // x
  imu_data.q[2] = 0.0f;  // y
  imu_data.q[3] = 0.0f;  // z

  last_time = millis();
}

void loop() {
  // 读取IMU数据
  SH5001_GetIMUData(&imu_data);
  
  // 计算姿态角
  SH5001_CalculateAttitude(&imu_data);
  
  // 打印数据（此处默认输出原始数据，如需输出四元数，可自行修改注释代码部分）
  
  //以下为输出原始数据部分
  Serial.print("ACC: ");
  Serial.print(imu_data.acc_x_g, 3);
  Serial.print(", ");
  Serial.print(imu_data.acc_y_g, 3);
  Serial.print(", ");
  Serial.print(imu_data.acc_z_g, 3);
  Serial.print(" g  |  GYRO: ");
  Serial.print(imu_data.gyro_x_dps, 2);
  Serial.print(", ");
  Serial.print(imu_data.gyro_y_dps, 2);
  Serial.print(", ");
  Serial.print(imu_data.gyro_z_dps, 2);
  Serial.print(" °/s  |  rangle: ");
  Serial.print(imu_data.pitch, 2);
  Serial.print(", ");
  Serial.print(imu_data.roll, 2);
  Serial.print(", ");
  Serial.print(imu_data.yaw, 2);
  Serial.print(" °  |  TEMP: ");
  Serial.print(imu_data.temperature, 1);
  Serial.println(" °C");

  // 以下为输出四元数部分
  // Serial.print(imu_data.q[0], 4);
  // Serial.print(",");
  // Serial.print(imu_data.q[1], 4);
  // Serial.print(",");
  // Serial.print(imu_data.q[2], 4);
  // Serial.print(",");
  // Serial.print(imu_data.q[3], 4);
  // Serial.print("\n");
  
  delay(10); // 100Hz输出频率（可自行修改）
}

// 添加四元数归一化函数
void normalizeQuaternion(float q[4]) {
  float norm = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
  if (norm > 0.0f) {
    q[0] /= norm;
    q[1] /= norm;
    q[2] /= norm;
    q[3] /= norm;
  }
}

// 计算姿态角（欧拉角）
void SH5001_CalculateAttitude(IMU_Data* data) {
  unsigned long current_time = millis();
  float dt = (current_time - last_time) / 1000.0f; // 转换为秒
  last_time = current_time;
  
  // 从加速度计计算俯仰角和横滚角
  float acc_pitch = atan2(data->acc_y_g, sqrt(data->acc_x_g * data->acc_x_g + data->acc_z_g * data->acc_z_g)) * 180.0 / PI;
  float acc_roll = atan2(-data->acc_x_g, data->acc_z_g) * 180.0 / PI;
  
  // 陀螺仪积分计算角度变化
  gyro_angle[0] += data->gyro_x_dps * dt;
  gyro_angle[1] += data->gyro_y_dps * dt;
  gyro_angle[2] += data->gyro_z_dps * dt;
  
  // 互补滤波融合加速度计和陀螺仪数据
  data->pitch = alpha * (data->pitch + data->gyro_x_dps * dt) + (1.0 - alpha) * acc_pitch;
  data->roll = alpha * (data->roll + data->gyro_y_dps * dt) + (1.0 - alpha) * acc_roll;
  data->yaw = gyro_angle[2]; // 偏航角只能通过陀螺仪积分或磁力计获得

  // 更新四元数
  updateQuaternionFromGyro(data, dt);
  
  // 使用加速度计数据修正四元数
  correctQuaternionFromAccel(data);
}

// 从陀螺仪数据更新四元数
void updateQuaternionFromGyro(IMU_Data* data, float dt) {
  // 将角速度从度/秒转换为弧度/秒
  float gx = data->gyro_x_dps * PI / 180.0f;
  float gy = data->gyro_y_dps * PI / 180.0f;
  float gz = data->gyro_z_dps * PI / 180.0f;
  
  // 四元数积分
  float qw = data->q[0];
  float qx = data->q[1];
  float qy = data->q[2];
  float qz = data->q[3];
  
  // 四元数导数
  float dqw = 0.5f * (-qx * gx - qy * gy - qz * gz);
  float dqx = 0.5f * (qw * gx + qy * gz - qz * gy);
  float dqy = 0.5f * (qw * gy - qx * gz + qz * gx);
  float dqz = 0.5f * (qw * gz + qx * gy - qy * gx);
  
  // 更新四元数
  data->q[0] += dqw * dt;
  data->q[1] += dqx * dt;
  data->q[2] += dqy * dt;
  data->q[3] += dqz * dt;
  
  // 归一化四元数
  normalizeQuaternion(data->q);
}

// 使用加速度计数据修正四元数
void correctQuaternionFromAccel(IMU_Data* data) {
  // 重力向量归一化
  float ax = data->acc_x_g;
  float ay = data->acc_y_g;
  float az = data->acc_z_g;
  float norm = sqrt(ax*ax + ay*ay + az*az);
  
  if (norm < 0.0001f) return; // 避免除零错误
  
  ax /= norm;
  ay /= norm;
  az /= norm;
  
  // 从当前四元数计算重力向量
  float qw = data->q[0];
  float qx = data->q[1];
  float qy = data->q[2];
  float qz = data->q[3];
  
  // 预期的重力向量 (0, 0, 1) 通过四元数旋转后的结果
  float vx = 2.0f * (qx*qz - qw*qy);
  float vy = 2.0f * (qw*qx + qy*qz);
  float vz = qw*qw - qx*qx - qy*qy + qz*qz;
  
  // 计算叉积 (测量的重力向量 × 预期的重力向量)
  float ex = ay*vz - az*vy;
  float ey = az*vx - ax*vz;
  float ez = ax*vy - ay*vx;
  
  // 应用修正 (比例积分控制)
  float Kp = 0.1f; // 比例系数
  
  // 更新四元数
  data->q[0] += Kp * (-qy*ez + qz*ey - qx*ex) * 0.5f;
  data->q[1] += Kp * (-qz*ex + qw*ez - qy*ey) * 0.5f;
  data->q[2] += Kp * (-qw*ey - qx*ez + qz*ex) * 0.5f;
  data->q[3] += Kp * (qx*ey - qy*ex + qw*ez) * 0.5f;
  
  // 归一化四元数
  normalizeQuaternion(data->q);
}


// SH5001初始化函数
bool SH5001_Init() {
  uint8_t chip_id;
  
  // 读取芯片ID
  if (!SH5001_ReadByte(SH5001_CHIP_ID, &chip_id)) {
    Serial.println("can't read IMU ID");
    return false;
  }
  
  if (chip_id != 0xA1) {
    Serial.print("IMU ID error: 0x");
    Serial.println(chip_id, HEX);
    return false;
  }
  
  Serial.print("IMU ID correct: 0x");
  Serial.println(chip_id, HEX);
  
  // 软复位
  SH5001_SoftReset();
  delay(100);
  
  // 配置加速度计
  SH5001_ConfigAccelerometer();
  
  // 配置陀螺仪
  SH5001_ConfigGyroscope();
  
  // 配置温度传感器
  SH5001_ConfigTemperature();
  
  delay(50); // 等待配置生效
  
  return true;
}

// 软复位
void SH5001_SoftReset() {
  SH5001_WriteByte(0x2B, 0x01);
  SH5001_WriteByte(0x00, 0x73);
  delay(50);
}

// 配置加速度计
void SH5001_ConfigAccelerometer() {
  uint8_t reg_data;
  
  // 配置ACC_CONF0: 启用滤波器和旁路
  SH5001_ReadByte(SH5001_ACC_CONF0, &reg_data);
  reg_data = (reg_data & 0xFC) | SH5001_ACC_FILTER_EN | SH5001_ACC_BYPASS_EN;
  SH5001_WriteByte(SH5001_ACC_CONF0, reg_data);
  
  // 配置ACC_CONF1: ODR和量程
  SH5001_ReadByte(SH5001_ACC_CONF1, &reg_data);
  reg_data = (reg_data & 0x80) | SH5001_ACC_ODR_125HZ | SH5001_ACC_RANGE_16G;
  SH5001_WriteByte(SH5001_ACC_CONF1, reg_data);
  
  // 配置ACC_CONF2: 截止频率
  SH5001_ReadByte(SH5001_ACC_CONF2, &reg_data);
  reg_data = (reg_data & 0xF0) | SH5001_ACC_ODRX040;
  SH5001_WriteByte(SH5001_ACC_CONF2, reg_data);
}

// 配置陀螺仪
void SH5001_ConfigGyroscope() {
  uint8_t reg_data;
  
  // 配置GYRO_CONF0: 启用滤波器和旁路
  SH5001_ReadByte(SH5001_GYRO_CONF0, &reg_data);
  reg_data = (reg_data & 0x7C) | SH5001_GYRO_FILTER_EN | SH5001_GYRO_BYPASS_EN;
  SH5001_WriteByte(SH5001_GYRO_CONF0, reg_data);
  
  // 配置GYRO_CONF1: ODR和量程
  SH5001_ReadByte(SH5001_GYRO_CONF1, &reg_data);
  reg_data = (reg_data & 0x80) | SH5001_GYRO_ODR_125HZ | SH5001_GYRO_RANGE_2000;
  SH5001_WriteByte(SH5001_GYRO_CONF1, reg_data);
  
  // 配置GYRO_CONF2: 截止频率
  SH5001_ReadByte(SH5001_GYRO_CONF2, &reg_data);
  reg_data = (reg_data & 0xF0) | SH5001_GYRO_ODRX040;
  SH5001_WriteByte(SH5001_GYRO_CONF2, reg_data);
}

// 配置温度传感器
void SH5001_ConfigTemperature() {
  uint8_t reg_data;

  SH5001_ReadByte(SH5001_TEMP_CONF0, &reg_data);
  reg_data = (reg_data & 0xF8) | SH5001_TEMP_ODR_63HZ | SH5001_TEMP_EN;
  SH5001_WriteByte(SH5001_TEMP_CONF0, reg_data);
}

// 读取IMU数据
void SH5001_GetIMUData(IMU_Data* data) {
  uint8_t raw_data[12];

  // 一次性读取12字节数据 (加速度计6字节 + 陀螺仪6字节)
  if (SH5001_ReadBytes(SH5001_ACC_XL, raw_data, 12)) {
    // 解析加速度计数据 (16位有符号整数，小端格式)
    data->acc_x = (int16_t)((raw_data[1] << 8) | raw_data[0]);
    data->acc_y = (int16_t)((raw_data[3] << 8) | raw_data[2]);
    data->acc_z = (int16_t)((raw_data[5] << 8) | raw_data[4]);

    // 解析陀螺仪数据 (16位有符号整数，小端格式)
    data->gyro_x = (int16_t)((raw_data[7] << 8) | raw_data[6]);
    data->gyro_y = (int16_t)((raw_data[9] << 8) | raw_data[8]);
    data->gyro_z = (int16_t)((raw_data[11] << 8) | raw_data[10]);

    // 转换为物理单位
    // 加速度计: ±16g量程，16位分辨率
    float acc_scale = 16.0 / 32768.0; // g/LSB
    data->acc_x_g = data->acc_x * acc_scale;
    data->acc_y_g = data->acc_y * acc_scale;
    data->acc_z_g = data->acc_z * acc_scale;

    // 陀螺仪: ±2000°/s量程，16位分辨率
    float gyro_scale = 2000.0 / 32768.0; // °/s/LSB
    data->gyro_x_dps = data->gyro_x * gyro_scale;
    data->gyro_y_dps = data->gyro_y * gyro_scale;
    data->gyro_z_dps = data->gyro_z * gyro_scale;
  }

  // 读取温度数据
  data->temperature = SH5001_GetTemperature();
}

// 读取温度数据
float SH5001_GetTemperature() {
  uint8_t temp_data[2];
  uint16_t temp_ref[2];

  // 读取温度参考值
  if (SH5001_ReadBytes(0x29, temp_data, 2)) { // TEMP_CONF1, TEMP_CONF2
    temp_ref[0] = ((uint16_t)(temp_data[1] & 0x0F) << 8) | temp_data[0];
  }

  // 读取温度数据
  if (SH5001_ReadBytes(SH5001_TEMP_ZL, temp_data, 2)) {
    temp_ref[1] = ((uint16_t)(temp_data[1] & 0x0F) << 8) | temp_data[0];

    // 计算温度 (根据SH5001数据手册公式)
    return (((float)(temp_ref[1] - temp_ref[0])) / 14.0) + 25.0;
  }

  return 0.0;
}

// I2C读取单个字节
bool SH5001_ReadByte(uint8_t reg, uint8_t* data) {
  Wire.beginTransmission(SH5001_ADDRESS);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) {
    return false;
  }

  Wire.requestFrom(SH5001_ADDRESS, 1);
  if (Wire.available()) {
    *data = Wire.read();
    return true;
  }

  return false;
}

// I2C写入单个字节
bool SH5001_WriteByte(uint8_t reg, uint8_t data) {
  Wire.beginTransmission(SH5001_ADDRESS);
  Wire.write(reg);
  Wire.write(data);
  return (Wire.endTransmission() == 0);
}

// I2C读取多个字节
bool SH5001_ReadBytes(uint8_t reg, uint8_t* data, uint8_t len) {
  Wire.beginTransmission(SH5001_ADDRESS);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) {
    return false;
  }

  Wire.requestFrom(SH5001_ADDRESS, len);
  uint8_t i = 0;
  while (Wire.available() && i < len) {
    data[i++] = Wire.read();
  }

  return (i == len);
}
