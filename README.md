# SH5001 六轴惯性测量单元 ESP32-C3 Arduino 示例

本项目提供了在ESP32-C3微控制器上使用SH5001六轴惯性测量单元(IMU)的Arduino示例代码。

## 硬件要求

- **微控制器**: ESP32-C3
- **传感器**: SH5001 六轴IMU (加速度计 + 陀螺仪)
- **通信接口**: I2C

## 接线图

```
ESP32-C3    SH5001
--------    ------
GPIO9   --> SDA
GPIO8   --> SCL
3.3V    --> VDD
GND     --> GND
```

**重要**: SH5001的SDO引脚状态决定I2C地址：
- SDO接GND: I2C地址 = 0x36
- SDO接VDD: I2C地址 = 0x37

## 文件说明

### 1. SH5001_Simple_Example.ino
**推荐新手使用**

这是一个简化的示例，专注于基本的六轴数据读取功能：

**特点:**
- 代码简洁易懂
- 基本的初始化和配置
- 实时读取并显示加速度计和陀螺仪数据
- 包含详细的注释和使用说明

**输出格式:**
```
ACC: 0.012 -0.003 0.998 g | GYRO: 0.5 -0.2 0.1 °/s
```

### 2. SH5001_ESP32C3_Example.ino
**功能完整版本**

这是一个功能更完整的示例，包含温度读取和更详细的配置：

**特点:**
- 完整的传感器配置
- 温度数据读取
- 结构化的数据处理
- 更详细的错误处理

### 3. SH5001.h 和 SH5001.C
**原厂驱动文件**

这些是Senodia提供的原始驱动文件，包含完整的功能实现：
- 完整的寄存器定义
- 高级功能支持 (FIFO、中断等)
- 多种配置选项

## 快速开始

1. **硬件连接**
   - 按照接线图连接ESP32-C3和SH5001：
     - GPIO9 → SDA
     - GPIO8 → SCL
     - 3.3V → VDD
     - GND → GND
   - 确认电源和地线连接正确

2. **软件设置**
   - 打开Arduino IDE
   - 选择ESP32-C3开发板
   - 打开 `SH5001_Simple_Example.ino`

3. **配置I2C地址**
   - 检查SH5001的SDO引脚连接
   - 在代码中设置正确的I2C地址：
     ```cpp
     #define SH5001_ADDRESS 0x37  // SDO=VDD
     // 或
     #define SH5001_ADDRESS 0x36  // SDO=GND
     ```

4. **上传和运行**
   - 编译并上传代码到ESP32-C3
   - 打开串口监视器 (波特率115200)
   - 观察输出的六轴数据

## 数据解释

### 加速度计数据
- **单位**: g (重力加速度，1g ≈ 9.8 m/s²)
- **量程**: ±16g
- **静止状态**: Z轴应显示约±1g (取决于传感器方向)，X和Y轴接近0g

### 陀螺仪数据
- **单位**: °/s (度每秒)
- **量程**: ±2000°/s
- **静止状态**: 三轴都应接近0°/s

### 温度数据
- **单位**: °C (摄氏度)
- **精度**: 约±1°C

## 故障排除

### 常见问题

1. **芯片ID读取失败**
   ```
   无法读取芯片ID
   ```
   **解决方案:**
   - 检查I2C接线 (SDA, SCL)
   - 检查电源连接 (VDD, GND)
   - 确认I2C地址设置正确

2. **芯片ID不匹配**
   ```
   芯片ID不匹配，期望值: 0xA1
   ```
   **解决方案:**
   - 确认使用的是SH5001传感器
   - 检查传感器是否损坏

3. **数据读取异常**
   - 数据全为0或异常值
   
   **解决方案:**
   - 检查传感器配置
   - 尝试降低I2C时钟频率
   - 增加延时时间

### 调试技巧

1. **I2C扫描**
   ```cpp
   // 添加到setup()中进行I2C设备扫描
   for (uint8_t addr = 1; addr < 127; addr++) {
     Wire.beginTransmission(addr);
     if (Wire.endTransmission() == 0) {
       Serial.print("发现I2C设备: 0x");
       Serial.println(addr, HEX);
     }
   }
   ```

2. **寄存器读取测试**
   ```cpp
   uint8_t test_data;
   if (readRegister(0x1F, &test_data)) {
     Serial.print("芯片ID: 0x");
     Serial.println(test_data, HEX);
   }
   ```

## 扩展功能

### 1. 数据滤波
可以添加低通滤波器来平滑数据：
```cpp
float alpha = 0.1;
filtered_acc_x = alpha * acc_x_g + (1 - alpha) * filtered_acc_x;
```

### 2. 姿态计算
使用加速度计数据计算倾斜角度：
```cpp
float pitch = atan2(acc_y_g, sqrt(acc_x_g*acc_x_g + acc_z_g*acc_z_g)) * 180.0/PI;
float roll = atan2(-acc_x_g, acc_z_g) * 180.0/PI;
```

### 3. 中断功能
配置SH5001的中断功能来实现数据就绪通知或运动检测。

### 4. FIFO模式
使用FIFO模式批量读取数据，减少I2C通信次数。

## 技术支持

如果遇到问题，请检查：
1. 硬件连接是否正确
2. I2C地址设置是否匹配
3. 电源电压是否稳定 (3.3V)
4. 代码配置是否正确

## 许可证

本示例代码基于Senodia官方驱动修改，仅供学习和开发使用。
