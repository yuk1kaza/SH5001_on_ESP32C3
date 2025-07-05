#ifndef __SH5001_H
#define __SH5001_H

// 可选功能定义 (根据需要取消注释)
//#define SH5001_FIFO_WM_INT      // FIFO水位中断
//#define SH5001_ACTIVITY_INT     // 活动检测中断
//#define SH5001_DATA_READY_INT   // 数据就绪中断

/******************************************************************
*	SH5001 I2C地址宏定义
*
*   (7位地址):    (0x37)011 0111@SDO=1;    (0x36)011 0110@SDO=0;
******************************************************************/
#define SH5001_ADDRESS				(0x37U)		// SH5001的I2C设备地址
#define SENODIA_VDD_3V3 1                       // 3.3V电源供电标志

/******************************************************************
*	SH5001 寄存器地址宏定义
******************************************************************/
// 数据寄存器地址定义
#define SH5001_ACC_XL				(0x00U)  // 加速度计X轴数据低字节
#define SH5001_ACC_XH				(0x01U)  // 加速度计X轴数据高字节
#define SH5001_ACC_YL				(0x02U)  // 加速度计Y轴数据低字节
#define SH5001_ACC_YH				(0x03U)  // 加速度计Y轴数据高字节
#define SH5001_ACC_ZL				(0x04U)  // 加速度计Z轴数据低字节
#define SH5001_ACC_ZH				(0x05U)  // 加速度计Z轴数据高字节
#define SH5001_GYRO_XL				(0x06U)  // 陀螺仪X轴数据低字节
#define SH5001_GYRO_XH				(0x07U)  // 陀螺仪X轴数据高字节
#define SH5001_GYRO_YL				(0x08U)  // 陀螺仪Y轴数据低字节
#define SH5001_GYRO_YH				(0x09U)  // 陀螺仪Y轴数据高字节
#define SH5001_GYRO_ZL				(0x0AU)  // 陀螺仪Z轴数据低字节
#define SH5001_GYRO_ZH				(0x0BU)  // 陀螺仪Z轴数据高字节
#define SH5001_TEMP_ZL				(0x0CU)  // 温度数据低字节
#define SH5001_TEMP_ZH				(0x0DU)  // 温度数据高字节
#define SH5001_TIMESTAMP_L			(0x0EU)  // 时间戳低字节
#define SH5001_TIMESTAMP_M			(0x0FU)  // 时间戳中字节
#define SH5001_TIMESTAMP_H			(0x10U)  // 时间戳高字节

// 状态寄存器地址定义
#define SH5001_INT_STA0				(0x16U)  // 中断状态寄存器0
#define SH5001_INT_STA1				(0x17U)  // 中断状态寄存器1
#define SH5001_INT_STA2				(0x18U)  // 中断状态寄存器2
#define SH5001_INT_STA3				(0x19U)  // 中断状态寄存器3
#define SH5001_INT_STA4				(0x1AU)  // 中断状态寄存器4
#define SH5001_FIFO_STA0			(0x1BU)  // FIFO状态寄存器0
#define SH5001_FIFO_STA1			(0x1CU)  // FIFO状态寄存器1
#define SH5001_FIFO_DATA			(0x1DU)  // FIFO数据寄存器

#define SH5001_CHIP_ID				(0x1FU)  // 芯片ID寄存器

// 加速度计配置寄存器
#define SH5001_ACC_CONF0			(0x20U)  // 加速度计配置寄存器0
#define SH5001_ACC_CONF1			(0x21U)  // 加速度计配置寄存器1
#define SH5001_ACC_CONF2			(0x22U)  // 加速度计配置寄存器2

// 陀螺仪配置寄存器
#define SH5001_GYRO_CONF0			(0x23U)  // 陀螺仪配置寄存器0
#define SH5001_GYRO_CONF1			(0x24U)  // 陀螺仪配置寄存器1
#define SH5001_GYRO_CONF2			(0x25U)  // 陀螺仪配置寄存器2

// OIS(光学防抖)配置寄存器
#define SH5001_OIS_ACC_CONF			(0x26U)  // OIS加速度计配置寄存器
#define SH5001_OIS_GYRO_CONF		(0x27U)  // OIS陀螺仪配置寄存器

// 温度和时间戳配置寄存器
#define SH5001_TEMP_CONF0			(0x28U)  // 温度配置寄存器0
#define SH5001_ACC_DOWNSAMPLE		(0x28U)  // 加速度计降采样寄存器(与TEMP_CONF0共用地址)
#define SH5001_TEMP_CONF1			(0x29U)  // 温度配置寄存器1
#define SH5001_TEMP_CONF2			(0x2AU)  // 温度配置寄存器2
#define SH5001_TIMESTAMP_CONF		(0x2AU)  // 时间戳配置寄存器(与TEMP_CONF2共用地址)

// 电源管理寄存器
#define SH5001_POWER_MODE			(0x30U)  // 电源模式寄存器
#define SH5001_POWER_MODE_PWM		(0x31U)  // PWM电源模式寄存器

// 接口配置寄存器
#define SH5001_I2C_CONF				(0x34U)  // I2C接口配置寄存器
#define SH5001_SPI_CONF				(0x34U)  // SPI接口配置寄存器(与I2C_CONF共用地址)
#define SH5001_I2C_READ_LOOP		(0x34U)  // I2C读取循环寄存器(与I2C_CONF共用地址)
#define SH5001_OIS_SPI_CONF			(0x34U)  // OIS SPI配置寄存器(与I2C_CONF共用地址)


// FIFO配置寄存器
#define SH5001_FIFO_CONF0			(0x35U)  // FIFO配置寄存器0
#define SH5001_FIFO_CONF1			(0x36U)  // FIFO配置寄存器1
#define SH5001_FIFO_CONF2			(0x37U)  // FIFO配置寄存器2
#define SH5001_FIFO_CONF3			(0x38U)  // FIFO配置寄存器3
#define SH5001_FIFO_CONF4			(0x39U)  // FIFO配置寄存器4

// 主I2C接口寄存器
#define SH5001_MI2C_CONF0			(0x3AU)  // 主I2C配置寄存器0
#define SH5001_MI2C_CONF1			(0x3BU)  // 主I2C配置寄存器1
#define SH5001_MI2C_CMD0			(0x3CU)  // 主I2C命令寄存器0
#define SH5001_MI2C_CMD1			(0x3DU)  // 主I2C命令寄存器1
#define SH5001_MI2C_WR				(0x3EU)  // 主I2C写数据寄存器
#define SH5001_MI2C_RD				(0x3FU)  // 主I2C读数据寄存器

// 中断配置寄存器
#define SH5001_INT_ENABLE0			(0x40U)  // 中断使能寄存器0
#define SH5001_INT_ENABLE1			(0x41U)  // 中断使能寄存器1
#define SH5001_INT_CONF				(0x42U)  // 中断配置寄存器
#define SH5001_INT_LIMIT			(0x43U)  // 中断限制寄存器

// 方向检测中断配置寄存器
#define SH5001_ORIEN_INTCONF0		(0x44U)  // 方向中断配置寄存器0
#define SH5001_ORIEN_INTCONF1		(0x45U)  // 方向中断配置寄存器1
#define SH5001_ORIEN_INT_1G5_LOW	(0x46U)  // 方向中断1.5G阈值低字节
#define SH5001_ORIEN_INT_1G5_HIGH	(0x47U)  // 方向中断1.5G阈值高字节
#define SH5001_ORIEN_INT_SLOPE_LOW	(0x48U)  // 方向中断斜率阈值低字节
#define SH5001_ORIEN_INT_SLOPE_HIGH	(0x49U)  // 方向中断斜率阈值高字节
#define SH5001_ORIEN_INT_HYST_LOW	(0x4AU)  // 方向中断滞后阈值低字节
#define SH5001_ORIEN_INT_HYST_HIGH	(0x4BU)  // 方向中断滞后阈值高字节

#define SH5001_FLAT_INT_CONF		(0x4CU)

#define SH5001_ACT_INACT_INT_CONF	(0x4DU)
#define SH5001_ACT_INACT_INT_LINK	(0x4EU)
#define SH5001_ACT_INT_THRESHOLDL	(0x54U)
#define SH5001_ACT_INT_THRESHOLDH	(0x55U)
#define SH5001_ACT_INT_TIME			(0x56U)

#define SH5001_INACT_INT_THRESHOLDL	(0x57U)
#define SH5001_INACT_INT_THRESHOLDH	(0x58U)
#define SH5001_INACT_INT_TIME		(0x59U)

#define SH5001_SMD_INT_THRESHOLDL	(0x5AU)
#define SH5001_SMD_INT_THRESHOLDH	(0x5BU)
#define SH5001_SMD_INT_TIME			(0x5CU)

#define SH5001_TAP_INT_CONF			(0x4EU)
#define SH5001_TAP_INT_THRESHOLDL	(0x4FU)
#define SH5001_TAP_INT_THRESHOLDH	(0x50U)
#define SH5001_TAP_INT_DURATION		(0x51U)
#define SH5001_TAP_INT_LATENCY		(0x52U)
#define SH5001_DTAP_INT_WINDOW		(0x53U)

#define SH5001_HIGHG_INT_CONF		(0x5DU)
#define SH5001_HIGHG_INT_THRESHOLDL	(0x5EU)
#define SH5001_HIGHG_INT_THRESHOLDH	(0x5FU)
#define SH5001_HIGHG_INT_TIME		(0x60U)

#define SH5001_LOWG_INT_CONF		(0x5DU)
#define SH5001_LOWG_INT_THRESHOLDL	(0x61U)
#define SH5001_LOWG_INT_THRESHOLDH	(0x62U)
#define SH5001_LOWG_INT_TIME		(0x63U)

#define SH5001_FREEFALL_INT_THRES	(0x64U)
#define SH5001_FREEFALL_INT_TIME	(0x65U)

#define SH5001_INT_PIN_MAP0			(0x66U)
#define SH5001_INT_PIN_MAP1			(0x67U)

#define SH5001_SPI_REG_ACCESS		(0x7FU)
#define SH5001_AUX_I2C_CONF			(0xFDU)

#define SH5001_PIN_PP_CONF0			(0xFDU)
#define SH5001_PIN_PP_CONF1			(0xFEU)







/******************************************************************
*	加速度计配置宏定义
******************************************************************/
// 加速度计输出数据率(ODR)配置 - 标准频率
#define SH5001_ACC_ODR_1000HZ	(0x00U)  // 1000Hz输出数据率
#define SH5001_ACC_ODR_500HZ	(0x01U)  // 500Hz输出数据率
#define SH5001_ACC_ODR_250HZ	(0x02U)  // 250Hz输出数据率
#define SH5001_ACC_ODR_125HZ	(0x03U)  // 125Hz输出数据率

// 加速度计输出数据率(ODR)配置 - 高频率
#define SH5001_ACC_ODR_2000HZ	(0x08U)  // 2000Hz输出数据率
#define SH5001_ACC_ODR_4000HZ	(0x09U)  // 4000Hz输出数据率
#define SH5001_ACC_ODR_8000HZ	(0x0AU)  // 8000Hz输出数据率

// 加速度计输出数据率(ODR)配置 - 备用频率设置
#define SH5001_ACC_ODR_880HZ	(0x00U)  // 880Hz输出数据率
#define SH5001_ACC_ODR_440HZ	(0x01U)  // 440Hz输出数据率
#define SH5001_ACC_ODR_220HZ	(0x02U)  // 220Hz输出数据率
#define SH5001_ACC_ODR_110HZ	(0x03U)  // 110Hz输出数据率

#define SH5001_ACC_ODR_1760HZ	(0x08U)  // 1760Hz输出数据率
#define SH5001_ACC_ODR_3520HZ	(0x09U)  // 3520Hz输出数据率
#define SH5001_ACC_ODR_7040HZ	(0x0AU)  // 7040Hz输出数据率

// 加速度计量程配置
#define SH5001_ACC_RANGE_4G		(0x10U)  // ±4G量程
#define SH5001_ACC_RANGE_8G		(0x20U)  // ±8G量程
#define SH5001_ACC_RANGE_16G	(0x30U)  // ±16G量程
//#define SH5001_ACC_RANGE_32G	(0x40U)  // ±32G量程(未启用)

// 加速度计低通滤波器截止频率配置(相对于ODR的倍数)
#define SH5001_ACC_ODRX040		(0x00U)  // ODR × 0.40 截止频率
#define SH5001_ACC_ODRX036		(0x01U)  // ODR × 0.36 截止频率
#define SH5001_ACC_ODRX032		(0x02U)  // ODR × 0.32 截止频率
#define SH5001_ACC_ODRX028		(0x03U)  // ODR × 0.28 截止频率
#define SH5001_ACC_ODRX024		(0x04U)  // ODR × 0.24 截止频率
#define SH5001_ACC_ODRX020		(0x05U)  // ODR × 0.20 截止频率
#define SH5001_ACC_ODRX016		(0x06U)  // ODR × 0.16 截止频率
#define SH5001_ACC_ODRX014		(0x07U)  // ODR × 0.14 截止频率
#define SH5001_ACC_ODRX012		(0x08U)  // ODR × 0.12 截止频率
#define SH5001_ACC_ODRX010		(0x09U)  // ODR × 0.10 截止频率
#define SH5001_ACC_ODRX008		(0x0aU)  // ODR × 0.08 截止频率
#define SH5001_ACC_ODRX006		(0x0bU)  // ODR × 0.06 截止频率
#define SH5001_ACC_ODRX004		(0x0cU)  // ODR × 0.04 截止频率
#define SH5001_ACC_ODRX003		(0x0dU)  // ODR × 0.03 截止频率
#define SH5001_ACC_ODRX002		(0x0eU)  // ODR × 0.02 截止频率
#define SH5001_ACC_ODRX001		(0x0fU)  // ODR × 0.01 截止频率

// 加速度计滤波器和旁路配置
#define SH5001_ACC_FILTER_EN	(0x01U)  // 使能数字滤波器
#define SH5001_ACC_FILTER_DIS	(0x00U)  // 禁用数字滤波器
#define SH5001_ACC_BYPASS_EN	(0x02U)  // 使能旁路模式
#define SH5001_ACC_BYPASS_DIS	(0x00U)  // 禁用旁路模式


/******************************************************************
*	陀螺仪配置宏定义
******************************************************************/
// 陀螺仪输出数据率(ODR)配置 - 标准频率
#define SH5001_GYRO_ODR_1000HZ	(0x00U)  // 1000Hz输出数据率
#define SH5001_GYRO_ODR_500HZ	(0x01U)  // 500Hz输出数据率
#define SH5001_GYRO_ODR_250HZ	(0x02U)  // 250Hz输出数据率
#define SH5001_GYRO_ODR_125HZ	(0x03U)  // 125Hz输出数据率

// 陀螺仪输出数据率(ODR)配置 - 高频率
#define SH5001_GYRO_ODR_2000HZ	(0x08U)  // 2000Hz输出数据率
#define SH5001_GYRO_ODR_4000HZ	(0x09U)  // 4000Hz输出数据率
#define SH5001_GYRO_ODR_8000HZ	(0x0AU)  // 8000Hz输出数据率
#define SH5001_GYRO_ODR_16000HZ	(0x0BU)  // 16000Hz输出数据率

// 陀螺仪输出数据率(ODR)配置 - 备用频率设置
#define SH5001_GYRO_ODR_880HZ	(0x00U)  // 880Hz输出数据率
#define SH5001_GYRO_ODR_440HZ	(0x01U)  // 440Hz输出数据率
#define SH5001_GYRO_ODR_220HZ	(0x02U)  // 220Hz输出数据率
#define SH5001_GYRO_ODR_110HZ	(0x03U)  // 110Hz输出数据率
#define SH5001_GYRO_ODR_1760HZ	(0x08U)  // 1760Hz输出数据率
#define SH5001_GYRO_ODR_3520HZ	(0x09U)  // 3520Hz输出数据率
#define SH5001_GYRO_ODR_7040HZ	(0x0AU)  // 7040Hz输出数据率
#define SH5001_GYRO_ODR_14080HZ	(0x0BU)  // 14080Hz输出数据率

// 陀螺仪量程配置(单位: dps - 度每秒)
#define SH5001_GYRO_RANGE_31	(0x00U)  // ±31 dps量程
#define SH5001_GYRO_RANGE_62	(0x10U)  // ±62 dps量程
#define SH5001_GYRO_RANGE_125	(0x20U)  // ±125 dps量程
#define SH5001_GYRO_RANGE_250	(0x30U)  // ±250 dps量程
#define SH5001_GYRO_RANGE_500	(0x40U)  // ±500 dps量程
#define SH5001_GYRO_RANGE_1000	(0x50U)  // ±1000 dps量程
#define SH5001_GYRO_RANGE_2000	(0x60U)  // ±2000 dps量程
#define SH5001_GYRO_RANGE_4000	(0x70U)  // ±4000 dps量程

#define SH5001_GYRO_ODRX040		(0x00U)
#define SH5001_GYRO_ODRX036		(0x01U)
#define SH5001_GYRO_ODRX032		(0x02U)
#define SH5001_GYRO_ODRX028		(0x03U)
#define SH5001_GYRO_ODRX024		(0x04U)
#define SH5001_GYRO_ODRX020		(0x05U)
#define SH5001_GYRO_ODRX016		(0x06U)
#define SH5001_GYRO_ODRX014		(0x07U)
#define SH5001_GYRO_ODRX012		(0x08U)
#define SH5001_GYRO_ODRX010		(0x09U)
#define SH5001_GYRO_ODRX008		(0x0aU)
#define SH5001_GYRO_ODRX006		(0x0bU)
#define SH5001_GYRO_ODRX004		(0x0cU)
#define SH5001_GYRO_ODRX003		(0x0dU)
#define SH5001_GYRO_ODRX002		(0x0eU)
#define SH5001_GYRO_ODRX001		(0x0fU)

#define SH5001_GYRO_FILTER_EN	(0x01U)
#define SH5001_GYRO_FILTER_DIS	(0x00U)
#define SH5001_GYRO_BYPASS_EN	(0x02U)
#define SH5001_GYRO_BYPASS_DIS	(0x00U)

#define SH5001_GYRO_OFF_INACT   (0x80U)
#define SH5001_GYRO_ON_INACT    (0x00U)

/******************************************************************
*	Time Stamp Config Macro Definitions 
******************************************************************/
#define SH5001_TS_ODR_25KHZ		(0x20U)
#define SH5001_TS_ODR_1KHZ		(0x00U)
#define SH5001_TS_EN			(0x40U)
#define SH5001_TS_DIS			(0x00U)

/******************************************************************
*	温度传感器配置宏定义
******************************************************************/
#define SH5001_TEMP_ODR_500HZ	(0x00U)  // 温度传感器500Hz输出数据率
#define SH5001_TEMP_ODR_250HZ	(0x02U)  // 温度传感器250Hz输出数据率
#define SH5001_TEMP_ODR_125HZ	(0x04U)  // 温度传感器125Hz输出数据率
#define SH5001_TEMP_ODR_63HZ	(0x06U)  // 温度传感器63Hz输出数据率

#define SH5001_TEMP_EN			(0x01U)  // 使能温度传感器
#define SH5001_TEMP_DIS			(0x00U)  // 禁用温度传感器

/******************************************************************
*	中断配置宏定义
******************************************************************/
// 中断类型定义
#define SH5001_INT_LOWG				(0x8000U)  // 低G中断
#define SH5001_INT_HIGHG			(0x4000U)  // 高G中断
#define SH5001_INT_INACT			(0x2000U)  // 非活动状态中断
#define SH5001_INT_ACT				(0x1000U)  // 活动状态中断
#define SH5001_INT_DOUBLE_TAP	  	(0x0800U)  // 双击中断
#define SH5001_INT_SINGLE_TAP	  	(0x0400U)  // 单击中断
#define SH5001_INT_FLAT				(0x0200U)  // 平放检测中断
#define SH5001_INT_ORIENTATION		(0x0100U)  // 方向检测中断
#define SH5001_INT_TAP				(0x0020U)  // 敲击中断(单击或双击)
#define SH5001_INT_SMD				(0x0010U)  // 重要运动检测中断
#define SH5001_INT_FIFO_WATERMARK	(0x0008U)  // FIFO水位中断
#define SH5001_INT_GYRO_READY		(0x0004U)  // 陀螺仪数据就绪中断
#define SH5001_INT_ACC_READY		(0x0002U)  // 加速度计数据就绪中断
#define SH5001_INT_FREE_FALL		(0x0001U)  // 自由落体中断
#define SH5001_INT_UP_DOWN_Z    	(0x0040U)  // Z轴上下方向变化中断

// 中断使能控制
#define SH5001_INT_EN				(0x01U)  // 使能中断
#define SH5001_INT_DIS				(0x00U)  // 禁用中断

#define SH5001_INT_MAP_INT1			(0x01U)
#define SH5001_INT_MAP_INT0			(0x00U)

#define SH5001_INT0_LEVEL_LOW		(0x80U)
#define SH5001_INT0_LEVEL_HIGH		(0x7FU)
#define SH5001_INT_NO_LATCH			(0x40U)
#define SH5001_INT_LATCH			(0xBFU)
#define SH5001_INT_CLEAR_ANY		(0x10U)
#define SH5001_INT_CLEAR_STATUS	    (0xEFU)
#define SH5001_INT1_OD				(0x08U)
#define SH5001_INT1_NORMAL			(0xF7U)
#define SH5001_INT1_OUTPUT			(0x04U)
#define SH5001_INT1_INPUT			(0xFBU)
#define SH5001_INT0_OD				(0x02U)
#define SH5001_INT0_NORMAL			(0xFDU)
#define SH5001_INT0_OUTPUT			(0x01U)
#define SH5001_INT0_INPUT			(0xFEU)


/******************************************************************
*	Orientation Blocking Config Macro Definitions 
******************************************************************/
#define SH5001_ORIENT_BLOCK_MODE0	(0x00U)
#define SH5001_ORIENT_BLOCK_MODE1	(0x04U)
#define SH5001_ORIENT_BLOCK_MODE2	(0x08U)
#define SH5001_ORIENT_BLOCK_MODE3	(0x0CU)

#define SH5001_ORIENT_SYMM			(0x00U)
#define SH5001_ORIENT_HIGH_ASYMM	(0x01U)
#define SH5001_ORIENT_LOW_ASYMM		(0x02U)


/******************************************************************
*	Flat Time Config Macro Definitions 
******************************************************************/
#define SH5001_FLAT_TIME_200MS		(0x00U)
#define SH5001_FLAT_TIME_400MS		(0x40U)
#define SH5001_FLAT_TIME_800MS		(0x80U)
#define SH5001_FLAT_TIME_000MS		(0xC0U)

/******************************************************************
*	ACT and INACT Int Config Macro Definitions 
******************************************************************/
#define SH5001_ACT_X_INT_EN		(0x40U)
#define SH5001_ACT_X_INT_DIS	(0x00U)
#define SH5001_ACT_Y_INT_EN		(0x20U)
#define SH5001_ACT_Y_INT_DIS	(0x00U)
#define SH5001_ACT_Z_INT_EN		(0x10U)
#define SH5001_ACT_Z_INT_DIS	(0x00U)

#define SH5001_INACT_X_INT_EN	(0x04U)
#define SH5001_INACT_X_INT_DIS	(0x00U)
#define SH5001_INACT_Y_INT_EN	(0x02U)
#define SH5001_INACT_Y_INT_DIS	(0x00U)
#define SH5001_INACT_Z_INT_EN	(0x01U)
#define SH5001_INACT_Z_INT_DIS	(0x00U)


#define SH5001_LINK_PRE_STA		(0x01U)
#define SH5001_LINK_PRE_STA_NO	(0x00U)

#define SH5001_ACT_INACT_CLR_STATUS1	(0x00U)
#define SH5001_ACT_INACT_CLR_STATUS3	(0x80U)

/******************************************************************
*	TAP Int Config Macro Definitions 
******************************************************************/
#define SH5001_TAP_X_INT_EN		(0x08U)
#define SH5001_TAP_X_INT_DIS	(0x00U)
#define SH5001_TAP_Y_INT_EN		(0x04U)
#define SH5001_TAP_Y_INT_DIS	(0x00U)
#define SH5001_TAP_Z_INT_EN		(0x02U)
#define SH5001_TAP_Z_INT_DIS	(0x00U)


/******************************************************************
*	HIGHG Int Config Macro Definitions 
******************************************************************/
#define SH5001_HIGHG_ALL_INT_EN		(0x80U)
#define SH5001_HIGHG_ALL_INT_DIS	(0x00U)
#define SH5001_HIGHG_X_INT_EN		(0x40U)
#define SH5001_HIGHG_X_INT_DIS		(0x00U)
#define SH5001_HIGHG_Y_INT_EN		(0x20U)
#define SH5001_HIGHG_Y_INT_DIS		(0x00U)
#define SH5001_HIGHG_Z_INT_EN		(0x10U)
#define SH5001_HIGHG_Z_INT_DIS		(0x00U)


/******************************************************************
*	LOWG Int Config Macro Definitions 
******************************************************************/
#define SH5001_LOWG_ALL_INT_EN		(0x01U)
#define SH5001_LOWG_ALL_INT_DIS		(0x00U)


/******************************************************************
*	SPI Interface Config Macro Definitions 
******************************************************************/
#define SH5001_SPI_3_WIRE      (0x20U)
#define SH5001_SPI_4_WIRE      (0x00U)


/******************************************************************
*	I2C Interface Config Macro Definitions 
******************************************************************/
#define SH5001_I2C_DIS				(0x80U)
#define SH5001_I2C_EN				(0x00U)

//Jin
#define SH5001_I2C_LP_06BYTE		(0x04U)
#define SH5001_I2C_LP_12BYTE		(0x02U)
#define SH5001_I2C_LP_14BYTE		(0x01U)
#define SH5001_I2C_LP_DISABLE		(0x00U)

/******************************************************************
*	OIS SPI Interface Config Macro Definitions 
******************************************************************/
//#define SH5001_OIS_SPI_3WIRE		(0x00U)
//#define SH5001_OIS_SPI_4WIRE		(0x40U)
#define SH5001_OIS_SPI_EN			(0x10U)
#define SH5001_OIS_SPI_DIS			(0x00U)

#define SH5001_ACC_OIS_ENABLE		(0x80U)
#define SH5001_ACC_OIS_DISABLE	(0x00U)

#define SH5001_ACC_OIS_RANGE_4G		(0x10U)
#define SH5001_ACC_OIS_RANGE_8G		(0x20U)
#define SH5001_ACC_OIS_RANGE_16G	(0x30U)

#define SH5001_ACC_OIS_ODRX008		(0x0CU)
#define SH5001_ACC_OIS_ODRX004		(0x08U)
#define SH5001_ACC_OIS_ODRX002		(0x04U)
#define SH5001_ACC_OIS_ODRX000		(0x00U)  //bypass


#define SH5001_ACC_OIS_ODR_1000HZ	(0x00U)  //TODO need to set ref datasheet
#define SH5001_ACC_OIS_ODR_2000HZ	(0x01U)
#define SH5001_ACC_OIS_ODR_4000HZ	(0x02U)
#define SH5001_ACC_OIS_ODR_8000HZ	(0x03U)




#define SH5001_GYRO_OIS_ENABLE		(0x80U)
#define SH5001_GYRO_OIS_DISABLE		(0x00U)

#define SH5001_GYRO_OIS_RANGE_31	(0x00U)
#define SH5001_GYRO_OIS_RANGE_62	(0x10U)
#define SH5001_GYRO_OIS_RANGE_125	(0x20U)
#define SH5001_GYRO_OIS_RANGE_250	(0x30U)
#define SH5001_GYRO_OIS_RANGE_500	(0x40U)
#define SH5001_GYRO_OIS_RANGE_1000	(0x50U)
#define SH5001_GYRO_OIS_RANGE_2000	(0x60U)
#define SH5001_GYRO_OIS_RANGE_4000	(0x70U)

#define SH5001_GYRO_OIS_ODRX008		(0x0CU)
#define SH5001_GYRO_OIS_ODRX004		(0x08U)
#define SH5001_GYRO_OIS_ODRX002		(0x04U)
#define SH5001_GYRO_OIS_ODRX000		(0x00U)  //bypass

#define SH5001_GYRO_OIS_ODR_1000HZ	(0x00U)
#define SH5001_GYRO_OIS_ODR_2000HZ	(0x01U)
#define SH5001_GYRO_OIS_ODR_4000HZ	(0x02U)
#define SH5001_GYRO_OIS_ODR_8000HZ	(0x03U)




/******************************************************************
*	FIFO Config Macro Definitions 
******************************************************************/
#define SH5001_FIFO_MODE_DIS		(0x00U)
#define SH5001_FIFO_MODE_FIFO		(0x01U)
#define SH5001_FIFO_MODE_STREAM		(0x02U)
#define SH5001_FIFO_MODE_TRIGGER	(0x03U)

#define SH5001_FIFO_ACC_DOWNS_EN	(0x80U)
#define SH5001_FIFO_ACC_DOWNS_DIS	(0x00U)
#define SH5001_FIFO_GYRO_DOWNS_EN	(0x08U)
#define SH5001_FIFO_GYRO_DOWNS_DIS	(0x00U)

#define SH5001_FIFO_FREQ_X1_2		(0x00U)
#define SH5001_FIFO_FREQ_X1_4		(0x01U)
#define SH5001_FIFO_FREQ_X1_8		(0x02U)
#define SH5001_FIFO_FREQ_X1_16		(0x03U)
#define SH5001_FIFO_FREQ_X1_32		(0x04U)
#define SH5001_FIFO_FREQ_X1_64		(0x05U)
#define SH5001_FIFO_FREQ_X1_128		(0x06U)
#define SH5001_FIFO_FREQ_X1_256		(0x07U)

#define SH5001_FIFO_TIMESTAMP_EN	(0x4000U)
#define SH5001_FIFO_EXT_Z_EN		(0x2000U)
#define SH5001_FIFO_EXT_Y_EN		(0x1000U)
#define SH5001_FIFO_EXT_X_EN		(0x0080U)
#define SH5001_FIFO_TEMPERATURE_EN	(0x0040U)
#define SH5001_FIFO_GYRO_Z_EN		(0x0020U)
#define SH5001_FIFO_GYRO_Y_EN		(0x0010U)
#define SH5001_FIFO_GYRO_X_EN		(0x0008U)
#define SH5001_FIFO_ACC_Z_EN		(0x0004U)
#define SH5001_FIFO_ACC_Y_EN		(0x0002U)
#define SH5001_FIFO_ACC_X_EN		(0x0001U)
#define SH5001_FIFO_ALL_DIS			(0x0000U)

/******************************************************************
*	AUX I2C Config Macro Definitions 
******************************************************************/
#define SH5001_MI2C_NORMAL_MODE			(0x00U)
#define SH5001_MI2C_BYPASS_MODE			(0x01U)

#define SH5001_MI2C_READ_ODR_200HZ	    (0x00U)
#define SH5001_MI2C_READ_ODR_100HZ	    (0x10U)
#define SH5001_MI2C_READ_ODR_50HZ		(0x20U)
#define SH5001_MI2C_READ_ODR_25HZ		(0x30U)

#define SH5001_MI2C_FAIL				(0x20U)
#define SH5001_MI2C_SUCCESS				(0x10U)

#define SH5001_MI2C_READ_MODE_AUTO		(0x40U)
#define SH5001_MI2C_READ_MODE_MANUAL	(0x00U)


/******************************************************************
*	Fast offset compensation Macro Definitions 
******************************************************************/
#define SH5001_X_UP     (0x00U)
#define SH5001_X_DOWN   (0x01U)
#define SH5001_Y_UP     (0x02U)
#define SH5001_Y_DOWN   (0x03U)
#define SH5001_Z_UP     (0x04U)
#define SH5001_Z_DOWN   (0x05U)


/******************************************************************
*	其他宏定义
******************************************************************/
#define SH5001_TRUE		(0U)  // 成功/真值
#define SH5001_FALSE	(1U)  // 失败/假值

// 寄存器数量定义
#define SH5001_CALI_REG_NUM 8           // 校准寄存器数量
#define SH5001_POWMODE_REG_NUM_O1 11    // 电源模式寄存器数量选项1
#define SH5001_POWMODE_REG_NUM_O2 14    // 电源模式寄存器数量选项2

// 电源模式定义
#define SH5001_NORMAL_MODE		(0x00)  // 正常模式
#define SH5001_ACC_HP_MODE		(0x01)  // 加速度计高性能模式
#define SH5001_ACC_LP_MODE		(0x02)  // 加速度计低功耗模式
#define SH5001_POWERDOWN_MODE	(0x03)  // 掉电模式

// FIFO相关定义
// FIFO模式分频器 = FIFO通道数 * 2;
#define SH5001_WATERMARK_DIV    (12U)   // FIFO水位分频器
//#define SH5001_FIFO_BUFFER      (1024U)  // FIFO缓冲区大小(最大)
#define SH5001_FIFO_BUFFER      (500U)   // FIFO缓冲区大小(当前设置)


/***************************************************************************
*       类型定义
****************************************************************************/
// IMU读取函数指针类型定义
typedef unsigned char (*IMU_read)(	unsigned char devAddr,  // 设备地址
                                    unsigned char regAddr,  // 寄存器地址
                                    unsigned short readLen, // 读取长度
                                    unsigned char *readBuf);// 读取缓冲区

// IMU写入函数指针类型定义
typedef unsigned char (*IMU_write)(	unsigned char devAddr,  // 设备地址
                                    unsigned char regAddr,  // 寄存器地址
                                    unsigned short writeLen,// 写入长度
                                    unsigned char *writeBuf);// 写入缓冲区




/***************************************************************************
        导出函数声明
****************************************************************************/

// 基本功能函数
extern unsigned char SH5001_init(void);                                    // SH5001初始化函数
extern void SH5001_GetImuData( short accData[3], short gyroData[3] );     // 获取IMU数据(加速度计和陀螺仪)
extern float SH5001_GetTempData(void);                                     // 获取温度数据
extern unsigned char SH5001_SwitchPowerMode(unsigned char powerMode);     // 切换电源模式
																																																																																						
																		

// 中断相关函数
extern void SH5001_INT_Enable(	unsigned short intType,     // 中断类型
								unsigned char intEnable,    // 中断使能/禁用
								unsigned char intPinSel);   // 中断引脚选择

extern void SH5001_INT_Config(	unsigned char intLevel,     // 中断电平
                        unsigned char intLatch,         // 中断锁存模式
                        unsigned char intClear,         // 中断清除模式
                        unsigned char intTime,          // 中断时间
                        unsigned char int1Mode,         // INT1引脚模式
                        unsigned char int0Mode,         // INT0引脚模式
                        unsigned char int1OE,           // INT1输出使能
                        unsigned char int0OE);          // INT0输出使能

extern void SH5001_INT_Output_Config(	unsigned char int1Mode,  // INT1引脚模式
										unsigned char int0Mode,  // INT0引脚模式
										unsigned char int1OE,    // INT1输出使能
										unsigned char int0OE);   // INT0输出使能

extern void SH5001_INT_Orient_Config(	unsigned char 	orientBlockMode,  // 方向阻塞模式
										unsigned char 	orientMode,       // 方向模式
										unsigned char	orientTheta,      // 方向角度阈值
										unsigned short	orientG1point5,   // 1.5G阈值
										unsigned short 	orientSlope,      // 斜率阈值
										unsigned short 	orientHyst);      // 滞后阈值

extern void SH5001_INT_Flat_Config(	unsigned char flatTimeTH,    // 平放时间阈值
									unsigned char flatTanHeta2); // 平放角度阈值
								
extern void SH5001_INT_Act_Config(	unsigned char actEnDisIntX,
                            		unsigned char actEnDisIntY,
                            		unsigned char actEnDisIntZ,
                            		unsigned char actTimeNum,
                            		unsigned short actIntThres,
                            		unsigned char actLinkStatus,
                            		unsigned char actIntClear);								

extern void SH5001_INT_Inact_Config(	unsigned char inactEnDisIntX,
                                		unsigned char inactEnDisIntY,
                                		unsigned char inactEnDisIntZ,
                                		unsigned char inactLinkStatus,
                                		unsigned char inactTimeMs,
                                		unsigned short inactIntThres,
                                		unsigned short inactG1,
                                		unsigned char inactIntClear);

extern void SH5001_INT_Tap_Config(	unsigned char tapEnDisIntX,
									unsigned char tapEnDisIntY,
									unsigned char tapEnDisIntZ,
									unsigned char tapIntThres,
									unsigned char tapTimeMs,
									unsigned char tapWaitTimeMs,
									unsigned char tapWaitTimeWindowMs);

extern void SH5001_INT_SMD_Config(	unsigned short smdThres,
                            		unsigned char smdBlockTimeS,
									unsigned char smdProofTimeS);									

extern void SH5001_INT_HighG_Config(	unsigned char highGEnDisIntX,
										unsigned char highGEnDisIntY,
										unsigned char highGEnDisIntZ,
										unsigned char highGEnDisIntAll,
										unsigned short highGThres,
										unsigned char highGTimeMs);

extern void SH5001_INT_LowG_Config(	unsigned char lowGEnDisIntAll,
									unsigned short lowGThres,
									unsigned char lowGTimeMs);

extern void SH5001_INT_FreeFall_Config(	unsigned char freeFallThres,
										unsigned char freeFallTimeMs);

extern unsigned short SH5001_INT_Read_Status01(void);

extern unsigned char SH5001_INT_Read_Status2(void);

extern unsigned char SH5001_INT_Read_Status3(void);

extern unsigned char SH5001_INT_Read_Status4(void);
								
			
// FIFO相关函数
extern void SH5001_FIFO_Reset(void);  // FIFO复位

extern void SH5001_FIFO_Freq_Config(	unsigned char fifoAccDownSampleEnDis,  // 加速度计降采样使能/禁用
										unsigned char fifoAccFreq,             // 加速度计FIFO频率
										unsigned char fifoGyroDownSampleEnDis, // 陀螺仪降采样使能/禁用
										unsigned char fifoGyroFreq);           // 陀螺仪FIFO频率

extern void SH5001_FIFO_Data_Config(	unsigned short fifoMode,          // FIFO模式配置
										unsigned short fifoWaterMarkLevel); // FIFO水位阈值

extern unsigned char SH5001_FIFO_Read_Status(unsigned short int *fifoEntriesCount);  // 读取FIFO状态和数据计数

extern void SH5001_FIFO_Read_Data(	unsigned char *fifoReadData,      // FIFO读取数据缓冲区
									unsigned short int fifoDataLength); // FIFO数据长度
			

// 主I2C接口相关函数
extern void SH5001_MI2C_Reset(void);  // 主I2C复位

extern void SH5001_MI2C_Bus_Config(	unsigned char mi2cReadMode,  // 主I2C读取模式
									unsigned char mi2cODR,       // 主I2C输出数据率
									unsigned char mi2cFreq);     // 主I2C频率

extern void SH5001_MI2C_Cmd_Config(	unsigned char mi2cSlaveAddr,  // 主I2C从设备地址
									unsigned char mi2cSlaveCmd,   // 主I2C从设备命令
									unsigned char mi2cReadMode);  // 主I2C读取模式

extern unsigned char SH5001_MI2C_Write(unsigned char mi2cWriteData);  // 主I2C写数据

extern unsigned char SH5001_MI2C_Read(unsigned char *mi2cReadData);   // 主I2C读数据


// SPI接口相关函数
extern void SH5001_SPI_Config(unsigned char spiInterfaceMode);  // SPI接口配置


// I2C接口相关函数
extern void SH5001_I2C_Config(	unsigned char i2cInterfaceEnDis);  // I2C接口使能/禁用配置

extern void SH5001_I2C_LP_Config(unsigned char i2cReadLoopMode);  // I2C低功耗循环模式配置


// OIS SPI接口相关函数
extern void SH5001_OIS_SPI_Config(unsigned char oisSpiInterfaceEnDis);  // OIS SPI接口使能/禁用配置


// 快速偏移补偿函数
extern void SH5001_FastOffsetComp(unsigned char SH5001Positon);  // 快速偏移补偿(根据设备位置)


// FIFO数据读取函数
extern void SH5001_initFIFO(void);  // 初始化FIFO

extern void SH5001_ReadFIFO(void);  // 读取FIFO数据
                                    
#endif

