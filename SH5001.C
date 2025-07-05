/******************************************************************
* Senodia 6D IMU(Gyroscope+Accelerometer) SH5001 driver, By Senodia AE@20220505;
*
*
* 1. SH5001_init() : SH5001 initialize function
* 2. SH5001_GetImuData( ... ): SH5001 read gyro and accel data function
* 3. SH5001_GetTempData(): SH5001 read termperature function
* 4. delay_ms(x): delay x millisecond.
* 5. SH5001_initFIFO(): initialize FIFO function
* 6. SH5001_ReadFIFO(): read FIFO data
*	
******************************************************************/

#include "SH5001.h"

// 32-bit version number represented as major[31:16].minor[15:0]
#define SH5001_MAJOR        1
#define SH5001_MINOR       01
#define SH5001_REV         38
#define MCU_VERSION_SH5001  ((SH5001_MAJOR<<16) | (SH5001_MINOR<<8) | SH5001_REV)


/******************************************************************
* Description:	I2C or SPI bus interface functions	and delay time function
*
* Parameters: 
*   devAddr: I2C device address, modify the macro(SH5001_ADDRESS @SH5001.h) based on your I2C function.						  
*            If SPI interface, please ingnore the parameter.
*   regAddr: register address
*   readLen: data length to read
*   *readBuf: data buffer to read; If using FIFO function, the max buffer size is 1024; 
*   writeLen: data length to write
*   *writeBuf: data buffer to write
*	
* Note: I2C interface, define USER_INTERFACE_I2C (1)
*       SPI interface, define USER_INTERFACE_I2C (0)
*		SPI 3 Wire interface, define SH5001_SPI_3WIRE_MODE (1)
*		SPI 4 Wire interface, define SH5001_SPI_3WIRE_MODE (0)
*	
******************************************************************/
#define USER_INTERFACE_I2C		(0)

#if USER_INTERFACE_I2C

unsigned char I2C_ReadNBytes(	unsigned char devAddr, 
                                unsigned char regAddr, 
                                unsigned short readLen, 
                                unsigned char *readBuf)
{
	//your I2C interface code:
	//......
	return (SH5001_TRUE);
}

unsigned char I2C_WriteNBytes(	unsigned char devAddr, 
                                unsigned char regAddr, 
                                unsigned short writeLen, 
                                unsigned char *writeBuf)
{
	//your I2C interface code:
	//......
	return (SH5001_TRUE);
}																

IMU_read    SH5001_read     = I2C_ReadNBytes;
IMU_write   SH5001_write    = I2C_WriteNBytes;

#else

#define SH5001_SPI_3WIRE_MODE	(0)

unsigned char SPI_readNBytes (	unsigned char devAddr, 
                                unsigned char regAddr, 
                                unsigned short readLen, 
                                unsigned char *readBuf)
{
	//your SPI interface code:

	//For example:	
	//unsigned char u8Data;		
	//devAddr = devAddr;	
	//u8Data = (regAddr > 0x7F) ? 0x01 : 0x00;
	//SPIWrite(SH5001_SPI_REG_ACCESS, &u8Data, 1);
	
	//SPIRead((regAddr | 0x80), readBuf, readLen);	
	return (SH5001_TRUE);
}

unsigned char SPI_writeNBytes(	unsigned char devAddr, 
                                unsigned char regAddr, 
                                unsigned short writeLen, 
                                unsigned char *writeBuf)
{
	//your SPI interface code:
	
	//For example: 
	//unsigned char u8Data;	
	//devAddr = devAddr;	
	//u8Data = (regAddr > 0x7F) ? 0x01 : 0x00;
	//SPIWrite(SH5001_SPI_REG_ACCESS, &u8Data, 1);
	
	//SPIWrite((regAddr & 0x7F), writeBuf, writeLen);
	return (SH5001_TRUE);
}																

IMU_read 	SH5001_read		= SPI_readNBytes;
IMU_write	SH5001_write	= SPI_writeNBytes;

#endif


static void delay_ms(unsigned short mSecond)
{  
	//your delay code(mSecond: millisecond):
	//......
	HAL_Delay(mSecond);
} 


#if SH5001_OIS_ENABLE
unsigned char SH5001_OIS_read(	unsigned char devAddr,
                                unsigned char regAddr,
                                unsigned short readLen,
                                unsigned char *readBuf)
{
	//your SPI interface code:


	//SPIRead((regAddr | 0x80), readBuf, readLen);
	return (SH5001_TRUE);
}

#endif


/******************************************************************
* Description:	Local Variable		
******************************************************************/


/******************************************************************
* Description:	Local Function Prototypes		
******************************************************************/
static void SH5001_ModuleReset(void);
static void SH5001_Acc_Config(unsigned char accODR, 
                              unsigned char accRange, 
                              unsigned char accCutOffFreq,
                              unsigned char accFilter,
                              unsigned char accByPass);

static void SH5001_Gyro_Config(	unsigned char gyroODR, 
                                unsigned char gyroRange, 
                                unsigned char gyroCutOffFreq,
                                unsigned char gyroFilter,
                                unsigned char gyroByPass);

static void SH5001_TimeStamp_Config(unsigned char timeStampODR, 
                                	unsigned char timeStampEnable);

static void SH5001_Temp_Config(	unsigned char tempODR, 
                                unsigned char tempEnable);																									

	

// 软件复位函数
static void SH5001_SoftReset(void)
{
	unsigned char regData = 0;

	regData = 0x01;  // 设置复位位
	SH5001_write(SH5001_ADDRESS, 0x2B, 1, &regData);
	regData = 0x73;  // 写入复位命令
	SH5001_write(SH5001_ADDRESS, 0x00, 1, &regData);

	delay_ms(50);  // 等待复位完成
}

// 驱动启动函数
static void SH5001_DriveStart(void)
{
	unsigned char regData = 0;

	// 启动驱动
	regData = 0x01;  // 启动驱动
	SH5001_write(SH5001_ADDRESS, 0x2B, 1, &regData);

	delay_ms(2);  // 等待启动

	regData = 0x00;  // 清除启动位
	SH5001_write(SH5001_ADDRESS, 0x2B, 1, &regData);

	delay_ms(1);  // 等待稳定

}

// ADC复位函数
static void SH5001_ADCReset(void)
{
    unsigned char regData = 0;

	regData = 0x08;  // 设置ADC复位模式
	SH5001_write(SH5001_ADDRESS, 0x30, 1, &regData);

	regData = 0x00;  // 配置ADC寄存器
	SH5001_write(SH5001_ADDRESS, 0xD2, 1, &regData);

	regData = 0x6B;  // 配置ADC控制寄存器
	SH5001_write(SH5001_ADDRESS, 0xD1, 1, &regData);

	regData = 0x02;  // 启动ADC复位
	SH5001_write(SH5001_ADDRESS, 0xD5, 1, &regData);
	delay_ms(5);  // 等待复位

	regData = 0x68;  // 配置ADC参数
	SH5001_write(SH5001_ADDRESS, 0xD1, 1, &regData);
	delay_ms(2);  // 等待配置

	regData = 0x00;  // 清除复位位
	SH5001_write(SH5001_ADDRESS, 0xD5, 1, &regData);

	regData = 0x00;  // 恢复正常模式
	SH5001_write(SH5001_ADDRESS, 0x30, 1, &regData);

	delay_ms(50);  // 等待ADC稳定
}

static void SH5001_CVAReset(void)
{
	unsigned char regData = 0;
	unsigned char regDEData = 0;

	SH5001_read(SH5001_ADDRESS, 0xDE, 1, &regDEData);

	regData = regDEData & 0xC7;
	SH5001_write(SH5001_ADDRESS, 0xDE, 1, &regData);
	delay_ms(5);

	regData = regDEData | 0x38;
	SH5001_write(SH5001_ADDRESS, 0xDE, 1, &regData);
	delay_ms(5);

	SH5001_write(SH5001_ADDRESS, 0xDE, 1, &regDEData);
	delay_ms(5);

	regData = 0x12;
	SH5001_write(SH5001_ADDRESS, 0xCD, 1, &regData);
	regData = 0x12;
	SH5001_write(SH5001_ADDRESS, 0xCE, 1, &regData);
	regData = 0x12;
	SH5001_write(SH5001_ADDRESS, 0xCF, 1, &regData);

	delay_ms(1);

	regData = 0x2;
	SH5001_write(SH5001_ADDRESS, 0xCD, 1, &regData);
	regData = 0x2;
	SH5001_write(SH5001_ADDRESS, 0xCE, 1, &regData);
	regData = 0x2;
	SH5001_write(SH5001_ADDRESS, 0xCF, 1, &regData);

}
#if SENODIA_VDD_3V3
void SH5001_AccReset(void)
{
    unsigned char regData = 0;

	regData = 0x08;
	SH5001_write(SH5001_ADDRESS, 0x30, 1, &regData);

	regData = 0xE0;
	SH5001_write(SH5001_ADDRESS, 0xD8, 1, &regData);

	delay_ms(5);

	regData = 0x00;
	SH5001_write(SH5001_ADDRESS, 0xD8, 1, &regData);

	regData = 0x00;
	SH5001_write(SH5001_ADDRESS, 0x30, 1, &regData);
}
#endif
/******************************************************************
* Description: reset some internal modules;
*
* Parameters: void	
*  						
* return:	void
*																
******************************************************************/
static void SH5001_ModuleReset(void)
{
	unsigned char regData = 0;	
	
	// soft reset
	SH5001_SoftReset();

#if SH5001_SPI_3WIRE_MODE
	SH5001_SPI_Config(SH5001_SPI_3_WIRE);
#endif	

	//Driver start
	SH5001_DriveStart();

	// ADC reset
	SH5001_ADCReset();

	//CVA Reset
	SH5001_CVAReset();

	delay_ms(200);
#if SENODIA_VDD_3V3
	SH5001_AccReset();
#endif
}	

//如switch power中也有用到0x20寄存器，对应bit需注意同步修改。不能修改其他位置设置
void SH5001_DeadZone_Dither(void)
{
	unsigned char regData = 0;

	SH5001_read(SH5001_ADDRESS, SH5001_ACC_CONF0, 1, &regData);
	regData = regData | 0x40;
	SH5001_write(SH5001_ADDRESS, SH5001_ACC_CONF0, 1, &regData);

	SH5001_read(SH5001_ADDRESS, 0xBC, 1, &regData);
	regData = regData | 0x1;
	SH5001_write(SH5001_ADDRESS, 0xBC, 1, &regData);
}

/******************************************************************
* 描述:	1.设置加速度计参数;
*       2.截止频率计算公式: accCutOffFreq = accODR * 0.40 或 accODR * 0.25 或 accODR * 0.11 或 accODR * 0.04 或 accODR * 0.02;
*
* 参数: 	accODR              	accRange                accCutOffFreq       	accFilter				accByPass
*       SH5001_ACC_ODR_1000HZ	SH5001_ACC_RANGE_16G	SH5001_ACC_ODRX040		SH5001_ACC_FILTER_EN	SH5001_ACC_BYPASS_EN
*       SH5001_ACC_ODR_500HZ	SH5001_ACC_RANGE_8G		SH5001_ACC_ODRX025		SH5001_ACC_FILTER_DIS	SH5001_ACC_BYPASS_DIS
*       SH5001_ACC_ODR_250HZ	SH5001_ACC_RANGE_4G		SH5001_ACC_ODRX011
*       SH5001_ACC_ODR_125HZ	SH5001_ACC_RANGE_2G		SH5001_ACC_ODRX004
*       SH5001_ACC_ODR_63HZ								SH5001_ACC_ODRX002
*       SH5001_ACC_ODR_31HZ
*       SH5001_ACC_ODR_16HZ
*       SH5001_ACC_ODR_2000HZ
*       SH5001_ACC_ODR_4000HZ
*       SH5001_ACC_ODR_8000HZ
*
* 返回值:	无
*
******************************************************************/
unsigned char storeAccODR;  // 存储加速度计ODR配置
static void SH5001_Acc_Config(unsigned char accODR,        // 加速度计输出数据率
                              unsigned char accRange,      // 加速度计量程
                              unsigned char accCutOffFreq, // 加速度计截止频率
                              unsigned char accFilter,     // 加速度计数字滤波器使能
                              unsigned char accByPass)     // 加速度计旁路模式使能
{
    unsigned char regData = 0;

	// 配置加速度计数字滤波器和旁路模式
	SH5001_read(SH5001_ADDRESS, SH5001_ACC_CONF0, 1, &regData);
	regData = (regData & 0xFCU) | accFilter | accByPass;  // 设置滤波器和旁路位
	SH5001_write(SH5001_ADDRESS, SH5001_ACC_CONF0, 1, &regData);

	// 设置加速度计ODR和量程
	SH5001_read(SH5001_ADDRESS, SH5001_ACC_CONF1, 1, &regData);
	regData = (regData & 0x80U) | accODR | accRange;  // 保留最高位，设置ODR和量程
	storeAccODR = regData;  // 保存配置用于后续使用
	//regData = (regData & 0x80U)| accRange | accODR;
	//regData = (regData & 0x80U)| 0x38;
	SH5001_write(SH5001_ADDRESS, SH5001_ACC_CONF1, 1, &regData);

	// 设置加速度计低通滤波器截止频率
	SH5001_read(SH5001_ADDRESS, SH5001_ACC_CONF2, 1, &regData);
	regData = (regData & 0xF0U) | accCutOffFreq;  // 保留高4位，设置截止频率
	SH5001_write(SH5001_ADDRESS, SH5001_ACC_CONF2, 1, &regData);

}




/******************************************************************
* Description:	1.set accelerometer parameters;
*               2.accCutOffRreq = accODR * 0.40 or accODR * 0.25 or accODR * 0.11 or accODR * 0.04 or accODR * 0.02;
*
* Parameters: 	accODR              	accRange                accCutOffFreq       	accOisEnable

* return:	void
*
******************************************************************/
static void SH5001_Acc_OIS_Config(unsigned char accODR,
                              unsigned char accRange,
                              unsigned char accCutOffFreq,
                              unsigned char accOisEnable)
{
    unsigned char regData = 0;

	regData = accODR | accRange | accCutOffFreq | accOisEnable;
	SH5001_write(SH5001_ADDRESS, SH5001_OIS_ACC_CONF, 1, &regData);

}

/******************************************************************
* Description:	1.set gyroscope parameters;
*               2.gyroCutOffRreq is at Page 33 of SH5001 datasheet.
*
* Parameters:   gyroODR             	gyroRangeX,Y,Z          gyroCutOffFreq      gyroFilter				gyroByPass
*               SH5001_GYRO_ODR_800HZ	SH5001_GYRO_RANGE_125	SH5001_GYRO_ODRX00	SH5001_GYRO_FILTER_EN	SH5001_GYRO_BYPASS_EN
*               SH5001_GYRO_ODR_400HZ	SH5001_GYRO_RANGE_250	SH5001_GYRO_ODRX01	SH5001_GYRO_FILTER_DIS	SH5001_GYRO_BYPASS_DIS
*               SH5001_GYRO_ODR_200HZ	SH5001_GYRO_RANGE_500	SH5001_GYRO_ODRX02
*               SH5001_GYRO_ODR_100HZ	SH5001_GYRO_RANGE_1000  SH5001_GYRO_ODRX03
*               SH5001_GYRO_ODR_50HZ	SH5001_GYRO_RANGE_2000
*               SH5001_GYRO_ODR_25HZ
*               SH5001_GYRO_ODR_1600HZ
*               SH5001_GYRO_ODR_3200HZ
*               SH5001_GYRO_ODR_6400HZ
*               SH5001_GYRO_ODR_12800HZ
*               SH5001_GYRO_ODR_25600HZ
*  	 
*               gyroOnOff
*               SH5001_GYRO_OFF_INACT
*               SH5001_GYRO_ON_INACT
* return:	void
*
******************************************************************/
/*
static void SH5001_Gyro_Config(	unsigned char gyroODR, 
                                unsigned char gyroRange, 
                                unsigned char gyroCutOffFreq,
                                unsigned char gyroFilter,
                                unsigned char gyroByPass,
                                unsigned char gyroOnOff)
{
    unsigned char regData = 0;	
	
	// enable gyro digital filter
	SH5001_read(SH5001_ADDRESS, SH5001_GYRO_CONF0, 1, &regData);
	regData = (regData & 0x7CU) | gyroFilter | gyroByPass | gyroOnOff;
	SH5001_write(SH5001_ADDRESS, SH5001_GYRO_CONF0, 1, &regData);
	
	// set gyro ODR
	SH5001_read(SH5001_ADDRESS, SH5001_GYRO_CONF1, 1, &regData);
	regData = (regData & 0xF0U) | gyroODR;
	SH5001_write(SH5001_ADDRESS, SH5001_GYRO_CONF1, 1, &regData);
	
	// set gyro low pass filter cut-off frequency
	SH5001_read(SH5001_ADDRESS, SH5001_GYRO_CONF2, 1, &regData);
	regData = (regData & 0xFCU) | gyroCutOffFreq;
	SH5001_write(SH5001_ADDRESS, SH5001_GYRO_CONF2, 1, &regData);
		

	// set gyro range
	SH5001_read(SH5001_ADDRESS, SH5001_GYRO_CONF3, 1, &regData);
	regData = (regData & 0xF8U) | gyroRange;
	SH5001_write(SH5001_ADDRESS, SH5001_GYRO_CONF3, 1, &regData);		 

}

*/
static void SH5001_Gyro_Config(	unsigned char gyroODR,
                                unsigned char gyroRange,
                                unsigned char gyroCutOffFreq,
                                unsigned char gyroFilter,
                                unsigned char gyroByPass)
{
    unsigned char regData = 0;

	// enable gyro digital filter
	SH5001_read(SH5001_ADDRESS, SH5001_GYRO_CONF0, 1, &regData);
	regData = (regData & 0x7CU) | gyroFilter | gyroByPass;
	SH5001_write(SH5001_ADDRESS, SH5001_GYRO_CONF0, 1, &regData);

	// set gyro ODR Range
	SH5001_read(SH5001_ADDRESS, SH5001_GYRO_CONF1, 1, &regData);
	regData = (regData & 0x80U) | gyroODR | gyroRange;
	SH5001_write(SH5001_ADDRESS, SH5001_GYRO_CONF1, 1, &regData);

	// set gyro low pass filter cut-off frequency
	SH5001_read(SH5001_ADDRESS, SH5001_GYRO_CONF2, 1, &regData);
	regData = (regData & 0xF0U) | gyroCutOffFreq;
	SH5001_write(SH5001_ADDRESS, SH5001_GYRO_CONF2, 1, &regData);

}

//Jin: Modify 
/*  */


static void SH5001_Gyro_OIS_Config(	unsigned char gyroODR,
                                unsigned char gyroRange,
                                unsigned char gyroCutOffFreq,
                                unsigned char gyroOisEnable)
{
    unsigned char regData = 0;

	regData = gyroODR | gyroRange | gyroCutOffFreq | gyroOisEnable;
	SH5001_write(SH5001_ADDRESS, SH5001_OIS_GYRO_CONF, 1, &regData);

}


/******************************************************************
* Description:	set time stamp parameters;
*
* Parameters: 	timeStampODR			timeStampEnable
*               SH5001_TS_ODR_25KHZ	  	SH5001_TS_EN
*               SH5001_TS_ODR_1KHZ	  	SH5001_TS_DIS		
*
* return:	void
*  							
******************************************************************/
static void SH5001_TimeStamp_Config(unsigned char timeStampODR, 
                                	unsigned char timeStampEnable)
{
    unsigned char regData = 0;	
	
	// config time stamp
	SH5001_read(SH5001_ADDRESS, SH5001_TIMESTAMP_CONF, 1, &regData);
	regData = (regData & 0x9FU) | timeStampODR | timeStampEnable;
	SH5001_write(SH5001_ADDRESS, SH5001_TIMESTAMP_CONF, 1, &regData);
}


/******************************************************************
* Description:	set temperature parameters;
*
* Parameters: 	tempODR                   tempEnable
*               SH5001_TEMP_ODR_500HZ	  SH5001_TEMP_EN
*               SH5001_TEMP_ODR_250HZ	  SH5001_TEMP_DIS
*               SH5001_TEMP_ODR_125HZ		
*               SH5001_TEMP_ODR_63HZ		
*
* return:	void
*  							
******************************************************************/
static void SH5001_Temp_Config(	unsigned char tempODR, 
                                unsigned char tempEnable)
{
    unsigned char regData = 0;	
	
	// enable temperature, set ODR
	SH5001_read(SH5001_ADDRESS, SH5001_TEMP_CONF0, 1, &regData);
	regData = (regData & 0xF8U) | tempODR | tempEnable;
	SH5001_write(SH5001_ADDRESS, SH5001_TEMP_CONF0, 1, &regData);
}


/******************************************************************
* Description:	read temperature parameters;
*
* Parameters: 	void	
*  		
* return:	temperature data(deg);
* 
******************************************************************/
float SH5001_GetTempData(void)
{
    unsigned char regData[2] = {0};	
    unsigned short int tempref[2] = {0};
	
	// read temperature data, unsigned 12bits;   SH5001_TEMP_CONF2..SH5001_TEMP_CONF1
	SH5001_read(SH5001_ADDRESS, SH5001_TEMP_CONF1, 2, &regData[0]);
	tempref[0] = ((unsigned short int)(regData[1] & 0x0FU) << 8U) | regData[0];
	
	SH5001_read(SH5001_ADDRESS, SH5001_TEMP_ZL, 2, &regData[0]);
	tempref[1] = ((unsigned short int)(regData[1] & 0x0FU) << 8U) | regData[0];	
	
	return ( (((float)(tempref[1] - tempref[0]))/14.0f) + 25.0f );
}



/******************************************************************
* Description:	enable or disable INT, mapping interrupt to INT pin or INT1 pin
*
* Parameters:   intType                 intEnable               intPinSel
*               SH5001_INT_LOWG         SH5001_INT_EN			SH5001_INT_MAP_INT0
*               SH5001_INT_HIGHG        SH5001_INT_DIS      	SH5001_INT_MAP_INT1
*               SH5001_INT_INACT        		  
*               SH5001_INT_ACT
*               SH5001_INT_DOUBLE_TAP
*               SH5001_INT_SINGLE_TAP
*               SH5001_INT_FLAT
*               SH5001_INT_ORIENTATION
*				SH5001_INT_TAP (Only INT/INT1 Map)
*				SH5001_INT_SMD
*               SH5001_INT_FIFO_WATERMARK
*               SH5001_INT_GYRO_READY
*               SH5001_INT_ACC_READY
*               SH5001_INT_FREE_FALL
*               SH5001_INT_UP_DOWN_Z
* return:	void
* 
******************************************************************/
void SH5001_INT_Enable(	unsigned short intType, 
                        unsigned char intEnable, 
                        unsigned char intPinSel)
{
  unsigned char regData[2] = {0};	
  unsigned short int u16IntVal = 0; 
	
	// Z axis change between UP to DOWN
	if((intType & 0x0040U) == SH5001_INT_UP_DOWN_Z)
	{
		SH5001_read(SH5001_ADDRESS, SH5001_ORIEN_INTCONF0, 1, &regData[0]);
		regData[0] = (intEnable == SH5001_INT_EN) \
                     ? (regData[0] & 0xBFU) : (regData[0] | 0x40U);
		SH5001_write(SH5001_ADDRESS, SH5001_ORIEN_INTCONF0, 1, &regData[0]);
	}	
	
	if(intType & 0xFF3FU)
	{	
		// enable or disable INT
		SH5001_read(SH5001_ADDRESS, SH5001_INT_ENABLE0, 2, &regData[0]); 
    
		u16IntVal = ((unsigned short int)regData[0] << 8U) | regData[1]; 
		
		u16IntVal = u16IntVal & 0xFF1FU;
		u16IntVal = (intEnable == SH5001_INT_EN) \
                    ? (u16IntVal | intType) : (u16IntVal & ~intType);		
                
		regData[0] = (unsigned char)(u16IntVal >> 8U);
		regData[1] = (unsigned char)(u16IntVal);		
		//SH5001_write(SH5001_ADDRESS, SH5001_INT_ENABLE0, 2, &regData[0]);
		SH5001_write(SH5001_ADDRESS, SH5001_INT_ENABLE0, 1, &regData[0]);
		SH5001_write(SH5001_ADDRESS, SH5001_INT_ENABLE1, 1, &regData[1]);
				
		// mapping interrupt to INT0 pin or INT1 pin
		SH5001_read(SH5001_ADDRESS, SH5001_INT_PIN_MAP0, 2, &regData[0]);
		u16IntVal = ((unsigned short int)regData[0] << 8U) | regData[1];
    
		u16IntVal = u16IntVal & 0xFF3FU;
        u16IntVal = (intPinSel == SH5001_INT_MAP_INT1) \
                    ? (u16IntVal | intType) : (u16IntVal & ~intType);	
		
		regData[0] = (unsigned char)(u16IntVal >> 8U);
		regData[1] = (unsigned char)(u16IntVal);
		SH5001_write(SH5001_ADDRESS, SH5001_INT_PIN_MAP0, 1, &regData[0]);
		SH5001_write(SH5001_ADDRESS, SH5001_INT_PIN_MAP1, 1, &regData[1]);
	}	
}


/******************************************************************
* Description:	1.config INT function
*               2.intCount is valid only when intLatch is equal to SH5001_INT_NO_LATCH;
*
* Parameters:   intLevel					intLatch					intClear					
*               SH5001_INT_LEVEL_HIGH		SH5001_INT_NO_LATCH			SH5001_INT_CLEAR_ANY
*               SH5001_INT_LEVEL_LOW	  	SH5001_INT_LATCH			SH5001_INT_CLEAR_STATUS
*
*               intTime
*               Unit: 2mS
*
*  				int1Mode					int0Mode
*               SH5001_INT1_NORMAL			SH5001_INT0_NORMAL
*               SH5001_INT1_OD		  		SH5001_INT0_OD
*
*				int1OE						int0OE
*				SH5001_INT1_OUTPUT			SH5001_INT0_OUTPUT
*				SH5001_INT1_INPUT			SH5001_INT1_INPUT
*
* return:	void
* 
******************************************************************/


void SH5001_INT_Config(	unsigned char intLevel,
                        unsigned char intLatch,
                        unsigned char intClear,
                        unsigned char intTime,
                        unsigned char int1Mode,
                        unsigned char int0Mode,
                        unsigned char int1OE,
                        unsigned char int0OE)
{
	unsigned char regData = 0;

	//SH5001_read(SH5001_ADDRESS, SH5001_INT_CONF, 1, &regData);

    regData = (intLevel == SH5001_INT0_LEVEL_LOW) \
              ? (regData | SH5001_INT0_LEVEL_LOW) : (regData & SH5001_INT0_LEVEL_HIGH);

    regData = (intLatch == SH5001_INT_NO_LATCH) \
              ? (regData | SH5001_INT_NO_LATCH) : (regData & SH5001_INT_LATCH);

    regData = (intClear == SH5001_INT_CLEAR_ANY) \
              ? (regData | SH5001_INT_CLEAR_ANY) : (regData & SH5001_INT_CLEAR_STATUS);

    regData = (int1Mode == SH5001_INT1_OD) \
              ? (regData | SH5001_INT1_OD) : (regData & SH5001_INT1_NORMAL);

    regData = (int1OE == SH5001_INT1_OUTPUT) \
              ? (regData | SH5001_INT1_OUTPUT) : (regData & SH5001_INT1_INPUT);

    regData = (int0Mode == SH5001_INT0_OD) \
              ? (regData | SH5001_INT0_OD) : (regData & SH5001_INT0_NORMAL);

    regData = (int0OE == SH5001_INT0_OUTPUT) \
              ? (regData | SH5001_INT0_OUTPUT) : (regData & SH5001_INT0_INPUT);

	SH5001_write(SH5001_ADDRESS, SH5001_INT_CONF, 1, &regData);

	if(intLatch == SH5001_INT_NO_LATCH)
	{
		regData = intTime;
		SH5001_write(SH5001_ADDRESS, SH5001_INT_LIMIT, 1, &regData);
	}
}




/******************************************************************
* Description:	1.config INT0 & INT1 function
*
* Parameters:	int1Mode					int0Mode
*               SH5001_INT1_NORMAL			SH5001_INT0_NORMAL
*               SH5001_INT1_OD		  		SH5001_INT0_OD
*
*				int1OE						int0OE		
*				SH5001_INT1_OUTPUT			SH5001_INT0_OUTPUT
*				SH5001_INT1_INPUT			SH5001_INT1_INPUT
*
* return:	void
* 
******************************************************************/
void SH5001_INT_Output_Config(	unsigned char int1Mode,
								unsigned char int0Mode,
								unsigned char int1OE,
								unsigned char int0OE)
{
	unsigned char regData = 0;	 
	
	SH5001_read(SH5001_ADDRESS, SH5001_INT_CONF, 1, &regData);
		
	regData = (int1Mode == SH5001_INT1_OD) \
              ? (regData | SH5001_INT1_OD) : (regData & SH5001_INT1_NORMAL);
	
	regData = (int1OE == SH5001_INT1_OUTPUT) \
              ? (regData | SH5001_INT1_OUTPUT) : (regData & SH5001_INT1_INPUT);
  
	regData = (int0Mode == SH5001_INT0_OD) \
              ? (regData | SH5001_INT0_OD) : (regData & SH5001_INT0_NORMAL);
	
	regData = (int0OE == SH5001_INT0_OUTPUT) \
              ? (regData | SH5001_INT0_OUTPUT) : (regData & SH5001_INT1_INPUT);			
	
	SH5001_write(SH5001_ADDRESS, SH5001_INT_CONF, 1, &regData);	 
}


/******************************************************************
* Description:	orientation config;
*
* Parameters: 	orientBlockMode				orientMode			    	orientTheta
*               SH5001_ORIENT_BLOCK_MODE0	SH5001_ORIENT_SYMM			(1 Byte)
*               SH5001_ORIENT_BLOCK_MODE1	SH5001_ORIENT_HIGH_ASYMM
*               SH5001_ORIENT_BLOCK_MODE2	SH5001_ORIENT_LOW_ASYMM
*               SH5001_ORIENT_BLOCK_MODE3
*
*               orientG1point5				orientSlope					orientHyst
*               (2 Bytes)					(2 Bytes)					(2 Bytes)
*  		
* return:	void
* 
******************************************************************/
void SH5001_INT_Orient_Config(	unsigned char 	orientBlockMode,
                                unsigned char 	orientMode,
                                unsigned char	orientTheta,
                                unsigned short	orientG1point5,
                                unsigned short 	orientSlope,
                                unsigned short 	orientHyst)
{
	unsigned char regData[2] = {0};	
	
	SH5001_read(SH5001_ADDRESS, SH5001_ORIEN_INTCONF0, 1, &regData[0]);
	regData[0] |= (regData[0] & 0xC0U) | (orientTheta & 0x3FU); 
	SH5001_write(SH5001_ADDRESS, SH5001_ORIEN_INTCONF0, 1, &regData[0]);

	SH5001_read(SH5001_ADDRESS, SH5001_ORIEN_INTCONF1, 1, &regData[0]);
	regData[0] |= (regData[0] & 0xF0U) | (orientBlockMode & 0x0CU) | (orientMode & 0x03U);
	SH5001_write(SH5001_ADDRESS, SH5001_ORIEN_INTCONF1, 1, &regData[0]);
	
	regData[0] = (unsigned char)orientG1point5;
	regData[1] = (unsigned char)(orientG1point5 >> 8U);
	SH5001_write(SH5001_ADDRESS, SH5001_ORIEN_INT_1G5_LOW, 2, &regData[0]);
	
	regData[0] = (unsigned char)orientSlope;
	regData[1] = (unsigned char)(orientSlope >> 8U);
	SH5001_write(SH5001_ADDRESS, SH5001_ORIEN_INT_SLOPE_LOW, 2, &regData[0]);	
	
	regData[0] = (unsigned char)orientHyst;
	regData[1] = (unsigned char)(orientHyst >> 8U);
	SH5001_write(SH5001_ADDRESS, SH5001_ORIEN_INT_HYST_LOW, 2, &regData[0]);	 
}



// Jin: flatTimeTH 200,400,800,000, sure? 

/******************************************************************
* Description:	flat INT time threshold, unit: mS;
*
* Parameters: 	flatTimeTH					flatTanHeta2
*               SH5001_FLAT_TIME_200MS		0 ~ 63
*               SH5001_FLAT_TIME_400MS
*               SH5001_FLAT_TIME_800MS
*               SH5001_FLAT_TIME_000MS
*  		
* return:	void
* 
******************************************************************/
void SH5001_INT_Flat_Config(unsigned char flatTimeTH, 
							unsigned char flatTanHeta2)
{
	unsigned char regData = 0;	
  
    SH5001_read(SH5001_ADDRESS, SH5001_ORIEN_INTCONF1, 1, &regData);	
    regData = (regData & 0xF3U) | SH5001_ORIENT_BLOCK_MODE1;
    SH5001_write(SH5001_ADDRESS, SH5001_ORIEN_INTCONF1, 1, &regData);	
  
    SH5001_read(SH5001_ADDRESS, SH5001_FLAT_INT_CONF, 1, &regData);	
	regData =  (flatTimeTH & 0xC0U) | (flatTanHeta2 & 0x3FU);
	SH5001_write(SH5001_ADDRESS, SH5001_FLAT_INT_CONF, 1, &regData);		
}	

// Jin: Act  and inAct function --- actIntClear ?

/******************************************************************
* Description:	activity INT config;
*				actIntThres: 1 LSB is 0.24mg@+/-2G, 0.48mg@+/-4G, 0.97mg@+/-8G, 1.95mg@+/-16G
*
* Parameters: 	actEnDisIntX			actEnDisIntY			actEnDisIntZ
*               SH5001_ACT_X_INT_EN		SH5001_ACT_Y_INT_EN		SH5001_ACT_Z_INT_EN
*               SH5001_ACT_X_INT_DIS	SH5001_ACT_Y_INT_DIS	SH5001_ACT_Z_INT_DIS
*								
*								
*               actTimeNum		actIntThres		actLinkStatus				actIntClear						
*               (1 Byte)		(2 Byte2)		SH5001_LINK_PRE_STA			SH5001_ACT_INACT_CLR_STATUS1
*               								SH5001_LINK_PRE_STA_NO		SH5001_ACT_INACT_CLR_STATUS3
*
* return:	void
* 
******************************************************************/
void SH5001_INT_Act_Config(	unsigned char actEnDisIntX,
                            unsigned char actEnDisIntY,
                            unsigned char actEnDisIntZ,
                            unsigned char actTimeNum,
                            unsigned short actIntThres,
                            unsigned char actLinkStatus,
                            unsigned char actIntClear)
{
	unsigned char regData[2] = {0};	
	
	SH5001_read(SH5001_ADDRESS, SH5001_ACT_INACT_INT_CONF, 1, &regData[0]);	
	regData[0] |= (regData[0] & 0x8FU) | actEnDisIntX | actEnDisIntY | actEnDisIntZ;
	SH5001_write(SH5001_ADDRESS, SH5001_ACT_INACT_INT_CONF, 1, &regData[0]);	
	
	regData[0] = (unsigned char)actIntThres;
	regData[1] = (unsigned char)(actIntThres >> 8U);
	//SH5001_write(SH5001_ADDRESS, SH5001_ACT_INT_THRESHOLDL, 2, &regData[0]);
	SH5001_write(SH5001_ADDRESS, SH5001_ACT_INT_THRESHOLDL, 1, &regData[0]);
	SH5001_write(SH5001_ADDRESS, SH5001_ACT_INT_THRESHOLDH, 1, &regData[1]);

	regData[0] = actTimeNum;
	SH5001_write(SH5001_ADDRESS, SH5001_ACT_INT_TIME, 1, &regData[0]);
	
	SH5001_read(SH5001_ADDRESS, SH5001_ACT_INACT_INT_LINK, 1, &regData[0]);	
	regData[0] |= (regData[0] & 0x7EU) | actLinkStatus | actIntClear;
	SH5001_write(SH5001_ADDRESS, SH5001_ACT_INACT_INT_LINK, 1, &regData[0]);	   
}	


/******************************************************************
* Description:	inactivity INT config;
*
* Parameters: 	inactEnDisIntX              inactEnDisIntY				inactEnDisIntZ
*               SH5001_INACT_X_INT_EN		SH5001_INACT_Y_INT_EN		SH5001_INACT_Z_INT_EN
*               SH5001_INACT_X_INT_DIS	    SH5001_INACT_Y_INT_DIS	    SH5001_INACT_Z_INT_DIS
*																
*               inactLinkStatus				inactTimeMs					inactIntThres
*               SH5001_LINK_PRE_STA			Unit: mS					(2 Bytes)				
*               SH5001_LINK_PRE_STA_NO
*
*               inactG1						inactIntClear
*               (2 Bytes)					SH5001_ACT_INACT_CLR_STATUS1
*											SH5001_ACT_INACT_CLR_STATUS3
*
* return:	void
* 
******************************************************************/
void SH5001_INT_Inact_Config(	unsigned char inactEnDisIntX,
                                unsigned char inactEnDisIntY,
                                unsigned char inactEnDisIntZ,
                                unsigned char inactLinkStatus,
                                unsigned char inactTimeMs,
                                unsigned short inactIntThres,
                                unsigned short inactG1,
                                unsigned char inactIntClear)
{
	unsigned char regData = 0;	
	
	SH5001_read(SH5001_ADDRESS, SH5001_ACT_INACT_INT_CONF, 1, &regData);		
	regData |= (regData & 0xF8U) | inactEnDisIntX | inactEnDisIntY | inactEnDisIntZ;
	SH5001_write(SH5001_ADDRESS, SH5001_ACT_INACT_INT_CONF, 1, &regData);	
	
	regData = (unsigned char)inactIntThres;
	SH5001_write(SH5001_ADDRESS, SH5001_INACT_INT_THRESHOLDL, 1, &regData);
	regData = (unsigned char)(inactIntThres >> 8U);
	SH5001_write(SH5001_ADDRESS, SH5001_INACT_INT_THRESHOLDH, 1, &regData);	
		
	regData = inactTimeMs;
	SH5001_write(SH5001_ADDRESS, SH5001_INACT_INT_TIME, 1, &regData);	
	
	/*
	regData = (unsigned char)inactG1;
	SH5001_write(SH5001_ADDRESS, SH5001_INACT_INT_1G_REFL, 1, &regData);
	regData = (unsigned char)(inactG1 >> 8U);
	SH5001_write(SH5001_ADDRESS, SH5001_INACT_INT_1G_REFH, 1, &regData);	
		
	SH5001_read(SH5001_ADDRESS, SH5001_ACT_INACT_INT_LINK, 1, &regData);	
	regData |= (regData & 0x7EU) | inactLinkStatus | inactIntClear;
	SH5001_write(SH5001_ADDRESS, SH5001_ACT_INACT_INT_LINK, 1, &regData);		
	*/
}	


/******************************************************************
* Description:	1.tap INT config;
*				2.tapIntThres is between 0 and 8191;
*               3.tapWaitTimeWindowMs is more than tapWaitTimeMs;
*
* Parameters: 	tapEnDisIntX				tapEnDisIntY				tapEnDisIntZ
*               SH5001_TAP_X_INT_EN			SH5001_TAP_Y_INT_EN			SH5001_TAP_Z_INT_EN
*               SH5001_TAP_X_INT_DIS		SH5001_TAP_Y_INT_DIS		SH5001_TAP_Z_INT_DIS
*																
*               tapIntThres		tapTimeMs	    tapWaitTimeMs	    	tapWaitTimeWindowMs
*               (13 bits)		Unit: mS		Unit: mS				Unit: 2mS
*								
* return:	void
* 
******************************************************************/
void SH5001_INT_Tap_Config(	unsigned char tapEnDisIntX,
                            unsigned char tapEnDisIntY,
                            unsigned char tapEnDisIntZ,
                            unsigned char tapIntThres,
                            unsigned char tapTimeMs,
                            unsigned char tapWaitTimeMs,
                            unsigned char tapWaitTimeWindowMs)
{
	unsigned char regData = 0;		
	
	SH5001_read(SH5001_ADDRESS, SH5001_ACT_INACT_INT_LINK, 1, &regData);	
	regData |= (regData & 0xF1U) | tapEnDisIntX | tapEnDisIntY | tapEnDisIntZ;
	SH5001_write(SH5001_ADDRESS, SH5001_ACT_INACT_INT_LINK, 1, &regData);																			
																			
	regData = (unsigned char)tapIntThres; 
	SH5001_write(SH5001_ADDRESS, SH5001_TAP_INT_THRESHOLDL, 1, &regData);
	SH5001_read(SH5001_ADDRESS, SH5001_TAP_INT_THRESHOLDH, 1, &regData);
	regData = ((unsigned char)(tapIntThres >> 8U) & 0x1FU) | (regData & 0xE0U); 
	SH5001_write(SH5001_ADDRESS, SH5001_TAP_INT_THRESHOLDH, 1, &regData);
	
	regData = tapTimeMs; 
	SH5001_write(SH5001_ADDRESS, SH5001_TAP_INT_DURATION, 1, &regData);	
	
	regData = tapWaitTimeMs; 
	SH5001_write(SH5001_ADDRESS, SH5001_TAP_INT_LATENCY, 1, &regData);	

	regData = tapWaitTimeWindowMs; 
	SH5001_write(SH5001_ADDRESS, SH5001_DTAP_INT_WINDOW, 1, &regData);	  
}


/******************************************************************
* Description:	1.SMD INT config;
*				2.smdBlockTimeS:0-0.5S;		1-1.0S;		2-1.5S;		3-2.0S;
*								4-3.0S;		5-4.0S;		6-5.0S;		7-6.0S;
*								8-7.0S;		9-8.0S;		A-9.0S;		B-10S;
*								C-12S;		D-14S;		E-15S;		F-16S;
*
*               3.smdProofTimeS:0-0.2S;		1-0.25S;	2-0.3S;		3-0.4S;
*								4-0.5S;		5-0.6S;		6-0.7S;		7-0.8S;
*								8-0.9S;		9-1.0S;		A-1.2S;		B-1.5S;
*								C-1.8S;		D-2.1S;		E-2.4S;		F-2.8S;
*
* Parameters: 	smdThres		smdBlockTimeS		smdProofTimeS
*				2 Bytes			Unit: S				Unit: S
*
* return:	void
* 
******************************************************************/
void SH5001_INT_SMD_Config(	unsigned short smdThres,
                            unsigned char smdBlockTimeS,
							unsigned char smdProofTimeS)
{
	unsigned char regData[2] = {0};		
						
	regData[0] = (unsigned char)smdThres; 
	regData[1] = (unsigned char)(smdThres >> 8U); 
	SH5001_write(SH5001_ADDRESS, SH5001_SMD_INT_THRESHOLDL, 2, &regData[0]);	

	regData[0] |= (unsigned char)((smdBlockTimeS & 0xF0U) << 4U) | (smdProofTimeS & 0xF0U);
	SH5001_write(SH5001_ADDRESS, SH5001_HIGHG_INT_CONF, 1, &regData[0]);	
}


/******************************************************************
* Description:	1.highG INT config;
*				2.highGThres is between 0 and 8191;
*               3.highGThres: x=0.5mG@2G, x=1mG@4G, x=2mG@8G, x=4mG@16G
*
* Parameters: 	highGEnDisIntX              highGEnDisIntY              highGEnDisIntZ				
*               SH5001_HIGHG_X_INT_EN		SH5001_HIGHG_Y_INT_EN		SH5001_HIGHG_Z_INT_EN		
*               SH5001_HIGHG_X_INT_DIS		SH5001_HIGHG_Y_INT_DIS		SH5001_HIGHG_Z_INT_DIS
*			
*               highGEnDisIntAll			highGThres					highGTimeMs
*               SH5001_HIGHG_ALL_INT_EN		Unit: x*8mG			    	Unit: 2mS
*               SH5001_HIGHG_ALL_INT_DIS	(13 bits)
*
* return:	void
* 
******************************************************************/
void SH5001_INT_HighG_Config(	unsigned char highGEnDisIntX,
                                unsigned char highGEnDisIntY,
                                unsigned char highGEnDisIntZ,
                                unsigned char highGEnDisIntAll,
                                unsigned short highGThres,
                                unsigned char highGTimeMs)
{
	unsigned char regData[2] = {0};		
	
	SH5001_read(SH5001_ADDRESS, SH5001_HIGHG_INT_CONF, 1, &regData[0]);	
	regData[0] |= (regData[0] & 0x0FU) | highGEnDisIntX | highGEnDisIntY | highGEnDisIntZ | highGEnDisIntAll;
	SH5001_write(SH5001_ADDRESS, SH5001_HIGHG_INT_CONF, 1, &regData[0]);		
				
	SH5001_read(SH5001_ADDRESS, SH5001_HIGHG_INT_THRESHOLDL, 2, &regData[0]);			
	regData[0] = (unsigned char)highGThres; 
	regData[1] = (regData[1] & 0xE0U) | ((unsigned char)(highGThres >> 8U) & 0x1FU); 
	SH5001_write(SH5001_ADDRESS, SH5001_HIGHG_INT_THRESHOLDL, 2, &regData[0]);	
			
	regData[0] = highGTimeMs; 
	SH5001_write(SH5001_ADDRESS, SH5001_HIGHG_INT_TIME, 1, &regData[0]);	 
}


/******************************************************************
* Description:	1.lowG INT config;
*				2.lowGThres is between 0 and 8191;
*               3.lowGThres: x=0.5mG@2G or x=1mG@4G or x=2mG@8G or x=4mG@16G
*
* Parameters: 	lowGEnDisIntAll	            lowGThres		lowGTimeMs
*               SH5001_LOWG_ALL_INT_EN		Unit: x*8mG		Unit: 2mS
*               SH5001_LOWG_ALL_INT_DIS		(13 bits)
*
* return:	void
* 
******************************************************************/
void SH5001_INT_LowG_Config(	unsigned char lowGEnDisIntAll,
                                unsigned short lowGThres,
                                unsigned char lowGTimeMs)
{
	unsigned char regData[2] = {0};		
	
	SH5001_read(SH5001_ADDRESS, SH5001_HIGHG_INT_CONF, 1, &regData[0]);	
	regData[0] |= (regData[0] & 0xFEU) | lowGEnDisIntAll;
	SH5001_write(SH5001_ADDRESS, SH5001_HIGHG_INT_CONF, 1, &regData[0]);	
	
	SH5001_read(SH5001_ADDRESS, SH5001_LOWG_INT_THRESHOLDL, 2, &regData[0]);	
	regData[0] = (unsigned char)lowGThres; 
	regData[1] = (regData[1] & 0xE0U) | ((unsigned char)(lowGThres >> 8U) & 0x1FU);
	SH5001_write(SH5001_ADDRESS, SH5001_LOWG_INT_THRESHOLDL, 2, &regData[0]);	
	
	regData[0] = lowGTimeMs; 
	SH5001_write(SH5001_ADDRESS, SH5001_LOWG_INT_TIME, 1, &regData[0]);		
}


/******************************************************************
* Description:	1.freeFall INT config;
*               2.freeFallThres: x=0.5mG@2G or x=1mG@4G or x=2mG@8G or x=4mG@16G
*
* Parameters: 	freeFallThres	freeFallTimeMs
*               Unit: x*8mG		Unit: 2mS
*
* return:	void
* 
******************************************************************/
void SH5001_INT_FreeFall_Config(	unsigned char freeFallThres,
                                    unsigned char freeFallTimeMs)
{
	unsigned char regData = 0;
	
	regData = freeFallThres; 
	SH5001_write(SH5001_ADDRESS, SH5001_FREEFALL_INT_THRES, 1, &regData);	
	
	regData = freeFallTimeMs; 
	SH5001_write(SH5001_ADDRESS, SH5001_FREEFALL_INT_TIME, 1, &regData);																						
}



/******************************************************************
* Description:	1.read INT status0
*
* Parameters: 	void
*
* return:	unsigned short
*			bit 13: Tap, Single or Double Tap Interrupt status	
*			bit 12: SMD Interrupt status
*           bit 11: FIFO Watermark Interrupt status 							
*           bit 10: Gyro Data Ready Interrupt status
*           bit 9: Acc Data Ready Interrupt status
*           bit 8: Free-fall Interrupt status 
*           bit 7: Low-G Interrupt status
*           bit 6: High-G Interrupt status
*           bit 5: Inactivity Interrupt status
*           bit 4: Activity Interrupt status
*           bit 3: Double Tap Interrupt status 
*           bit 2: Single Tap Interrupt status
*           bit 1: Flat Interrupt status
*           bit 0: Orientation Interrupt status 
*               0: Not Active
*               1: Active
*
******************************************************************/
unsigned short SH5001_INT_Read_Status01(void)
{
	unsigned char regData[3] = {0};

	SH5001_read(SH5001_ADDRESS, SH5001_INT_STA0, 3, &regData[0]);

	return( ((unsigned short)(regData[1] & 0x3FU) << 8U) | regData[0] );
}


/******************************************************************
* Description:	1.read INT status2
*
* Parameters: 	void
*
* return:	unsigned char
*           bit 7: High-G Sign
*               0: Positive
*               1: Negative
*           bit 6: High-G X-axis Interrupt status
*           bit 5: High-G Y-axis Interrupt status
*           bit 4: High-G Z-axis Interrupt status
*               0: No Active
*               1: Active
*           bit 3: Reserved
*           bit 2: Reserved
*           bit 1: Reserved
*           bit 0: Low-G Interrupt status
*               0: Not Active
*               1: Active
*
******************************************************************/
unsigned char SH5001_INT_Read_Status2(void)
{
	unsigned char regData = 0;	
	
	SH5001_read(SH5001_ADDRESS, SH5001_INT_STA2, 1, &regData);	

	return( (regData & 0xF1U));
}


/******************************************************************
* Description:	1.read INT status3
*
* Parameters: 	void
*
* return:	unsigned char
*           bit 7: Activity-Inactivity Sign
*               0: Positive
*               1: Negative
*           bit 6: Activity-Inactivity X-axis Interrupt Status
*           bit 5: Activity-Inactivity Y-axis Interrupt Status
*           bit 4: Activity-Inactivity Z-axis Interrupt Status
*               0: No Active
*               1: Active
*			bit 3: Sign of acceleration that trigger Single or Double Tap Interrup
*               0: Positive
*               1: Negative
*			bit 2: Whether Single or Double Tap Interrupt is triggered by X axis
*			bit 1: Whether Single or Double Tap Interrupt is triggered by Y axis
*			bit 0: Whether Single or Double Tap Interrupt is triggered by Z axis
*               0: No
*               1: Yes
******************************************************************/
unsigned char SH5001_INT_Read_Status3(void)
{
	unsigned char regData = 0;	
	
	SH5001_read(SH5001_ADDRESS, SH5001_INT_STA3, 1, &regData);	

	return(regData);
}


/******************************************************************
* Description:	1.read INT status4
*
* Parameters: 	void
*
* return:	unsigned char
*           bit 7: Reserved
*           bit 6: Reserved
*           bit 5: Reserved
*           bit 4: Reserved
*           bit 3: Reserved
*           bit 2: Orientation Interrupt Value of Z-axis
*               0: Upward
*               1: Downward
*           bit 1..0: Orientation Interrupt Value of X and Y-axis
*               00: Landscape left
*               01: Landscape right
*               10: Portrait upside down
*               11: Portrait upright
*
******************************************************************/
unsigned char SH5001_INT_Read_Status4(void)
{
	unsigned char regData = 0;	
	
	SH5001_read(SH5001_ADDRESS, SH5001_INT_STA4, 1, &regData);	

	return(regData & 0x07U);
}



/******************************************************************
* Description:	reset FIFO controller;
*
* Parameters: 	fifoMode
*               SH5001_FIFO_MODE_DIS
*               SH5001_FIFO_MODE_FIFO
*               SH5001_FIFO_MODE_STREAM
*               SH5001_FIFO_MODE_TRIGGER
*
* return:	void
* 
******************************************************************/
void SH5001_FIFO_Mode_Set(unsigned char fifoMode)
{
	unsigned char regData = 0;		

	SH5001_read(SH5001_ADDRESS, SH5001_FIFO_CONF0, 1, &regData);
	regData &= 0x7F;
	SH5001_write(SH5001_ADDRESS, SH5001_FIFO_CONF0, 1, &regData);
	
	regData = fifoMode & 0x03;
	SH5001_write(SH5001_ADDRESS, SH5001_FIFO_CONF0, 1, &regData);
}

void SH5001_FIFO_Reset(void)
{
	unsigned char regData = 0;

	SH5001_read(SH5001_ADDRESS, SH5001_FIFO_CONF0, 1, &regData);
	regData |= 0x80;
	SH5001_write(SH5001_ADDRESS, SH5001_FIFO_CONF0, 1, &regData);
}



/******************************************************************
* Description:	1.FIFO down sample frequency config;
*               2.fifoAccFreq:	accODR*1/2, *1/4, *1/8, *1/16, *1/32, *1/64, *1/128, *1/256
*               3.fifoGyroFreq:	gyroODR*1/2, *1/4, *1/8, *1/16, *1/32, *1/64, *1/128, *1/256
*
* Parameters: 	fifoAccDownSampleEnDis		fifoAccFreq/fifoGyroFreq	fifoGyroDownSampleEnDis
*               SH5001_FIFO_ACC_DOWNS_EN	SH5001_FIFO_FREQ_X1_2		SH5001_FIFO_GYRO_DOWNS_EN
*               SH5001_FIFO_ACC_DOWNS_DIS	SH5001_FIFO_FREQ_X1_4		SH5001_FIFO_GYRO_DOWNS_DIS
*											SH5001_FIFO_FREQ_X1_8
*											SH5001_FIFO_FREQ_X1_16
*											SH5001_FIFO_FREQ_X1_32
*											SH5001_FIFO_FREQ_X1_64
*											SH5001_FIFO_FREQ_X1_128
*											SH5001_FIFO_FREQ_X1_256
* return:	void
* 
******************************************************************/
void SH5001_FIFO_Freq_Config(	unsigned char fifoAccDownSampleEnDis,
                                unsigned char fifoAccFreq,
                                unsigned char fifoGyroDownSampleEnDis,
                                unsigned char fifoGyroFreq)
{
	unsigned char regData = 0;		
	
	regData |= fifoAccDownSampleEnDis | fifoGyroDownSampleEnDis;
	regData |= (fifoAccFreq << 4) | fifoGyroFreq;
	SH5001_write(SH5001_ADDRESS, SH5001_FIFO_CONF4, 1, &regData); 
}


// Jin: fifoWaterMarkLevel???
/******************************************************************
* Description:	1.config data type in FIFO;
*               2.fifoWaterMarkLevel <= channelNum*2*(INT(1024/(channelNum*2))-1)
*                 eg. fifomode = SH5001_FIFO_ACC_X_EN | SH5001_FIFO_ACC_Y_EN | SH5001_FIFO_ACC_Z_EN
*                     then fifoWaterMarkLevel maximum is 1014
*
* Parameters: 	fifoMode						fifoWaterMarkLevel
				SH5001_FIFO_TIMESTAMP_EN		<=1024			          
*               SH5001_FIFO_EXT_Z_EN
*               SH5001_FIFO_EXT_Y_EN
*               SH5001_FIFO_EXT_X_EN							
*               SH5001_FIFO_TEMPERATURE_EN							
*               SH5001_FIFO_GYRO_Z_EN							
*               SH5001_FIFO_GYRO_Y_EN						
*               SH5001_FIFO_GYRO_X_EN
*               SH5001_FIFO_ACC_Z_EN
*               SH5001_FIFO_ACC_Y_EN
*               SH5001_FIFO_ACC_X_EN
*               SH5001_FIFO_ALL_DIS
*															
* return:	void
* 
******************************************************************/
void SH5001_FIFO_Data_Config(	unsigned short fifoMode,
                                unsigned short fifoWaterMarkLevel)
{
	unsigned char regData = 0;
	
	if(fifoWaterMarkLevel > 1024U)
	{
		fifoWaterMarkLevel = 1024U;
	}
	
	SH5001_read(SH5001_ADDRESS, SH5001_FIFO_CONF2, 1, &regData);
	regData = (regData & 0x88U) \
              | ((unsigned char)(fifoMode >> 8U) & 0x70U) \
              | (((unsigned char)(fifoWaterMarkLevel >> 8U)) & 0x07U);
	SH5001_write(SH5001_ADDRESS, SH5001_FIFO_CONF2, 1, &regData);

	regData = (unsigned char)fifoMode;
	SH5001_write(SH5001_ADDRESS, SH5001_FIFO_CONF3, 1, &regData);
	
	regData = (unsigned char)fifoWaterMarkLevel;
	SH5001_write(SH5001_ADDRESS, SH5001_FIFO_CONF1, 1, &regData);            
}



/******************************************************************
* Description:	1. read Fifo status and fifo entries count
*
* Parameters: 	*fifoEntriesCount, store fifo entries count, less than or equal to 1024;
*
*															
* return:	unsigned char
*           bit 7: 0
*           bit 6: 0
*           bit 5: Whether FIFO Watermark has been reached
*           bit 4: Whether FIFO is full
*           bit 3: Whether FIFO is empty
*               0: No
*               1: Yes
*           bit 2: 0
*           bit 1: 0
*           bit 0: 0
*
*
******************************************************************/
unsigned char SH5001_FIFO_Read_Status(unsigned short int *fifoEntriesCount)
{
	unsigned char regData[2] = {0};		

	SH5001_read(SH5001_ADDRESS, SH5001_FIFO_STA0, 2, &regData[0]);

	*fifoEntriesCount = ((unsigned short int)(regData[1] & 0x0FU) << 8U) | regData[0];

	
	return (regData[1] & 0x70U);
}


/******************************************************************
* Description:	1. read Fifo data
*
* Parameters: 	*fifoReadData		fifoDataLength
*               data				data length
*															
* return:	void
*
*
******************************************************************/
void SH5001_FIFO_Read_Data(unsigned char *fifoReadData, unsigned short int fifoDataLength)
{
    if(fifoDataLength == 0)
	{
		return;
	} 
    
	if(fifoDataLength > 1024U)
	{
		fifoDataLength = 1024U;
	} 
	
	SH5001_read(SH5001_ADDRESS, SH5001_FIFO_DATA, fifoDataLength, fifoReadData);
	/*
    while(fifoDataLength--)
	{
		SH5001_read(SH5001_ADDRESS, SH5001_FIFO_DATA, 1, fifoReadData);
		fifoReadData++;
	}  
	*/
}	


/******************************************************************
* Description:	reset Mater I2C;
*
* Parameters: 	void
*															
* return:	void
* 
******************************************************************/
void SH5001_MI2C_Reset(void)
{
	unsigned char regData = 0;		
	
	SH5001_read(SH5001_ADDRESS, SH5001_MI2C_CONF0, 1, &regData);
	regData |= 0x80U;
	SH5001_write(SH5001_ADDRESS, SH5001_MI2C_CONF0, 1, &regData);
	regData &= 0x7FU;
	SH5001_write(SH5001_ADDRESS, SH5001_MI2C_CONF0, 1, &regData);	
	
}


/******************************************************************
* Description:	1.master I2C config;
*               2.master I2C clock frequency is (1MHz/(6+3*mi2cFreq));
*               3.mi2cSlaveAddr: slave device address, 7 bits;
*
*
* Parameters: 	mi2cReadMode					mi2cODR							mi2cFreq
*               SH5001_MI2C_READ_MODE_AUTO		SH5001_MI2C_READ_ODR_200HZ		<=15
*               SH5001_MI2C_READ_MODE_MANUAL	SH5001_MI2C_READ_ODR_100HZ
*												SH5001_MI2C_READ_ODR_50HZ		
*												SH5001_MI2C_READ_ODR_25HZ																							
* return:	void
* 
******************************************************************/
void SH5001_MI2C_Bus_Config(	unsigned char mi2cReadMode,
                                unsigned char mi2cODR,
                                unsigned char mi2cFreq)
{
	unsigned char regData = 0;		
	
	if(mi2cFreq > 15U)
	{
		mi2cFreq = 15U;
	}
	
	SH5001_read(SH5001_ADDRESS, SH5001_MI2C_CONF1, 1, &regData);
	regData = (regData & 0xC0U) | (mi2cODR & 0x30U) | mi2cFreq;
	SH5001_write(SH5001_ADDRESS, SH5001_MI2C_CONF1, 1, &regData);	
	
	SH5001_read(SH5001_ADDRESS, SH5001_MI2C_CONF0, 1, &regData);
	regData = (regData & 0xBFU) | mi2cReadMode;
	SH5001_write(SH5001_ADDRESS, SH5001_MI2C_CONF0, 1, &regData);		
}


/******************************************************************
* Description:	1.master I2C address and command config;
*               2.mi2cSlaveAddr: slave device address, 7 bits;
*
* Parameters: 	mi2cSlaveAddr		mi2cSlaveCmd		mi2cReadMode
*               (1 Byte)			(1 Byte)			SH5001_MI2C_READ_MODE_AUTO
*                                                       SH5001_MI2C_READ_MODE_MANUAL
*																						
* return:	void
* 
******************************************************************/
void SH5001_MI2C_Cmd_Config(	unsigned char mi2cSlaveAddr,
                                unsigned char mi2cSlaveCmd,
                                unsigned char mi2cReadMode)
{
	unsigned char regData = 0;	

	SH5001_read(SH5001_ADDRESS, SH5001_MI2C_CONF0, 1, &regData);
	regData = (regData & 0xBFU) | mi2cReadMode;
	SH5001_write(SH5001_ADDRESS, SH5001_MI2C_CONF0, 1, &regData);
	
	SH5001_read(SH5001_ADDRESS, SH5001_MI2C_CMD0, 1, &regData);
	regData |= (mi2cSlaveAddr << 1U);
	SH5001_write(SH5001_ADDRESS, SH5001_MI2C_CMD0, 1, &regData);
	SH5001_write(SH5001_ADDRESS, SH5001_MI2C_CMD1, 1, &mi2cSlaveCmd);																																							
}


/******************************************************************
* Description:	1.master I2C write data fucntion;
*               2.mi2cWriteData: write data;
*
* Parameters: 	mi2cWriteData
*																						
* return:	SH5001_TRUE or SH5001_FALSE
* 
******************************************************************/
unsigned char SH5001_MI2C_Write(unsigned char mi2cWriteData)
{
	unsigned char regData = 0;	
	unsigned char i = 0;

	SH5001_write(SH5001_ADDRESS, SH5001_MI2C_WR, 1, &mi2cWriteData);
		
	//Master I2C enable, write-operation
	SH5001_read(SH5001_ADDRESS, SH5001_MI2C_CONF0, 1, &regData);
	regData = (regData & 0xFCU) | 0x02U;
	SH5001_write(SH5001_ADDRESS, SH5001_MI2C_CONF0, 1, &regData);
	
	// wait write-operation to end
	while(i++ < 20U)
	{	
		SH5001_read(SH5001_ADDRESS, SH5001_MI2C_CONF0, 1, &regData);
		if(regData & 0x30U)
			break;
	}
		
	if((regData & 0x30U) == SH5001_MI2C_SUCCESS)
	{
		return (SH5001_TRUE);	
	}	
	else
	{
		return (SH5001_FALSE);	
	}		
}																


/******************************************************************
* Description:	1.master I2C read data fucntion;
*               2.*mi2cReadData: read data;
*
* Parameters: 	*mi2cReadData
*																						
* return:	SH5001_TRUE or SH5001_FALSE
* 
******************************************************************/
unsigned char SH5001_MI2C_Read(unsigned char *mi2cReadData)
{
	unsigned char regData = 0;			
	unsigned char i = 0;
		
	
	SH5001_read(SH5001_ADDRESS, SH5001_MI2C_CONF0, 1, &regData);
	if((regData & 0x40U) == 0)
	{	
		//Master I2C enable, read-operation
		regData |= 0x03U;
		SH5001_write(SH5001_ADDRESS, SH5001_MI2C_CONF0, 1, &regData);
	}
	
	// wait read-operation to end
	while(i++ < 20U)
	{	
		SH5001_read(SH5001_ADDRESS, SH5001_MI2C_CONF0, 1, &regData);
		if(regData & 0x30U)
			break;
	}	

	SH5001_read(SH5001_ADDRESS, SH5001_MI2C_CONF0, 1, &regData);
	if((regData & 0x30U) == SH5001_MI2C_SUCCESS)
	{
		SH5001_read(SH5001_ADDRESS, SH5001_MI2C_RD, 1, &regData);
		*mi2cReadData = regData;
		
		return (SH5001_TRUE);	
	}	
	else
	{
		return (SH5001_FALSE);	
	}		
}	


/******************************************************************
* Description:	1.SPI interface config;  Default: SH5001_SPI_4_WIRE
*
* Parameters: 	spiInterfaceMode
*               SH5001_SPI_3_WIRE
*               SH5001_SPI_4_WIRE
*
* return:	void
* 
******************************************************************/
void SH5001_SPI_Config(	unsigned char spiInterfaceMode)
{
	#if 0
	unsigned char regData = 0;		
	
	SH5001_read(SH5001_ADDRESS, SH5001_SPI_CONF, 1, &regData);	
	regData = (regData & 0xDFU) | spiInterfaceMode;
	#endif
	unsigned char regData = spiInterfaceMode;

	SH5001_write(SH5001_ADDRESS, SH5001_SPI_CONF, 1, &regData);
} 


/******************************************************************
* Description:	1.I2C interface config;  Default: SH5001_I2C_EN
*
* Parameters: 	i2cInterfaceEnDis
*               SH5001_I2C_EN
*               SH5001_I2C_DIS
*
* return:	void
* 
******************************************************************/
void SH5001_I2C_Config(unsigned char i2cInterfaceEnDis)
{
	unsigned char regData = 0;		
	
	SH5001_read(SH5001_ADDRESS, SH5001_I2C_CONF, 1, &regData);	
	regData = (regData & 0x7FU) | i2cInterfaceEnDis;
	SH5001_write(SH5001_ADDRESS, SH5001_I2C_CONF, 1, &regData);
} 



// Jin
/******************************************************************
* Description:	1.I2C loop mode config;
*				2.SH5001_I2C_LP_06BYTE: ACC_X ACC_Y ACC_Z
*				3.SH5001_I2C_LP_12BYTE: ACC_X ACC_Y ACC_Z GYRO_X GYRO_Y GYRO_Z
*				4.SH5001_I2C_LP_14BYTE: ACC_X ACC_Y ACC_Z GYRO_X GYRO_Y GYRO_Z TEMPERATURE
*
* Parameters: 	i2cReadLoopMode
*               SH5001_I2C_LP_06BYTE
*               SH5001_I2C_LP_12BYTE
*				SH5001_I2C_LP_14BYTE
*				SH5001_I2C_LP_DISABLE
*
* return:	void
* 
******************************************************************/
void SH5001_I2C_LP_Config(unsigned char i2cReadLoopMode)
{
	unsigned char regData = 0;		
	
	SH5001_read(SH5001_ADDRESS, SH5001_I2C_CONF, 1, &regData);	
	regData = (regData & 0xF8U) | i2cReadLoopMode;
	SH5001_write(SH5001_ADDRESS, SH5001_I2C_CONF, 1, &regData);
} 


/******************************************************************
* Description:	1.OIS SPI interface config;  Default: SH5001_OIS_SPI_DIS
*
* Parameters: 	oisSpiInterfaceMode		oisSpiInterfaceEnDis
*               SH5001_OIS_SPI_3WIRE	SH5001_OIS_SPI_EN
*               SH5001_OIS_SPI_4WIRE	SH5001_OIS_SPI_DIS
*
* return:	void
* 
******************************************************************/

void SH5001_OIS_SPI_Config(unsigned char oisSpiInterfaceEnDis)
{
  unsigned char regData = 0;

  SH5001_read(SH5001_ADDRESS, SH5001_OIS_SPI_CONF, 1, &regData);
  regData = (regData & 0xEFU) | oisSpiInterfaceEnDis;
  SH5001_write(SH5001_ADDRESS, SH5001_OIS_SPI_CONF, 1, &regData);
}


/******************************************************************
* Description:	1.Switch SH5001 power mode;
*               2.SH5001_NORMAL_MODE
*				3.SH5001_ACC_HP_MODE
*               4.SH5001_ACC_LP_MODE
*				5.SH5001_POWERDOWN_MODE
*
* Parameters:   powerMode
*               SH5001_NORMAL_MODE
*               SH5001_ACC_HP_MODE
*				SH5001_ACC_LP_MODE
*               SH5001_POWERDOWN_MODE
*
* return:	SH5001_FALSE or SH5001_TRUE
*
******************************************************************/
unsigned char SH5001_O1_SwitchPowerMode(unsigned char powerMode)
{

	int i = 0;
	unsigned char reg_addr[SH5001_POWMODE_REG_NUM_O1] = { 0x30, 0x20, 0x2B, 0xBD, 0xD9, 0xD1, 0xD4, 0xD5, 0xD6, 0xD7, 0xD8};
	unsigned char reg_data_normal[SH5001_POWMODE_REG_NUM_O1] = { 0x08, 0x23, 0x00, 0x0C, 0x99, 0x68, 0x00, 0x00, 0x01, 0x50, 0x00};
	unsigned char reg_data_acc_hp[SH5001_POWMODE_REG_NUM_O1] = { 0x08, 0x23, 0x01, 0x0C, 0x98, 0x6B, 0x00, 0x7A, 0xFF, 0xFC, 0x00};
	unsigned char reg_data_acc_lp[SH5001_POWMODE_REG_NUM_O1] = { 0x08, 0xA1, 0x01, 0x0E, 0x98, 0x6B, 0x90, 0xFA, 0xFF, 0xFC, 0x00};
	unsigned char reg_data_powerdown[SH5001_POWMODE_REG_NUM_O1] = { 0x08, 0xA1, 0x01, 0x0E, 0x98, 0x6B, 0xF1, 0xFE, 0xFF, 0xFC, 0xE2};
	unsigned char accODR = SH5001_ACC_ODR_2000HZ;
    unsigned char regData = 0;

	if(powerMode > 0x03)
	{
		return (SH5001_FALSE);
	}

	switch(powerMode)
	{
	case SH5001_NORMAL_MODE:

		SH5001_read(SH5001_ADDRESS, SH5001_ACC_CONF2, 1, &regData);
		regData = regData & 0x7F;
		SH5001_write(SH5001_ADDRESS, SH5001_ACC_CONF2, 1, &regData);

		SH5001_write(SH5001_ADDRESS, SH5001_ACC_CONF1, 1, &storeAccODR);
		for(i = 0; i < SH5001_POWMODE_REG_NUM_O1; i++)
		{
			SH5001_write(SH5001_ADDRESS, reg_addr[i], 1, &reg_data_normal[i]);
		}

		SH5001_DriveStart();
		SH5001_ADCReset();
		SH5001_CVAReset();
		delay_ms(100);// or discard 10
#if SENODIA_VDD_3V3
		SH5001_AccReset();
#endif
		return (SH5001_TRUE);

	case SH5001_ACC_HP_MODE:

		SH5001_read(SH5001_ADDRESS, SH5001_ACC_CONF2, 1, &regData);
		regData = regData | 0x80;
		SH5001_write(SH5001_ADDRESS, SH5001_ACC_CONF2, 1, &regData);

		SH5001_write(SH5001_ADDRESS, SH5001_ACC_CONF1, 1, &storeAccODR);
		for(i = 0; i < SH5001_POWMODE_REG_NUM_O1; i++)
		{
			SH5001_write(SH5001_ADDRESS, reg_addr[i], 1, &reg_data_acc_hp[i]);
		}

		delay_ms(30); // or discard 2

		return (SH5001_TRUE);

	case SH5001_ACC_LP_MODE:

		SH5001_read(SH5001_ADDRESS, SH5001_ACC_CONF2, 1, &regData);
		regData = regData & 0x7F;
		SH5001_write(SH5001_ADDRESS, SH5001_ACC_CONF2, 1, &regData);

		regData = (storeAccODR & 0xF0) | accODR;
		SH5001_write(SH5001_ADDRESS, SH5001_ACC_CONF1, 1, &regData);
		for(i = 0; i < SH5001_POWMODE_REG_NUM_O1; i++)
		{
			SH5001_write(SH5001_ADDRESS, reg_addr[i], 1, &reg_data_acc_lp[i]);
		}

		delay_ms(30); // or discard 2
		return (SH5001_TRUE);

	case SH5001_POWERDOWN_MODE:

		for(i = 0; i < SH5001_POWMODE_REG_NUM_O1; i++)
		{
			SH5001_write(SH5001_ADDRESS, reg_addr[i], 1, &reg_data_powerdown[i]);
		}

		return (SH5001_TRUE);

	default:
		return (SH5001_FALSE);
	}
}

int gain_first = 0;
unsigned char RegOld[SH5001_CALI_REG_NUM] = {0};
unsigned char RegNew[SH5001_CALI_REG_NUM] = {0};
unsigned char SH5001_O2_SwitchPowerMode(unsigned char powerMode)
{

	int i = 0;
	unsigned char reg_data = 0;
	unsigned char reg_addr_cali[SH5001_CALI_REG_NUM] = {0x28, 0xDB, 0x6A, 0x6B, 0x75, 0x76, 0x81, 0x82};
    short RatioList[16] = { 1344, 1376, 1408, 1440, 1472, 1504, 1536, 1568, 1312, 1280, 1248, 1216, 1184, 1152, 1120, 1088 };
	unsigned char reg_addr[SH5001_POWMODE_REG_NUM_O2] = {0x20, 0x30, 0x2B, 0xBD, 0xD1, 0xD4, 0xD5, 0xD6, 0xD7, 0xD8, 0xD9, 0x69, 0x74, 0x80};
	unsigned char reg_data_normal[SH5001_POWMODE_REG_NUM_O2] = {0x27, 0x08, 0x00, 0x0C, 0x68, 0x00, 0x00, 0x01, 0x50, 0x00, 0x99, 0x0C, 0x0C, 0x0C};
	unsigned char reg_data_acc_hp[SH5001_POWMODE_REG_NUM_O2] = {0x27, 0x08, 0x01, 0x2C, 0x6B, 0x00, 0x7A, 0xFF, 0xFC, 0x00, 0x98, 0x08, 0x08, 0x08};
	unsigned char reg_data_acc_lp[SH5001_POWMODE_REG_NUM_O2] = {0xA7, 0x08, 0x01, 0x2C, 0x6B, 0x90, 0x7A, 0xFF, 0xFC, 0x00, 0x98, 0x04, 0x04, 0x04};
	unsigned char reg_data_powerdown[SH5001_POWMODE_REG_NUM_O2] = {0xA5, 0x08, 0x01, 0x0E, 0x6B, 0xF1, 0xFE, 0xFF, 0xFC, 0xE2, 0x98, 0x0C, 0x0C, 0x0C};
	int CodeOld;
	int CodeNew;
	short RatioOld;
	short RatioNew;
	short GainOld[3];
	short GainNew[3];
	unsigned char accODR = SH5001_ACC_ODR_880HZ;
	unsigned char regData = 0;

	if(powerMode > 0x03)
	{
		return (SH5001_FALSE);
	}

	switch(powerMode)
	{
	case SH5001_NORMAL_MODE:

		SH5001_write(SH5001_ADDRESS, SH5001_ACC_CONF1, 1, &storeAccODR);
		for(i = 0; i < SH5001_POWMODE_REG_NUM_O2; i++)
		{
			SH5001_write(SH5001_ADDRESS, reg_addr[i], 1, &reg_data_normal[i]);
		}

		if(gain_first == 1)
		{
			for(i = 0; i < SH5001_CALI_REG_NUM; i++)
			{
				SH5001_write(SH5001_ADDRESS, reg_addr_cali[i], 1, &RegOld[i]);
			}
		}

		SH5001_DriveStart();
		SH5001_ADCReset();
		SH5001_CVAReset();
		//delay_ms(100);// or discard 10
#if SENODIA_VDD_3V3
		SH5001_AccReset();
#endif
		return (SH5001_TRUE);

	case SH5001_ACC_HP_MODE:

		SH5001_write(SH5001_ADDRESS, SH5001_ACC_CONF1, 1, &storeAccODR);
		for(i = 0; i < SH5001_POWMODE_REG_NUM_O2; i++)
		{
			SH5001_write(SH5001_ADDRESS, reg_addr[i], 1, &reg_data_acc_hp[i]);
		}

		if(gain_first == 0)
		{
			for(i = 0; i < SH5001_CALI_REG_NUM; i++)
			{
				SH5001_read(SH5001_ADDRESS, reg_addr_cali[i], 1, &RegOld[i]);
			}
			RegOld[6] = RegOld[6] & 0xFE;
			CodeOld = (RegOld[0] & 0xF0) >> 4;
			CodeNew = RegOld[1] & 0x0F;

			RatioOld = RatioList[CodeOld];
			RatioNew = RatioList[CodeNew];

			//计算原始加计Gain
			GainOld[0] = RegOld[2] + RegOld[3] * 256;
			GainOld[1] = RegOld[4] + RegOld[5] * 256;
			GainOld[2] = RegOld[6] + RegOld[7] * 256;
			if(GainOld[0] > 2047)
				GainOld[0] -=4096;
			if(GainOld[1] > 2047)
				GainOld[1] -=4096;
			if(GainOld[2] > 2047)
				GainOld[2] -=4096;

			//计算新的加计Gain
			GainNew[0] = GainOld[0] * ((float)RatioOld / RatioNew) * ((float)RatioOld / RatioNew);
			GainNew[1] = GainOld[1] * ((float)RatioOld / RatioNew) * ((float)RatioOld / RatioNew);
			GainNew[2] = GainOld[2] * ((float)RatioOld / RatioNew) * ((float)RatioOld / RatioNew);
			if(GainNew[0] < 0)
				GainNew[0] +=4096;
			if(GainNew[1] < 0)
				GainNew[1] +=4096;
			if(GainNew[2] < 0)
				GainNew[2] +=4096;

			//计算新的ODR Ratio和新的加计Gain对应需要写的寄存器
			RegNew[0] = (RegOld[0] & 0x0F) | (CodeNew << 4);
			RegNew[1] = (RegOld[1] & 0xF0) | CodeOld;
			RegNew[2] = GainNew[0] & 0xFF;
			RegNew[3] = (GainNew[0] & 0xF00) >> 8;
			RegNew[4] = GainNew[1] & 0xFF;
			RegNew[5] = (GainNew[1] & 0xF00) >> 8;
			RegNew[6] = GainNew[2] & 0xFE;
			RegNew[7] = (GainNew[2] & 0xF00) >> 8;
			gain_first++;
		}

		for(i = 0; i < SH5001_CALI_REG_NUM; i++)
		{
			SH5001_write(SH5001_ADDRESS, reg_addr_cali[i], 1, &RegNew[i]);
		}

		//delay_ms(30); // or discard 2

		return (SH5001_TRUE);

	case SH5001_ACC_LP_MODE:

		regData = (storeAccODR & 0xF0) | accODR;
		SH5001_write(SH5001_ADDRESS, SH5001_ACC_CONF1, 1, &regData);
		for(i = 0; i < SH5001_POWMODE_REG_NUM_O2; i++)
		{
			SH5001_write(SH5001_ADDRESS, reg_addr[i], 1, &reg_data_acc_lp[i]);
		}

		if(gain_first == 0)
		{
			for(i = 0; i < SH5001_CALI_REG_NUM; i++)
			{
				SH5001_read(SH5001_ADDRESS, reg_addr_cali[i], 1, &RegOld[i]);
			}

			CodeOld = (RegOld[0] & 0xF0) >> 4;
			CodeNew = RegOld[1] & 0x0F;

			RatioOld = RatioList[CodeOld];
			RatioNew = RatioList[CodeNew];

			GainOld[0] = RegOld[2] + RegOld[3] * 256;
			GainOld[1] = RegOld[4] + RegOld[5] * 256;
			GainOld[2] = RegOld[6] + RegOld[7] * 256;
			if(GainOld[0] > 2047)
				GainOld[0] -=4096;
			if(GainOld[1] > 2047)
				GainOld[1] -=4096;
			if(GainOld[2] > 2047)
				GainOld[2] -=4096;

			GainNew[0] = GainOld[0] * ((float)RatioOld / RatioNew) * ((float)RatioOld / RatioNew);
			GainNew[1] = GainOld[1] * ((float)RatioOld / RatioNew) * ((float)RatioOld / RatioNew);
			GainNew[2] = GainOld[2] * ((float)RatioOld / RatioNew) * ((float)RatioOld / RatioNew);
			if(GainNew[0] < 0)
				GainNew[0] +=4096;
			if(GainNew[1] < 0)
				GainNew[1] +=4096;
			if(GainNew[2] < 0)
				GainNew[2] +=4096;

			RegNew[0] = (RegOld[0] & 0x0F) | (CodeNew << 4);
			RegNew[1] = (RegOld[1] & 0xF0) | CodeOld;
			RegNew[2] = GainNew[0] & 0xFF;
			RegNew[3] = (GainNew[0] & 0xF00) >> 8;
			RegNew[4] = GainNew[1] & 0xFF;
			RegNew[5] = (GainNew[1] & 0xF00) >> 8;
			RegNew[6] = GainNew[2] & 0xFF;
			RegNew[7] = (GainNew[2] & 0xF00) >> 8;
			gain_first++;
		}

		for(i = 0; i < SH5001_CALI_REG_NUM; i++)
		{
			SH5001_write(SH5001_ADDRESS, reg_addr_cali[i], 1, &RegNew[i]);
		}

		//delay_ms(30); // or discard 2
		return (SH5001_TRUE);

	case SH5001_POWERDOWN_MODE:

		for(i = 0; i < SH5001_POWMODE_REG_NUM_O2; i++)
		{
			SH5001_write(SH5001_ADDRESS, reg_addr[i], 1, &reg_data_powerdown[i]);
		}

		return (SH5001_TRUE);

	default:
		return (SH5001_FALSE);
	}
}

static unsigned char O1_switchpower = 1;
unsigned char SH5001_SwitchPowerMode(unsigned char powerMode)
{
	if(O1_switchpower){
		SH5001_O1_SwitchPowerMode(powerMode);
	} else {
		SH5001_O2_SwitchPowerMode(powerMode);
	}
}

/******************************************************************
* Description:	1. fast compensate offset
*
* Parameters:   SH5001Positon
*               SH5001_X_UP
*               SH5001_X_DOWN
*				SH5001_Y_UP
*               SH5001_Y_DOWN
*				SH5001_Z_UP
*               SH5001_Z_DOWN
*
* return:	SH5001_FALSE or SH5001_TRUE
*
******************************************************************/
void SH5001_FastOffsetComp(unsigned char SH5001Positon)
{
  unsigned char regData = 0;
    
  switch(SH5001Positon)
    {
  case SH5001_X_UP:
    regData = 0x90U;
    break;
  case SH5001_X_DOWN:
    regData = 0x91U;
    break;  
  case SH5001_Y_UP:
    regData = 0x92U;
    break;
  case SH5001_Y_DOWN:
    regData = 0x93U;
    break;     
  case SH5001_Z_UP:
    regData = 0x94U;
    break;
  case SH5001_Z_DOWN:
    regData = 0x95U;
    break; 
  default:
    regData = 0x94U;
    break;
  }
  
//  SH5001_write(SH5001_ADDRESS, SH5001_FAST_COMP, 1, &regData);
}  


#ifdef SH5001_ACTIVITY_INT

void SH5001_Init_Activity_Int(void)
{

	unsigned char regData = 0;

	SH5001_INT_Config(SH5001_INT0_LEVEL_HIGH,
                      SH5001_INT_LATCH, //SH5001_INT_NO_LATCH, SH5001_INT_LATCH
                      SH5001_INT_CLEAR_STATUS,  //SH5001_INT_CLEAR_ANY, SH5001_INT_CLEAR_STATUS
                      1,
                      SH5001_INT1_NORMAL,
                      SH5001_INT0_NORMAL,
                      SH5001_INT1_OUTPUT,
                      SH5001_INT0_OUTPUT);

	SH5001_INT_Act_Config(SH5001_ACT_X_INT_EN,
	SH5001_ACT_Y_INT_EN,
	SH5001_ACT_Z_INT_EN,
	2,
	20,
	SH5001_LINK_PRE_STA_NO,
	SH5001_ACT_INACT_CLR_STATUS1);

	SH5001_INT_Enable(SH5001_INT_ACT,
                      SH5001_INT_EN,
                      SH5001_INT_MAP_INT0);
	//TODO
	SH5001_read(SH5001_ADDRESS, SH5001_ACC_CONF2, 1, &regData);
	regData = regData & 0x7F;
	SH5001_write(SH5001_ADDRESS, SH5001_ACC_CONF2, 1, &regData);

	SH5001_read(SH5001_ADDRESS, 0xBD, 1, &regData);
	regData = (regData & 0x7F);
	SH5001_write(SH5001_ADDRESS, 0xBD, 1, &regData);
}

#endif

#ifdef SH5001_DATA_READY_INT
void SH5001_Init_Data_Ready_Int(void)
{

	unsigned char regData = 0;

	SH5001_INT_Config(SH5001_INT0_LEVEL_HIGH,
                      SH5001_INT_LATCH, //SH5001_INT_NO_LATCH, SH5001_INT_LATCH
                      SH5001_INT_CLEAR_STATUS,  //SH5001_INT_CLEAR_ANY, SH5001_INT_CLEAR_STATUS
                      1,
                      SH5001_INT1_NORMAL,
                      SH5001_INT0_NORMAL,
                      SH5001_INT1_OUTPUT,
                      SH5001_INT0_OUTPUT);

	SH5001_INT_Enable(SH5001_INT_ACC_READY, //SH5001_INT_GYRO_READY SH5001_INT_ACC_READY
                      SH5001_INT_EN,
                      SH5001_INT_MAP_INT0);

}

#endif

/******************************************************************
* Description: 	1. SH5001 initialization FIFO
*               2. INT mode
*               3. Enable ACC_X, ACC_Y, ACC_Z, GYRO_X, GYRO_Y, GYRO_Z
*
* Parameters:	void
*
* return:	void
*
******************************************************************/
void SH5001_InitFIFO(void)
{  

	SH5001_INT_Config(SH5001_INT0_LEVEL_HIGH,
                      SH5001_INT_LATCH, //SH5001_INT_NO_LATCH, SH5001_INT_LATCH
                      SH5001_INT_CLEAR_STATUS,  //SH5001_INT_CLEAR_ANY, SH5001_INT_CLEAR_STATUS
                      1,
                      SH5001_INT1_NORMAL,
                      SH5001_INT0_NORMAL,
                      SH5001_INT1_OUTPUT,
                      SH5001_INT0_OUTPUT);

	SH5001_FIFO_Reset();

	SH5001_FIFO_Freq_Config(SH5001_FIFO_ACC_DOWNS_DIS,
                            SH5001_FIFO_FREQ_X1_32,
                            SH5001_FIFO_GYRO_DOWNS_DIS,
                            SH5001_FIFO_FREQ_X1_32);

    SH5001_FIFO_Data_Config(SH5001_FIFO_ACC_X_EN
                            | SH5001_FIFO_ACC_Y_EN
                            | SH5001_FIFO_ACC_Z_EN
                            | SH5001_FIFO_GYRO_X_EN
                            | SH5001_FIFO_GYRO_Y_EN
                            | SH5001_FIFO_GYRO_Z_EN,
                            120);

    SH5001_FIFO_Mode_Set(SH5001_FIFO_MODE_FIFO);

	SH5001_INT_Enable(SH5001_INT_FIFO_WATERMARK,
                      SH5001_INT_EN, 
                      SH5001_INT_MAP_INT0);

}



/******************************************************************
* Description: 	1. SH5001 read ACC_X, ACC_Y, ACC_Z, GYRO_X, GYRO_Y, GYRO_Z @FIFO
*               2. clear FIFO Watermark INT
*
* Parameters:	void
*
* return:	void
*
******************************************************************/
void SH5001_ReadFIFO(void)
{
	unsigned short fifoEntriesCount = 0;
	unsigned char fifoData[SH5001_FIFO_BUFFER] = {0};
	unsigned short i = 0, j = 0, dataGroup = 0;
	short accData[3], gyroData[3];
	unsigned char regData = 0;

	// read FIFO entries count
	SH5001_FIFO_Read_Status(&fifoEntriesCount);

	dataGroup = (unsigned short)(fifoEntriesCount/SH5001_WATERMARK_DIV);
	fifoEntriesCount = dataGroup * SH5001_WATERMARK_DIV;

	SH5001_FIFO_Read_Data(fifoData, fifoEntriesCount);

	printf("fc %d\r\n", fifoEntriesCount);

	for(i = 0; i < dataGroup; i++) 
	{				
		accData[0] = ((short)fifoData[j+1] << 8) | fifoData[j];
		accData[1] = ((short)fifoData[j+3] << 8) | fifoData[j+2];
		accData[2] = ((short)fifoData[j+5] << 8) | fifoData[j+4];	
		gyroData[0] = ((short)fifoData[j+7] << 8) | fifoData[j+6];
		gyroData[1] = ((short)fifoData[j+9] << 8) | fifoData[j+8];
		gyroData[2] = ((short)fifoData[j+11] << 8) | fifoData[j+10];	

		printf("%d %d %d %d %d %d\r\n", accData[0], accData[1], accData[2],gyroData[0], gyroData[1], gyroData[2]);
       
		j = j + SH5001_WATERMARK_DIV;
	}	
    
        
    // clear INT    
   // SH5001_INT_Read_Status01();


}  

 #define SH5001_REGISTER_SETTING_TOTAL 18
static int sh5001DumpReg(void)
{
	int i = 0;
	int ret = 0;
	uint8_t reg = 0;
	uint8_t regData = 0;

	//uint8_t SH5001Reg[SH5001_REGISTER_SETTING_TOTAL]	=	{0x20, 0x21,0x22, 0x23, 0x24, 0x25};

	//uint8_t SH5001Reg[SH5001_REGISTER_SETTING_TOTAL]	=	{0x20, 0x21,0x22, 0x23, 0x24, 0x25, 0x40, 0x41, 0x42, 0x43, 0x66}; //FIFO wm int
	uint8_t SH5001Reg[SH5001_REGISTER_SETTING_TOTAL]	=	{0x20, 0x21,0x22, 0x23, 0x24, 0x25,0x40, 0x41, 0x42, 0x43, 0x66, 0x4D, 0x4E,  0x54,0x55, 0x56, 0xBD};  //Activity int

	//uint8_t SH3001Reg[SH3001_REGISTER_SETTING_TOTAL]	=	{0x28, 0x29, 0x2B, 0x23, 0x26, 0xdd}; //lpf setting
	// uint8_t SH3001Reg[SH3001_REGISTER_SETTING_TOTAL]	=	{0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29,0x2A, 0x2B};
	//uint8_t SH3001Reg[SH3001_REGISTER_SETTING_TOTAL]	=	{0x22, 0x23, 0x24, 0x25, 0x26, 0x35,0x36,0x37, 0x38,0x39};
	//uint8_t ST3001RegData[SH3001_REGISTER_SETTING_TOTAL] = {0x00,0x1F,0x00,0x2B,0x00};
	//uint8_t SH3001INTFIFOReg[SH3001_REGISTER_SETTING_TOTAL]	=	{0x35, 0x36, 0x37, 0x38, 0x39, 0x40,0x41,0x44, 0x45,0x46, 0x47, 0x4d,0x79,0x7A,0x0F,0x8F,0x9F,0xAF};
	//uint8_t SH3001Reg[SH3001_REGISTER_SETTING_TOTAL]	=	{0x35, 0x36, 0x37, 0x38, 0x39, 0x40,0x41,0x42, 0x43,0x44, 0x45,0x46, 0x47, 0xFE};
	//uint8_t SH3001Reg[SH3001_REGISTER_SETTING_TOTAL]	=	{0x20, 0x21, 0x0c, 0x0d};

	for( i = 0 ; i < SH5001_REGISTER_SETTING_TOTAL ; i++ )
	{

		//read
		reg = SH5001Reg[i];

		ret = SH5001_read(SH5001_ADDRESS, reg, 1, &regData);
		if(ret != 0)
		{
			printf("Senodia Dump Reg 0x%x read error\r\n", reg);
			return -1;
		}

		printf("Senodia DR >>>>> 0x%x = 0x%x\r\n", reg, regData);

	}


	return ret;

}

static void SH5001_IMU_Data_Stablize(void)
{
	unsigned char regData = 0;

	//TODO Three times maybe
	//acc
	SH5001_read(SH5001_ADDRESS, SH5001_ACC_CONF0, 1, &regData);
	regData &= 0xFE;
	SH5001_write(SH5001_ADDRESS, SH5001_ACC_CONF0, 1, &regData);
	regData |= 0x1;
	SH5001_write(SH5001_ADDRESS, SH5001_ACC_CONF0, 1, &regData);


	//gyro
	SH5001_read(SH5001_ADDRESS, SH5001_GYRO_CONF0, 1, &regData);
	regData &= 0xFE;
	SH5001_write(SH5001_ADDRESS, SH5001_GYRO_CONF0, 1, &regData);
	regData |= 0x1;
	SH5001_write(SH5001_ADDRESS, SH5001_GYRO_CONF0, 1, &regData);
}

static void SH5001_OSC_FREQ(void)
{
	unsigned char regData = 0;

	SH5001_read(SH5001_ADDRESS, 0xDA, 1, &regData);
	if((regData & 0x07) != 0x07)
	{
		O1_switchpower = 1;
		regData |= 0x07;
		SH5001_write(SH5001_ADDRESS, 0xDA, 1, &regData);
	} else if((regData & 0x07) == 0x07){
		O1_switchpower = 0;
	}
	printf("Chip is %s\r\n", O1_switchpower?"O1":"O2");
}

#if SENODIA_VDD_3V3
short v3_acc[3] = {0};
unsigned char use_otp_comp = 0;

void SH5001_Get_Otpoffset(short vacc[3])
{
	unsigned char regData[3]={0};
	unsigned char data = 0;

	//entry otp read mode
	data = 0x72;
	SH5001_write(SH5001_ADDRESS, 0x02, 1, &data);

	//read otp page
	data = 0;
	SH5001_read(SH5001_ADDRESS, 0x0, 1, &data);
	printf("otp page data:0x%x\n", data);
	if(data == 0x70){
		//exit otp mode
		data = 0x71;
		SH5001_write(SH5001_ADDRESS, 0x03, 1, &data);

		//entry otp page 2
		data = 0x70;
		SH5001_write(SH5001_ADDRESS, 0x05, 1, &data);

		//entry otp read mode
		data = 0x72;
		SH5001_write(SH5001_ADDRESS, 0x02, 1, &data);

	}
	//otp read data
	SH5001_read(SH5001_ADDRESS, 0x08, 3, regData);

	//exit otp mode
	data = 0x71;
	SH5001_write(SH5001_ADDRESS, 0x03, 1, &data);

	if(regData[0] == 0x5f && regData[1] == 0x0 && regData[2] == 0x48)
		use_otp_comp = 0;
	else
		use_otp_comp = 1;
	printf("use_otp_comp:%d\n", use_otp_comp);
	vacc[0] = ((signed char)regData[0]) * 8;
	vacc[1] = ((signed char)regData[1]) * 8;
	vacc[2] = ((signed char)regData[2]) * 8;
	printf("%d %d %d %d %d %d \r\n", regData[0], regData[1], regData[2], vacc[0], vacc[1], vacc[2]);
}
#endif

void SH5001_Adjust_Cf(void)
{
	unsigned char reg_addr[15] = { 0x8C, 0x8D, 0x8E, 0x8F, 0x98, 0x99, 0x9A, 0x9B, 0xA4, 0xA5, 0xA6, 0xA7, 0xC4, 0xC5, 0xC6};
	unsigned char regDataold[15];
	unsigned char regDatanew[15];
	short OldData[6];
	short NewData[6];
	int i = 0;

	SH5001_read(SH5001_ADDRESS, reg_addr[12], 1, &regDataold[12]);
	//printf("Adjust read %x= %x\r\n", reg_addr[12], regDataold[12]);
	if(regDataold[12] != 0x06)
		return;

	for(i = 0; i < 15; i++)
	{
		SH5001_read(SH5001_ADDRESS, reg_addr[i], 1, &regDataold[i]);
		//printf("Adjust read = %x\r\n", regDataold[i]);
	}

	OldData[0] = (short)(regDataold[1] << 8 | regDataold[0]);
	OldData[1] = (short)(regDataold[3] << 8 | regDataold[2]);
	OldData[2] = (short)(regDataold[5] << 8 | regDataold[4]);
	OldData[3] = (short)(regDataold[7] << 8 | regDataold[6]);
	OldData[4] = (short)(regDataold[9] << 8 | regDataold[8]);
	OldData[5] = (short)(regDataold[11] << 8 | regDataold[10]);

	NewData[0] = OldData[0] * 1.693;
	NewData[1] = OldData[1] * 0.588 - 91;
	NewData[2] = OldData[2] * 1.693;
	NewData[3] = OldData[3] * 0.585 - 313;
	NewData[4] = OldData[4] * 1.679;
	NewData[5] = OldData[5] * 0.590 - 143;

	//printf("OldData %d %d %d %d %d %d \r\n",OldData[0], OldData[1], OldData[2], OldData[3], OldData[4], OldData[5]);
	//printf("NewData %d %d %d %d %d %d \r\n",NewData[0], NewData[1], NewData[2], NewData[3], NewData[4], NewData[5]);

	regDatanew[0] = NewData[0] & 0xFF;
	regDatanew[1] = (NewData[0] >> 8) & 0xFF;
	regDatanew[2] = NewData[1] & 0xFF;
	regDatanew[3] = (NewData[1] >> 8) & 0xFF;
	regDatanew[4] = NewData[2] & 0xFF;
	regDatanew[5] = (NewData[2] >> 8) & 0xFF;
	regDatanew[6] = NewData[3] & 0xFF;
	regDatanew[7] = (NewData[3] >> 8) & 0xFF;
	regDatanew[8] = NewData[4] & 0xFF;
	regDatanew[9] = (NewData[4] >> 8) & 0xFF;
	regDatanew[10] = NewData[5] & 0xFF;
	regDatanew[11] = (NewData[5] >> 8) & 0xFF;
	regDatanew[12] = 0x04;
	regDatanew[13] = 0x04;
	regDatanew[14] = 0x04;

	for(i = 0; i < 15; i++)
	{
		//printf("Adjust write = %x\r\n", regDatanew[i]);
		SH5001_write(SH5001_ADDRESS, reg_addr[i], 1, &regDatanew[i]);
	}
}

void LinearFit(uint8_t *X, short *Y, double *a, double *b)
{
	int N = 5;

	double A = 0;
	double B = 0;
	double C = 0;
	double D = 0;

	for (int i = 0; i < N; i++)
	{
		A += (double)X[i] * X[i];
		B += (double)X[i];
		C += (double)Y[i] * X[i];
		D += (double)Y[i];
		printf("Linear %d %d\r\n", X[i], Y[i]);
	}

	if ((A * N - B * B) != 0)
	{
		*a = (C * N - B * D) / (A * N - B * B);
		*b = (A * D - C * B) / (A * N - B * B);
	}
	//printf("LinearFit %f %f %f %f\r\n", (C * N - B * D) / (A * N - B * B), (A * D - C * B) / (A * N - B * B), *a, *b);

}

int Give_Sign(int Input, int N)
{
	int Output = (Input < (1 << N - 1)) ? Input : Input - (1 << N);
	return Output;
}

void SH5001_Accode_Cali(uint8_t result[5])
{
	unsigned char regData = 0;
	int data14 = 0, data15 = 0;
	float Slope = 4.3;
	int i = 0, k = 0;
	int sum = 0;
	uint8_t AC_Zero;
	int AC_Code, DC_Code;
	uint8_t reg[5] = {0x2C, 0xB0, 0xB1, 0xB2, 0xAF};
	uint8_t regDataR[5];
	uint8_t regDataW[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
	int gyro[3] = {0};
	uint8_t rawdata[6];
	short GX[5], GY[5], GZ[5];
	uint8_t AC[5];
	short Gyro_Gain_X, Gyro_Gain_Y, Gyro_Gain_Z;
	int Gyro_Jump_X, Gyro_Jump_Y, Gyro_Jump_Z;
	double Gyro_Step_X, Gyro_Step_Y, Gyro_Step_Z;
	double a = 0, b = 0;

	for(i = 0; i < 5; i++)
	{
		SH5001_read(SH5001_ADDRESS, reg[i], 1, &regDataR[i]);
		printf("regDataR %x %x\r\n", reg[i], regDataR[i]);
	}

	for(i = 0; i < 5; i++)
	{
		SH5001_write(SH5001_ADDRESS, reg[i], 1, &regDataW[i]);
		printf("regDataW %x %x\r\n", reg[i], regDataW[i]);
	}

	for(i = 0; i < 20; i++)
	{
		SH5001_read(SH5001_ADDRESS, 0x14, 1, &regData);
		sum += (regData & 0x1F);
		delay_ms(10);
	}
	AC_Code = sum / 20;
	printf("AC_Code %d\r\n", AC_Code);

	sum = 0;
	for(i = 0; i < 20; i++)
	{
		SH5001_read(SH5001_ADDRESS, 0x15, 1, &regData);
		regData = (regData & 0x1F);
		if(regData >= 16)
			data15 = (int)(regData - 32);
		else
			data15 = (int)(regData);

		sum += data15;
		delay_ms(10);
	}

	DC_Code = sum / 20;
	AC_Zero = (int)(AC_Code - ((float)DC_Code / Slope) + 0.5);
	if (AC_Zero < 0)
	{
		AC_Zero = 0;
	}
	if (AC_Zero > 31)
	{
		AC_Zero = 31;
	}
	printf("DC_Code %d %d %f %d\r\n", DC_Code, AC_Zero, ((float)DC_Code / Slope), (int)(AC_Code - ((float)DC_Code / Slope)));

	regData = (regDataR[0] & 0xE0) | ((uint8_t)AC_Zero);
	printf("0x2C0 regData %x\r\n", regData);
	SH5001_write(SH5001_ADDRESS, 0x2C, 1, &regData);

	regData |= 0x80;
	SH5001_write(SH5001_ADDRESS, 0x2C, 1, &regData);
	printf("0x2C1 regData %x\r\n", regData);

	for(i = 0; i < 5; i++)
	{
		AC_Code = AC_Zero + i - 2;
		if (AC_Code < 0)
		{
			AC_Code = 0;
		}
		if (AC_Code > 31)
		{
			AC_Code = 31;
		}

		SH5001_read(SH5001_ADDRESS, 0x2C, 1, &regData);
		regData = (regData & 0xE0) | (((uint8_t)AC_Code) & 0x1F);
		printf("0x2C2 write regData %x\r\n", regData);
		SH5001_write(SH5001_ADDRESS, 0x2C, 1, &regData);

		SH5001_read(SH5001_ADDRESS, 0x2C, 1, &regData);
		printf("0x2C2 read regData %x\r\n", regData);

		delay_ms(100);

		for(k = 0; k < 20; k++)
		{
			SH5001_read(SH5001_ADDRESS, 0x06, 6, rawdata);
			gyro[0] += (short)(rawdata[1] << 8 | rawdata[0]);
			gyro[1] += (short)(rawdata[3] << 8 | rawdata[2]);
			gyro[2] += (short)(rawdata[5] << 8 | rawdata[4]);
			delay_ms(10);
			printf("gyro[0] %d %d %d\r\n", gyro[0], gyro[1], gyro[2]);
		}
		GX[i] = gyro[0] / 20;
		GY[i] = gyro[1] / 20;
		GZ[i] = gyro[2] / 20;

		sum = 0;
		gyro[0] = 0;
		gyro[1] = 0;
		gyro[2] = 0;

		for(k = 0; k < 20; k++)
		{
			SH5001_read(SH5001_ADDRESS, 0x14, 1, &regData);
			sum += (regData & 0x1F);
			delay_ms(10);
		}
		AC[i] = sum / 20;
		printf("GX %d %d %d %d\r\n", GX[i], GY[i], GZ[i], AC[i]);
	}

	SH5001_read(SH5001_ADDRESS, 0x8C, 2, rawdata);
	Gyro_Gain_X = (short)(rawdata[1] << 8 | rawdata[0]);
	Gyro_Gain_X = Give_Sign(Gyro_Gain_X, 16);

	SH5001_read(SH5001_ADDRESS, 0x98, 2, rawdata);
	Gyro_Gain_Y = (short)(rawdata[1] << 8 | rawdata[0]);
	Gyro_Gain_Y = Give_Sign(Gyro_Gain_Y, 16);

	SH5001_read(SH5001_ADDRESS, 0xA4, 2, rawdata);
	Gyro_Gain_Z = (short)(rawdata[1] << 8 | rawdata[0]);
	Gyro_Gain_Z = Give_Sign(Gyro_Gain_Z, 16);

	printf("Gyro_Gain_X %d %d %d\r\n", Gyro_Gain_X, Gyro_Gain_Y, Gyro_Gain_Z);

	LinearFit(AC, GX, &Gyro_Step_X, &b);
	LinearFit(AC, GY, &Gyro_Step_Y, &b);
	LinearFit(AC, GZ, &Gyro_Step_Z, &b);
	printf("Gyro_Step_X %d %d %d\r\n", AC[0], GX[0], GX[1]);
	printf("Gyro_Step_X %f %f %f\r\n", Gyro_Step_X, Gyro_Step_Y, Gyro_Step_Z);

	Gyro_Jump_X = (int)(Gyro_Step_X * 2048 * 4 / Gyro_Gain_X + 0.5);
	Gyro_Jump_Y = (int)(Gyro_Step_Y * 2048 * 4 / Gyro_Gain_Y + 0.5);
	Gyro_Jump_Z = (int)(Gyro_Step_Z * 2048 * 4 / Gyro_Gain_Z + 0.5);

	Gyro_Jump_X = Gyro_Jump_X >= 0 ? Gyro_Jump_X : Gyro_Jump_X + 128;
	Gyro_Jump_Y = Gyro_Jump_Y >= 0 ? Gyro_Jump_Y : Gyro_Jump_Y + 128;
	Gyro_Jump_Z = Gyro_Jump_Z >= 0 ? Gyro_Jump_Z : Gyro_Jump_Z + 128;

	printf("Gyro_Jump_X %d %d %d\r\n", Gyro_Jump_X, Gyro_Jump_Y, Gyro_Jump_Z);

	regData = AC_Zero;
	SH5001_write(SH5001_ADDRESS, 0x2C, 1, &regData);
	printf("Gyro_0x2C1 %x\r\n", regData);
	result[0] = regData;

	SH5001_write(SH5001_ADDRESS, 0xAF, 1, &regData);
	printf("Gyro_0xAF %x\r\n", regData);
	result[4] = regData;

	regDataW[1] = (Gyro_Jump_X & 0x7F);
	regDataW[2] = (Gyro_Jump_Y & 0x7F);
	regDataW[3] = (Gyro_Jump_Z & 0x7F);

	for(i = 1; i < 4; i++)
	{
		SH5001_write(SH5001_ADDRESS, reg[i], 1, &regDataW[i]);
		result[i] = regDataW[i];
		printf("Gyro %x %x\r\n", reg[i], regDataW[i]);
	}

}

void SH5001_Set_AC_Zero(void)
{
	unsigned char regData = 0;
	int data14 = 0, data15 = 0;
	float Slope = 4.3;
	int i = 0;
	int sum = 0;
	uint8_t AC_Zero;
	int AC_Code, DC_Code;
	short GX[5], GY[5], GZ[5];
	uint8_t AC[5];
	short Gyro_Gain_X, Gyro_Gain_Y, Gyro_Gain_Z;
	int Gyro_Jump_X, Gyro_Jump_Y, Gyro_Jump_Z;
	float Gyro_Step_X, Gyro_Step_Y, Gyro_Step_Z;
	double a = 0, b = 0;

	for(i = 0; i < 20; i++)
	{
		SH5001_read(SH5001_ADDRESS, 0x14, 1, &regData);
		sum += (regData & 0x1F);
		delay_ms(10);
	}
	AC_Code = sum / 20;
	printf("AC_Code %d\r\n", AC_Code);

	sum = 0;
	for(i = 0; i < 20; i++)
	{
		SH5001_read(SH5001_ADDRESS, 0x15, 1, &regData);
		regData = (regData & 0x1F);
		if(regData >= 16)
			data15 = (int)(regData - 32);
		else
			data15 = (int)(regData);

		sum += data15;
		delay_ms(10);
	}

	DC_Code = sum / 20;
	AC_Zero = (int)(AC_Code - ((float)DC_Code / Slope) + 0.5);
	if (AC_Zero < 0)
	{
		AC_Zero = 0;
	}
	if (AC_Zero > 31)
	{
		AC_Zero = 31;
	}
	printf("DC_Code %d %d %f %d\r\n", DC_Code, AC_Zero, ((float)DC_Code / Slope), (int)(AC_Code - ((float)DC_Code / Slope)));

	SH5001_read(SH5001_ADDRESS, 0x2C, 1, &regData);
	regData = (regData & 0xE0) | AC_Zero;
	printf("0x2C0 regData %x\r\n", regData);
	SH5001_write(SH5001_ADDRESS, 0x2C, 1, &regData);

	regData |= 0x80;
	SH5001_write(SH5001_ADDRESS, 0x2C, 1, &regData);
	printf("0x2C1 regData %x\r\n", regData);

	delay_ms(50);
	regData &= 0x7F;
	SH5001_write(SH5001_ADDRESS, 0x2C, 1, &regData);
	printf("0x2C2 regData %x\r\n", regData);

}

/******************************************************************
* Description: 	SH5001 initialization function
*
* Parameters:	void
*																						
* return:	SH5001_TRUE or SH5001_FALSE
*
******************************************************************/
unsigned char SH5001_init(void) 
{
	unsigned char regData = 0;
	unsigned char i = 0;

#if SH5001_SPI_3WIRE_MODE
	SH5001_SPI_Config(SH5001_SPI_3_WIRE);
#endif

	// SH5001 chipID = 0xA1;	
	while((regData != 0xA1U) && (i++ < 3U))
	{
		SH5001_read(SH5001_ADDRESS, SH5001_CHIP_ID, 1, &regData);	
		if((i == 3) && (regData != 0xA1U))
		{
			printf("SH5001 init fail, Chip ID = 0x%x\r\n", regData);
			return SH5001_FALSE;
		}	
	}
	printf("SH5001 Chip ID = 0x%x\r\n", regData);
		
	// reset internal module, don't modify it
	SH5001_ModuleReset();	

	SH5001_OSC_FREQ();

	#if SH5001_OIS_ENABLE
	SH5001_OIS_SPI_Config(SH5001_OIS_SPI_EN);
	#endif

	if(O1_switchpower){
		// 500Hz, 16G, cut off Freq(BW)=500*0.25Hz=125Hz, enable filter;
		SH5001_Acc_Config(SH5001_ACC_ODR_125HZ, 
						SH5001_ACC_RANGE_16G, //SH5001_ACC_RANGE_8G  SH5001_ACC_RANGE_16G
						SH5001_ACC_ODRX040,
						SH5001_ACC_FILTER_EN,
						SH5001_ACC_BYPASS_EN);

		printf("SH5001 O1 acc config ok\r\n");

		// 800Hz, X\Y\Z 2000deg/s, cut off Freq(BW)=291Hz, enable filter;
		SH5001_Gyro_Config(	SH5001_GYRO_ODR_125HZ,
						SH5001_GYRO_RANGE_2000,
						SH5001_GYRO_ODRX040,
						SH5001_GYRO_FILTER_EN,
						SH5001_GYRO_BYPASS_EN);
	} else {
		// 500Hz, 16G, cut off Freq(BW)=500*0.25Hz=125Hz, enable filter;
		SH5001_Acc_Config(SH5001_ACC_ODR_110HZ, 
						SH5001_ACC_RANGE_16G, //SH5001_ACC_RANGE_8G  SH5001_ACC_RANGE_16G
						SH5001_ACC_ODRX040,
						SH5001_ACC_FILTER_EN,
						SH5001_ACC_BYPASS_EN);

		printf("SH5001 O2 acc config ok\r\n");

		// 800Hz, X\Y\Z 2000deg/s, cut off Freq(BW)=291Hz, enable filter;
		SH5001_Gyro_Config(SH5001_GYRO_ODR_110HZ,
						SH5001_GYRO_RANGE_2000,
						SH5001_GYRO_ODRX040,
						SH5001_GYRO_FILTER_EN,
						SH5001_GYRO_BYPASS_EN);
	}

	SH5001_IMU_Data_Stablize();

	#if SH5001_OIS_ENABLE
	SH5001_Acc_OIS_Config(SH5001_ACC_OIS_ODR_1000HZ,
		SH5001_ACC_OIS_RANGE_8G,
		SH5001_ACC_OIS_ODRX004,
		SH5001_ACC_OIS_ENABLE);

	SH5001_Gyro_OIS_Config(SH5001_GYRO_OIS_ODR_8000HZ,
		SH5001_GYRO_OIS_RANGE_2000,
		SH5001_GYRO_OIS_ODRX004,
		SH5001_GYRO_OIS_ENABLE);
	#endif

	#if SH5001_TS_ENABLE
	SH5001_TimeStamp_Config(SH5001_TS_ODR_1KHZ, SH5001_TS_EN);
	#endif

	// temperature ODR is 63Hz, enable temperature measurement
	SH5001_Temp_Config(SH5001_TEMP_ODR_63HZ, SH5001_TEMP_EN);

	//sh5001DumpReg();
	#ifdef SH5001_FIFO_WM_INT
	SH5001_InitFIFO();
	#endif

	#ifdef SH5001_ACTIVITY_INT
	SH5001_Init_Activity_Int();
	#endif

	#ifdef SH5001_DATA_READY_INT
	SH5001_Init_Data_Ready_Int();
	#endif

#if SENODIA_VDD_3V3
	SH5001_Get_Otpoffset(v3_acc);
#endif

	SH5001_Adjust_Cf();
	//sh5001DumpReg();
	//sh5001DumpRegAll();

	return SH5001_TRUE;
}



/******************************************************************
* Description: 	Read SH5001 gyroscope and accelerometer data
*
* Parameters:	accData[3]: acc X,Y,Z;  gyroData[3]: gyro X,Y,Z;
*																						
* return:	void
* 
******************************************************************/
void SH5001_GetImuData( short accData[3], short gyroData[3] )
{
	unsigned char regData[12]={0};	
		
	SH5001_read(SH5001_ADDRESS, SH5001_ACC_XL, 12, regData); 
  
	accData[0] = ((short)regData[1] << 8) | regData[0];
	accData[1] = ((short)regData[3] << 8) | regData[2];
	accData[2] = ((short)regData[5] << 8) | regData[4];		
	
	gyroData[0] = ((short)regData[7] << 8) | regData[6];
	gyroData[1] = ((short)regData[9] << 8) | regData[8];
	gyroData[2] = ((short)regData[11] << 8) | regData[10];

#if SENODIA_VDD_3V3
	if(use_otp_comp){
		accData[0] -= v3_acc[0];
		accData[1] -= v3_acc[1];
		accData[2] -= v3_acc[2];
	}
#endif

	printf("%d %d %d %d %d %d\r\n", accData[0], accData[1], accData[2], gyroData[0], gyroData[1], gyroData[2]);
}

void SH5001_OIS_GetTS(int timestamp[1])
{
	unsigned char regData[3]={0};

	SH5001_read(SH5001_ADDRESS, 0xE, 3, regData);

	timestamp[0] = ((int)regData[2] << 16) | ((int)regData[1] << 8) | regData[0];

}

#if SH5001_OIS_ENABLE
void SH5001_OIS_GetImuData( short accData[3], short gyroData[3] )
{
	unsigned char regData[12]={0};
	int i = 0;

	SH5001_OIS_read(SH5001_ADDRESS, SH5001_ACC_XL, 12, regData);

	accData[0] = ((short)regData[1] << 8) | regData[0];
	accData[1] = ((short)regData[3] << 8) | regData[2];
	accData[2] = ((short)regData[5] << 8) | regData[4];

	gyroData[0] = ((short)regData[7] << 8) | regData[6];
	gyroData[1] = ((short)regData[9] << 8) | regData[8];
	gyroData[2] = ((short)regData[11] << 8) | regData[10];

	//printf("%d %d %d %d %d %d\r\n", accData[0], accData[1], accData[2], gyroData[0], gyroData[1], gyroData[2]);
}
#endif

#define SH5001_OFFSET_REG_NUM 13
void offset_comp_save_reg(float offset[3])
{
	unsigned char offsetData[SH5001_OFFSET_REG_NUM] = {0};

	short offset_otp[3] = {0};
	short gain_otp[3] = {0};
	unsigned int range = 1;

	int offset_temp[3] = {0};
	short offset_new[3] = {0};
	unsigned short s_offset_new[3] = {0};

	unsigned char i;


	unsigned char offsetRegister[SH5001_OFFSET_REG_NUM] = { 0x8C, 0x8D, 0x8E, 0x8F, 0x98, 0x99, 0x9A, 0x9B, 0xA4, 0xA5 , 0xA6,
	0xA7, 0x24};

	for(i=0; i<SH5001_OFFSET_REG_NUM; i++)
	{
		SH5001_read(SH5001_ADDRESS, offsetRegister[i], 1, &offsetData[i]);
		#ifdef SH5001_OT_DEBUG
		printf("0x%x = 0x%x\r\n", offsetRegister[i], offsetData[i]);
		#endif
	}
	#ifdef SH5001_OT_DEBUG
	printf("n offset %f %f %f \r\n", offset[0], offset[1],offset[2]);
	#endif

	offset_otp[0] = ((short)offsetData[3] << 8) | offsetData[2];
	offset_otp[1] = ((short)offsetData[7] << 8) | offsetData[6];
	offset_otp[2] = ((short)offsetData[11] << 8) | offsetData[10];

	gain_otp[0] = ((short)offsetData[1] << 8) | offsetData[0];
	gain_otp[1] = ((short)offsetData[5] << 8) | offsetData[4];
	gain_otp[2] = ((short)offsetData[9] << 8) | offsetData[8];

	#ifdef SH5001_OT_DEBUG
	printf("offset_otp %d %d %d \r\n", offset_otp[0], offset_otp[1],offset_otp[2]);
	printf("gain_otp %d %d %d \r\n", gain_otp[0], gain_otp[1],gain_otp[2]);
	#endif

	offsetData[12] = (offsetData[12] & 0x70) >> 4;

	if(offsetData[12] != 0)
	{
			for(i=0; i < offsetData[12] + 7; i++)
			{
				range = range * 2;
			}
	}

	#ifdef SH5001_OT_DEBUG
	printf("range %d \r\n", range);
	#endif

	for(i = 0; i < 3; i++)
	{
		offset_temp[i] = offset_otp[i] + (int)(offset[i] * ((float)range) / ((float)gain_otp[i]));  //
		//offset_new[i] = offset_otp[i] + (offset[i] * range / gain_otp[i]);  //
	}

	if(offset_temp[0] > 32767)
		offset_temp[0] = 32767;
	else if(offset_temp[0] < -32768)
		offset_temp[0] = -32768;
	if(offset_temp[1] > 32767)
		offset_temp[1] = 32767;
	else if(offset_temp[1] < -32768)
		offset_temp[1] = -32768;
	if(offset_temp[2] > 32767)
		offset_temp[2] = 32767;
	else if(offset_temp[2] < -32768)
		offset_temp[2] = -32768;

	offset_new[0] = (short)offset_temp[0];
	offset_new[1] = (short)offset_temp[1];
	offset_new[2] = (short)offset_temp[2];

	#ifdef SH5001_OT_DEBUG
	printf("offset_new %d %d %d \r\n", offset_new[0], offset_new[1],offset_new[2]);
	#endif

	s_offset_new[0] = (unsigned short)offset_new[0];
	s_offset_new[1] = (unsigned short)offset_new[1];
	s_offset_new[2] = (unsigned short)offset_new[2];

	offsetData[3] = (unsigned char)((s_offset_new[0] & 0xFF00)>>8);
	offsetData[7] = (unsigned char)((s_offset_new[1] & 0xFF00)>>8);
	offsetData[11] = (unsigned char)((offset_new[2] & 0xFF00)>>8);
	offsetData[2] = (unsigned char)((s_offset_new[0] & 0x00FF));
	offsetData[6] = (unsigned char)((s_offset_new[1] & 0x00FF));
	offsetData[10] = (unsigned char)((s_offset_new[2] & 0x00FF));

	#ifdef SH5001_OT_DEBUG
	printf("offsetData %x %x %x %x %x %x\r\n", offsetData[2], offsetData[3],offsetData[6], offsetData[7], offsetData[10], offsetData[11]);
  #endif

	SH5001_write(SH5001_ADDRESS, 0x8E, 1, &offsetData[2]);
	SH5001_write(SH5001_ADDRESS, 0x8F, 1, &offsetData[3]);
	SH5001_write(SH5001_ADDRESS, 0x9A, 1, &offsetData[6]);
	SH5001_write(SH5001_ADDRESS, 0x9B, 1, &offsetData[7]);
	SH5001_write(SH5001_ADDRESS, 0xA6, 1, &offsetData[10]);
	SH5001_write(SH5001_ADDRESS, 0xA7, 1, &offsetData[11]);

	//for test
	#if 0
	for(i=0; i<SH5001_OFFSET_REG_NUM; i++)
	{
		SH5001_read(SH5001_ADDRESS, offsetRegister[i], 1, &offsetData[i]);
		#ifdef SH5001_OT_DEBUG
		printf("read back 0x%x = 0x%x\r\n", offsetRegister[i], offsetData[i]);
		#endif
	}
	#endif
}

#define SH5001_TEMPCOMP_REG_NUM 10
void temp_comp_save_reg(float k[3])
{
	unsigned char tempCompData[SH5001_OFFSET_REG_NUM] = {0};

	signed char beta_otp[3] = {0};
	short gain_otp[3] = {0};
	unsigned int range = 1;

	unsigned char  beta_new[3] = {0};
	signed char  s_beta_new[3] = {0};
	int  s_beta_temp[3] = {0};

	unsigned char i;

	unsigned char tempCompRegister[SH5001_OFFSET_REG_NUM] = { 0x8C, 0x8D,  0x98, 0x99,  0xA4, 0xA5 , 0x92,0x9e,0xAA, 0x24};

	for(i=0; i<SH5001_TEMPCOMP_REG_NUM; i++)
	{
		SH5001_read(SH5001_ADDRESS, tempCompRegister[i], 1, &tempCompData[i]);
		#ifdef SH5001_OT_DEBUG
		printf("0x%x = 0x%x\r\n", tempCompRegister[i], tempCompData[i]);
		#endif
	}

	for(i=0; i<3; i++)
	{
		k[i] = k[i] * 16.4 / 14.0; // deg/c  to lsb/lsb  range2000dps
	}

	#ifdef SH5001_OT_DEBUG
	printf("k %f %f %f \r\n", k[0], k[1],k[2]);
	#endif

	gain_otp[0] = ((short)tempCompData[1] << 8) | tempCompData[0];
	gain_otp[1] = ((short)tempCompData[3] << 8) | tempCompData[2];
	gain_otp[2] = ((short)tempCompData[5] << 8) | tempCompData[4];

	//TODO unsigned to signed
	beta_otp[0] = ((signed char)tempCompData[6]);
	beta_otp[1] = ((signed char)tempCompData[7]);
	beta_otp[2] = ((signed char)tempCompData[8]);

	tempCompData[9] = (tempCompData[9] & 0x70) >> 4;

	#ifdef SH5001_OT_DEBUG
	printf("gain_otp %d %d %d \r\n", gain_otp[0], gain_otp[1],gain_otp[2]);
	printf("temp_otp %d %d %d \r\n", beta_otp[0], beta_otp[1],beta_otp[2]);
	printf("range 0x%x \r\n", tempCompData[9]);
	#endif

	if(tempCompData[9] != 0)
	{
			for(i=0; i < tempCompData[9] + 14; i++)
			{
				range = range * 2;
			}
	}

	#ifdef SH5001_OT_DEBUG
	printf("range222 %d \r\n", range);
	#endif
  //TODO
	for(i = 0; i < 3; i++)
	{
		s_beta_temp[i] = beta_otp[i] - (int)(k[i] * ((float)range) / ((float)gain_otp[i]));
		#ifdef SH5001_OT_DEBUG
			printf("beta_ 111 %f %d\r\n", k[i] * range / gain_otp[i], (int)(k[i] * range / gain_otp[i]));
		#endif
	}

	if(s_beta_temp[0] > 127)
		s_beta_temp[0] = 127;
	else if(s_beta_temp[0] < -128)
		s_beta_temp[0] = -128;
	if(s_beta_temp[1] > 127)
		s_beta_temp[1] = 127;
	else if(s_beta_temp[1] < -128)
		s_beta_temp[1] = -128;
	if(s_beta_temp[2] > 127)
		s_beta_temp[2] = 127;
	else if(s_beta_temp[2] < -128)
		s_beta_temp[2] = -128;

	s_beta_new[0] = (char)s_beta_temp[0];
	s_beta_new[1] = (char)s_beta_temp[1];
	s_beta_new[2] = (char)s_beta_temp[2];

	beta_new[0] = (unsigned char)s_beta_new[0];
	beta_new[1] = (unsigned char)s_beta_new[1];
	beta_new[2] = (unsigned char)s_beta_new[2];

	#ifdef SH5001_OT_DEBUG
	printf("s_beta_new %d %d %d \r\n", s_beta_new[0], s_beta_new[1],s_beta_new[2]);
	printf("beta_new %x %x %x \r\n", beta_new[0], beta_new[1],beta_new[2]);
    #endif

	SH5001_write(SH5001_ADDRESS, 0x92, 1, &beta_new[0]);
	SH5001_write(SH5001_ADDRESS, 0x9e, 1, &beta_new[1]);
	SH5001_write(SH5001_ADDRESS, 0xAA, 1, &beta_new[2]);
}

#define SH5001_ACC_REG_NUM 4
void acc_temp_comp_save_reg(float k[3])
{
	unsigned char tempCompData[SH5001_ACC_REG_NUM] = {0};
	signed char beta_otp[3] = {0};
	short gain_otp[3] = {0};
	unsigned int range = 1;
	float rangek = 1;
	unsigned char beta_new[3] = {0};
	signed char s_beta_new[3] = {0};
	int s_beta_temp[3] = {0};
	unsigned char i;
	unsigned char tempCompRegister[SH5001_ACC_REG_NUM] = {0x71, 0x7c, 0x88, 0x21};
	for(i=0; i<SH5001_ACC_REG_NUM; i++)
	{
		SH5001_read(SH5001_ADDRESS, tempCompRegister[i], 1, &tempCompData[i]);
		#ifdef SH5001_OT_DEBUG
		printf("0x%x = 0x%x\r\n", tempCompRegister[i], tempCompData[i]);
		#endif
	}
	beta_otp[0] = ((signed char)tempCompData[0]);
	beta_otp[1] = ((signed char)tempCompData[1]);
	beta_otp[2] = ((signed char)tempCompData[2]);
	tempCompData[3] = (tempCompData[3] & 0x70) >> 4;
	#ifdef SH5001_OT_DEBUG
	printf("temp_otp %d %d %d \r\n", beta_otp[0], beta_otp[1],beta_otp[2]);
	printf("range 0x%x \r\n", tempCompData[3]);
	#endif
	if(tempCompData[3] != 0)
	{
			for(i=0; i < 6 - tempCompData[3]; i++)
			{
				range = range * 2;
			}
	}
	for(i=0; i<3; i++)
	{
		k[i] = k[i] * range * 2.048 / 14;
	}
	#ifdef SH5001_OT_DEBUG
	printf("k %f %f %f \r\n", k[0], k[1],k[2]);
	#endif
	if(tempCompData[3] != 0)
	{
			for(i=0; i < tempCompData[3] + 1; i++)
			{
				rangek = rangek * 2;
			}
	}
	#ifdef SH5001_OT_DEBUG
	printf("range222 %f \r\n", rangek);
	#endif
	for(i = 0; i < 3; i++)
	{
		s_beta_temp[i] = beta_otp[i] - (int)(k[i] * rangek);
		#ifdef SH5001_OT_DEBUG
		printf("beta_111 %f %d\r\n", k[i] * rangek , (int)(k[i] * rangek));
		#endif
	}
	if(s_beta_temp[0] > 127)
		s_beta_temp[0] = 127;
	else if(s_beta_temp[0] < -128)
		s_beta_temp[0] = -128;
	if(s_beta_temp[1] > 127)
		s_beta_temp[1] = 127;
	else if(s_beta_temp[1] < -128)
		s_beta_temp[1] = -128;
	if(s_beta_temp[2] > 127)
		s_beta_temp[2] = 127;
	else if(s_beta_temp[2] < -128)
		s_beta_temp[2] = -128;
	s_beta_new[0] = (char)s_beta_temp[0];
	s_beta_new[1] = (char)s_beta_temp[1];
	s_beta_new[2] = (char)s_beta_temp[2];
	beta_new[0] = (unsigned char)s_beta_new[0];
	beta_new[1] = (unsigned char)s_beta_new[1];
	beta_new[2] = (unsigned char)s_beta_new[2];
	#ifdef SH5001_OT_DEBUG
	printf("s_beta_new %d %d %d \r\n", s_beta_new[0], s_beta_new[1],s_beta_new[2]);
	printf("beta_new %x %x %x \r\n", beta_new[0], beta_new[1],beta_new[2]);
    #endif
	SH5001_write(SH5001_ADDRESS, 0x71, 1, &beta_new[0]);
	SH5001_write(SH5001_ADDRESS, 0x7c, 1, &beta_new[1]);
	SH5001_write(SH5001_ADDRESS, 0x88, 1, &beta_new[2]);
}

#define SH5001_QUAD_REG_NUM 10
#define SH5001_QUAD_DEBUG

void sh5001_gyro_trim_quad(unsigned char quadval[6])
{
  short S_Gyro_Quad[3] = {0};
  short C_Gyro_Quad[3] = {0};
  unsigned char quadData[SH5001_QUAD_REG_NUM] = {0};
  int sumGyroData[3] = {0};
  short avgGyroData[3] = {0};
  short GyroData[3] = {0};
  unsigned char temp[6]= {0};
  unsigned char i;
  double Slope[3] = {560, 560, 560};

  unsigned char quadRegister[SH5001_QUAD_REG_NUM] = {0x8B, 0x97, 0xA3, 0xD9,  0xC7, 0xC8, 0xC9, 0xCA, 0xCB, 0xCC};
  unsigned char RegWrt_1[SH5001_QUAD_REG_NUM] = {0xFF, 0xFF, 0xFF, 0x91, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  for(i=0; i<SH5001_QUAD_REG_NUM; i++)
  {
    SH5001_read(SH5001_ADDRESS, quadRegister[i], 1, &quadData[i]);
    #ifdef SH5001_QUAD_DEBUG
    printf("quadRegister RegWrt_1 raw 0x%x:0x%x\r\n",quadRegister[i], quadData[i]);
    #endif
  }

  for(i=0; i<4; i++)
  {
    SH5001_write(SH5001_ADDRESS, quadRegister[i], 1, &RegWrt_1[i]);
  }

  delay_ms(80);

  for(i=20; i>0; i--)
  {
    delay_ms(1);

    SH5001_read(SH5001_ADDRESS, 0x06, 6, temp);
    GyroData[0]=((short)(temp[1]<<8))|temp[0];
    GyroData[1]=((short)(temp[3]<<8))|temp[2];
    GyroData[2]=((short)(temp[5]<<8))|temp[4];

    sumGyroData[0] += GyroData[0];
    sumGyroData[1] += GyroData[1];
    sumGyroData[2] += GyroData[2];

    #ifdef SH5001_QUAD_DEBUG
    printf("sumGyroData %d %d %d\r\n",GyroData[0], GyroData[1], GyroData[2]);
    #endif
  }

  for(i = 0; i < 3; i++)
  {
    avgGyroData[i] = sumGyroData[i] / 20;
  }

  #ifdef SH5001_QUAD_DEBUG
  printf("avgGyroData %d %d %d\r\n",avgGyroData[0], avgGyroData[1], avgGyroData[2]);
  #endif

  for (i = 0; i < 3; i++)
  {
      S_Gyro_Quad[i] = (quadData[i + 7] & 0x04) >> 2;
      C_Gyro_Quad[i] = quadData[i + 4] + (quadData[i + 7] & 0x03) * 256;

      #ifdef SH5001_QUAD_DEBUG
      printf("raw S_Gyro_Quad[%d] : %d  C_Gyro_Quad[%d]: %d  \r\n",i, S_Gyro_Quad[i], i, C_Gyro_Quad[i]);
      #endif

      C_Gyro_Quad[i] = (S_Gyro_Quad[i] == 0) ? C_Gyro_Quad[i]: (C_Gyro_Quad[i] * (-1) - 1);
      C_Gyro_Quad[i] = (int)(C_Gyro_Quad[i] - avgGyroData[i] / Slope[i]);

      S_Gyro_Quad[i] = C_Gyro_Quad[i] >= 0 ? 0 : 1;
      C_Gyro_Quad[i] = C_Gyro_Quad[i] >= 0 ? C_Gyro_Quad[i]: (C_Gyro_Quad[i] * (-1) - 1);

      RegWrt_1[i + 4] = C_Gyro_Quad[i] & 0x00FF;
      RegWrt_1[i + 7] = (quadData[i + 7] & 0xF8) + S_Gyro_Quad[i] * 4 + ((C_Gyro_Quad[i] & 0x0300) >> 8);

  }
  #ifdef SH5001_QUAD_DEBUG
  printf("mod S_Gyro_Quad %d %d %d C_Gyro_Quad %d %d %d RegWrt_1 %d %d %d %d %d %d\r\n",S_Gyro_Quad[0],
  S_Gyro_Quad[1],S_Gyro_Quad[2],C_Gyro_Quad[0],C_Gyro_Quad[1],C_Gyro_Quad[2], RegWrt_1[4], RegWrt_1[5], RegWrt_1[6], RegWrt_1[7], RegWrt_1[8], RegWrt_1[9]);
  #endif


  //TODO Save Reg
  for(i=0; i<6; i++) {
    SH5001_write(SH5001_ADDRESS, quadRegister[i + 4], 1, &RegWrt_1[i + 4]);
    quadval[i] = RegWrt_1[i + 4];
  }

  for(i=0; i<4; i++)
    SH5001_write(SH5001_ADDRESS, quadRegister[i], 1, &quadData[i]);
}


/******************************************************************
* Description: 	SH5001 Acc self-test function
*
* Parameters:	void
*
* return:	SH3001_TRUE or SH3001_FALSE
*
******************************************************************/

#define  SH5001_ACC_SELFTEST_REGNUM 7
#define  SH5001_ACC_SELFTEST_DEBUG

unsigned char SH5001_AccSelfTest(void)
{
	unsigned char regAddr[SH5001_ACC_SELFTEST_REGNUM] = {0x21, 0x69, 0x74, 0x80, 0xB9, 0xBA, 0xBB};
	unsigned char regInit[SH5001_ACC_SELFTEST_REGNUM] = {0x30, 0xFF, 0xFF, 0xFF, 0x40, 0x45, 0x45};
	unsigned char regBack[SH5001_ACC_SELFTEST_REGNUM] = {0};
	unsigned char accTemp[6] = {0};
	signed int accSum[3] = {0};
	signed short accAvg1[3] = {0}, accAvg2[3] = {0};
	unsigned char i = 0;
	signed short accRange[3] = {0};


	for(i=0; i<SH5001_ACC_SELFTEST_REGNUM; i++)
	{
		SH5001_read(SH5001_ADDRESS, regAddr[i], 1, &regBack[i]);
	}

	for(i=0; i<SH5001_ACC_SELFTEST_REGNUM; i++)
	{
		SH5001_write(SH5001_ADDRESS, regAddr[i], 1, &regInit[i]);
	}

	delay_ms(100);

	i = 0;
	while(i++ < 20)
	{
		SH5001_read(SH5001_ADDRESS, SH5001_ACC_XL, 6, accTemp);
		accSum[0] += ((short)accTemp[1] << 8) | accTemp[0];
		accSum[1] += ((short)accTemp[3] << 8) | accTemp[2];
		accSum[2] += ((short)accTemp[5] << 8) | accTemp[4];

		#ifdef SH5001_ACC_SELFTEST_DEBUG
		printf("bf %d %d %d\r\n", ((short)accTemp[1] << 8) | accTemp[0], ((short)accTemp[3] << 8) | accTemp[2], ((short)accTemp[5] << 8) | accTemp[4]);
		#endif
	}
	accAvg1[0] = accSum[0]/20;
	accAvg1[1] = accSum[1]/20;
	accAvg1[2] = accSum[2]/20;


	accTemp[0] = 0x60;
	SH5001_write(SH5001_ADDRESS, 0xB9, 1, &accTemp[0]);
	accTemp[0] = 0x65;
	SH5001_write(SH5001_ADDRESS, 0xBA, 1, &accTemp[0]);
	accTemp[0] = 0x65;
	SH5001_write(SH5001_ADDRESS, 0xBB, 1, &accTemp[0]);

	delay_ms(100);

	accSum[0] = 0;
	accSum[1] = 0;
	accSum[2] = 0;
	i = 0;
	while(i++ < 20)
	{
		SH5001_read(SH5001_ADDRESS, SH5001_ACC_XL, 6, accTemp);
		accSum[0] += ((short)accTemp[1] << 8) | accTemp[0];
		accSum[1] += ((short)accTemp[3] << 8) | accTemp[2];
		accSum[2] += ((short)accTemp[5] << 8) | accTemp[4];
		#ifdef SH5001_ACC_SELFTEST_DEBUG
		printf("af %d %d %d\r\n", ((short)accTemp[1] << 8) | accTemp[0], ((short)accTemp[3] << 8) | accTemp[2], ((short)accTemp[5] << 8) | accTemp[4]);
		#endif
	}
	accAvg2[0] = accSum[0]/20;
	accAvg2[1] = accSum[1]/20;
	accAvg2[2] = accSum[2]/20;


	for(i=0; i<SH5001_ACC_SELFTEST_REGNUM; i++)
	{
		SH5001_write(SH5001_ADDRESS, regAddr[i], 1, &regBack[i]);
	}

	accRange[0] = accAvg2[0] - accAvg1[0];
	accRange[1] = accAvg2[1] - accAvg1[1];
	accRange[2] = accAvg2[2] - accAvg1[2];

	#ifdef SH5001_ACC_SELFTEST_DEBUG
	printf("accdiff %d %d %d\r\n", accRange[0],accRange[1] ,accRange[2]);
	#endif

	if(((accRange[0] > 3900) && (accRange[0] < 9200))
	&& ((accRange[1] > 4100) && (accRange[1] < 9600))
	&& ((accRange[2] > 4500) && (accRange[2] < 10900)))
	{
		return(SH5001_TRUE);
	}
	else
	{
		return(SH5001_FALSE);
	}
}


/******************************************************************
* Description: 	SH5001 Gyro self-test function
*
* Parameters:	void
*
* return:	SH3001_TRUE or SH3001_FALSE
*
******************************************************************/

#define SH5001_GYRO_SELFTEST_REGNUM 11
#define  SH5001_GYRO_SELFTEST_DEBUG

unsigned char SH5001_GyroSelfTest(void)
{
	unsigned char regAddr[SH5001_GYRO_SELFTEST_REGNUM] = {0x24, 0x8B, 0x97, 0xA3, 0xCA, 0xCB, 0xCC, 0xCD,\
								0xCE, 0xCF, 0xD9};
	unsigned char regInit[SH5001_GYRO_SELFTEST_REGNUM] = {0x60, 0xFF, 0xFF, 0xFF, 0x60, 0x60, 0x60, 0x02,\
								0x02, 0x02, 0x91};
	unsigned char regBack[SH5001_GYRO_SELFTEST_REGNUM] = {0};
	unsigned char gyroTemp[6] = {0};
	signed int gyroSum[3] = {0};
	signed short gyroAvg1[3] = {0}, gyroAvg2[3] = {0};
	unsigned char i = 0;
	signed short gyroRange[3] = {0};

	for(i=0; i<SH5001_GYRO_SELFTEST_REGNUM; i++)
	{
		SH5001_read(SH5001_ADDRESS, regAddr[i], 1, &regBack[i]);
	}

	for(i=0; i<SH5001_GYRO_SELFTEST_REGNUM; i++)
	{
		SH5001_write(SH5001_ADDRESS, regAddr[i], 1, &regInit[i]);
	}

	//Gyro x
	gyroTemp[0] = 0x15;
	SH5001_write(SH5001_ADDRESS, 0xCD, 1, &gyroTemp[0]);
	gyroTemp[0] = 0x02;
	SH5001_write(SH5001_ADDRESS, 0xCE, 1, &gyroTemp[0]);
	gyroTemp[0] = 0x02;
	SH5001_write(SH5001_ADDRESS, 0xCF, 1, &gyroTemp[0]);

	delay_ms(50);

	i = 0;
	while(i++ < 20)
	{
		SH5001_read(SH5001_ADDRESS, SH5001_GYRO_XL, 2, gyroTemp);
		gyroSum[0] += ((short)gyroTemp[1] << 8) | gyroTemp[0];
		#ifdef SH5001_GYRO_SELFTEST_DEBUG
		printf("Gx bf %d \r\n", (signed short)(((short)gyroTemp[1] << 8) | gyroTemp[0]));
		#endif
	}
	gyroAvg1[0] = gyroSum[0]/20;

	gyroTemp[0] = 0x14;
	SH5001_write(SH5001_ADDRESS, 0xCD, 1, &gyroTemp[0]);
	gyroTemp[0] = 0x02;
	SH5001_write(SH5001_ADDRESS, 0xCE, 1, &gyroTemp[0]);
	gyroTemp[0] = 0x02;
	SH5001_write(SH5001_ADDRESS, 0xCF, 1, &gyroTemp[0]);

	delay_ms(50);

	gyroSum[0] = 0;

	i = 0;
	while(i++ < 20)
	{
		SH5001_read(SH5001_ADDRESS, SH5001_GYRO_XL, 2, gyroTemp);
		gyroSum[0] += ((short)gyroTemp[1] << 8) | gyroTemp[0];

		#ifdef SH5001_GYRO_SELFTEST_DEBUG
		printf("Gx af %d \r\n", (signed short)(((short)gyroTemp[1] << 8) | gyroTemp[0]));
		#endif
	}
	gyroAvg2[0] = gyroSum[0]/20;

	//gyro y
	gyroTemp[0] = 0x02;
	SH5001_write(SH5001_ADDRESS, 0xCD, 1, &gyroTemp[0]);
	gyroTemp[0] = 0x15;
	SH5001_write(SH5001_ADDRESS, 0xCE, 1, &gyroTemp[0]);
	gyroTemp[0] = 0x02;
	SH5001_write(SH5001_ADDRESS, 0xCF, 1, &gyroTemp[0]);

	delay_ms(50);

	i = 0;
	while(i++ < 20)
	{
		SH5001_read(SH5001_ADDRESS, SH5001_GYRO_YL, 2, gyroTemp);
		gyroSum[1] += ((short)gyroTemp[1] << 8) | gyroTemp[0];

		#ifdef SH5001_GYRO_SELFTEST_DEBUG
		printf("Gy bf %d \r\n", (signed short)(((short)gyroTemp[1] << 8) | gyroTemp[0]));
		#endif
	}
	gyroAvg1[1] = gyroSum[1]/20;

	gyroTemp[0] = 0x02;
	SH5001_write(SH5001_ADDRESS, 0xCD, 1, &gyroTemp[0]);
	gyroTemp[0] = 0x14;
	SH5001_write(SH5001_ADDRESS, 0xCE, 1, &gyroTemp[0]);
	gyroTemp[0] = 0x02;
	SH5001_write(SH5001_ADDRESS, 0xCF, 1, &gyroTemp[0]);

	delay_ms(50);

	gyroSum[1] = 0;

	i = 0;
	while(i++ < 20)
	{
		SH5001_read(SH5001_ADDRESS, SH5001_GYRO_YL, 2, gyroTemp);
		gyroSum[1] += ((short)gyroTemp[1] << 8) | gyroTemp[0];

		#ifdef SH5001_GYRO_SELFTEST_DEBUG
		printf("Gy af %d \r\n", (signed short)(((short)gyroTemp[1] << 8) | gyroTemp[0]));
		#endif

	}
	gyroAvg2[1] = gyroSum[1]/20;

	//gyro z
	gyroTemp[0] = 0x02;
	SH5001_write(SH5001_ADDRESS, 0xCD, 1, &gyroTemp[0]);
	gyroTemp[0] = 0x02;
	SH5001_write(SH5001_ADDRESS, 0xCE, 1, &gyroTemp[0]);
	gyroTemp[0] = 0x15;
	SH5001_write(SH5001_ADDRESS, 0xCF, 1, &gyroTemp[0]);

	delay_ms(50);

	i = 0;
	while(i++ < 20)
	{
		SH5001_read(SH5001_ADDRESS, SH5001_GYRO_ZL, 2, gyroTemp);
		gyroSum[2] += ((short)gyroTemp[1] << 8) | gyroTemp[0];
		#ifdef SH5001_GYRO_SELFTEST_DEBUG
		printf("Gz bf %d \r\n", (signed short)(((short)gyroTemp[1] << 8) | gyroTemp[0]));
		#endif
	}
	gyroAvg1[2] = gyroSum[2]/20;

	gyroTemp[0] = 0x02;
	SH5001_write(SH5001_ADDRESS, 0xCD, 1, &gyroTemp[0]);
	gyroTemp[0] = 0x02;
	SH5001_write(SH5001_ADDRESS, 0xCE, 1, &gyroTemp[0]);
	gyroTemp[0] = 0x14;
	SH5001_write(SH5001_ADDRESS, 0xCF, 1, &gyroTemp[0]);

	delay_ms(50);

	gyroSum[2] = 0;

	i = 0;
	while(i++ < 20)
	{
		SH5001_read(SH5001_ADDRESS, SH5001_GYRO_ZL, 2, gyroTemp);
		gyroSum[2] += ((short)gyroTemp[1] << 8) | gyroTemp[0];
		#ifdef SH5001_GYRO_SELFTEST_DEBUG
		printf("Gz af %d\r\n", (signed short)(((short)gyroTemp[1] << 8) | gyroTemp[0]));
		#endif
	}
	gyroAvg2[2] = gyroSum[2]/20;

	for(i=0; i<SH5001_GYRO_SELFTEST_REGNUM; i++)
	{
		SH5001_write(SH5001_ADDRESS, regAddr[i], 1, &regBack[i]);
	}

	#ifdef SH5001_GYRO_SELFTEST_DEBUG
	printf("G b average %d %d %d\r\n", gyroAvg1[0],gyroAvg1[1] ,gyroAvg1[2]);
	printf("G a average %d %d %d\r\n", gyroAvg2[0],gyroAvg2[1] ,gyroAvg2[2]);
	#endif

	gyroRange[0] = gyroAvg2[0] - gyroAvg1[0];
	gyroRange[1] = gyroAvg2[1] - gyroAvg1[1];
	gyroRange[2] = gyroAvg2[2] - gyroAvg1[2];

	#ifdef SH5001_GYRO_SELFTEST_DEBUG
	printf("Gyro diff %d %d %d\r\n", gyroRange[0],gyroRange[1] ,gyroRange[2]);
	#endif

	if(((gyroRange[0] > -4500) && (gyroRange[0] < 7100))
		&& ((gyroRange[1] > -6900) && (gyroRange[1] < 8200))
		&& ((gyroRange[2] > -9300) && (gyroRange[2] < 4600)))
	{
		return(SH5001_TRUE);
	}
	else
	{
		return(SH5001_FALSE);
	}
}

void sh5001_adc_phase(void)
{
	unsigned char addr = 0xD0;
	unsigned char regData;

	SH5001_read(SH5001_ADDRESS, addr, 1, &regData);
	//printf("0xD0 = 0x%x\r\n", regData);

	if((regData & 0x02) == 0x02)
		regData &= 0xFD;
	else
		regData |= 0x02;
	SH5001_write(SH5001_ADDRESS, addr, 1, &regData);
	//printf("0xD0 = 0x%x\r\n", regData);

}

#define SH5001_GYRO_ADC_S1_REGNUM 5
#define SH5001_GYRO_ADC_S2_REGNUM 1
//#define SH5001_GYRO_ADC__DEBUG
int adc_count = 0;
int err_count = 0;
void SH5001_Get_Adc_Polarity(short gyroPolarity[3])
{
	unsigned char regAddrS1[SH5001_GYRO_ADC_S1_REGNUM] = {0xA3, 0xC6, 0xCC, 0xCF, 0xD0};
	unsigned char regInitS1[SH5001_GYRO_ADC_S1_REGNUM] = {0xFF, 0x00, 0x69, 0x1A, 0x0A};
	unsigned char regAddrS2[SH5001_GYRO_ADC_S2_REGNUM] = {0xD0};
	unsigned char regInitS2[SH5001_GYRO_ADC_S2_REGNUM] = {0x08};
	unsigned char regBackS1[SH5001_GYRO_ADC_S1_REGNUM] = {0};
	unsigned char gyroTemp[6] = {0};
	signed int gyroSum[3] = {0};
	signed short gyroAvg1[3] = {0}, gyroAvg2[3] = {0};
	unsigned char i = 0;
	//signed short gyroPolarity[3] = {0};

	for(i=0; i<SH5001_GYRO_ADC_S1_REGNUM; i++)
	{
		SH5001_read(SH5001_ADDRESS, regAddrS1[i], 1, &regBackS1[i]);
	}

	for(i=0; i<SH5001_GYRO_ADC_S1_REGNUM; i++)
	{
		SH5001_write(SH5001_ADDRESS, regAddrS1[i], 1, &regInitS1[i]);
	}

	delay_ms(50);

	i = 0;
	while(i++ < 20)
	{
		SH5001_read(SH5001_ADDRESS, SH5001_GYRO_XL, 6, gyroTemp);
		gyroSum[0] += (signed short)((short)gyroTemp[1] << 8) | gyroTemp[0];
		gyroSum[1] += (signed short)((short)gyroTemp[3] << 8) | gyroTemp[2];
		gyroSum[2] += (signed short)((short)gyroTemp[5] << 8) | gyroTemp[4];
		#ifdef SH5001_GYRO_ADC__DEBUG
		printf("bf %d %d %d\r\n", (signed short)(((short)gyroTemp[1] << 8) | gyroTemp[0]), (signed short)(((short)gyroTemp[3] << 8) | gyroTemp[2]), (signed short)(((short)gyroTemp[5] << 8) | gyroTemp[4]));
		#endif
	}
	gyroAvg1[0] = gyroSum[0]/20;
	gyroAvg1[1] = gyroSum[1]/20;
	gyroAvg1[2] = gyroSum[2]/20;

	for(i=0; i<SH5001_GYRO_ADC_S2_REGNUM; i++)
	{
		SH5001_write(SH5001_ADDRESS, regAddrS2[i], 1, &regInitS2[i]);
	}

	delay_ms(50);

	gyroSum[0] = 0;
	gyroSum[1] = 0;
	gyroSum[2] = 0;
	i = 0;

	while(i++ < 20)
	{
		SH5001_read(SH5001_ADDRESS, SH5001_GYRO_XL, 6, gyroTemp);
		gyroSum[0] += (signed short)((short)gyroTemp[1] << 8) | gyroTemp[0];
		gyroSum[1] += (signed short)((short)gyroTemp[3] << 8) | gyroTemp[2];
		gyroSum[2] += (signed short)((short)gyroTemp[5] << 8) | gyroTemp[4];

		#ifdef SH5001_GYRO_ADC__DEBUG
		printf("af %d %d %d\r\n", (signed short)(((short)gyroTemp[1] << 8) | gyroTemp[0]), (signed short)(((short)gyroTemp[3] << 8) | gyroTemp[2]), (signed short)(((short)gyroTemp[5] << 8) | gyroTemp[4]));
		#endif
	}

	gyroAvg2[0] = gyroSum[0]/20;
	gyroAvg2[1] = gyroSum[1]/20;
	gyroAvg2[2] = gyroSum[2]/20;

	for(i=0; i<SH5001_GYRO_ADC_S1_REGNUM; i++)
	{
		SH5001_write(SH5001_ADDRESS, regAddrS1[i], 1, &regBackS1[i]);
	}

	gyroPolarity[0] = gyroAvg2[0] - gyroAvg1[0];
	gyroPolarity[1] = gyroAvg2[1] - gyroAvg1[1];
	gyroPolarity[2] = gyroAvg2[2] - gyroAvg1[2];
	adc_count++;
	#ifdef SH5001_GYRO_ADC__DEBUG
	printf("gyroPolarity %d %d %d %d %d\r\n", adc_count, err_count, gyroPolarity[0],gyroPolarity[1] ,gyroPolarity[2]);
	#endif

	if(gyroPolarity[2] < 0)
	{
		err_count++;
	}

}

int sh5001_get_version(void)
{
	return MCU_VERSION_SH5001;
}