/****************************************************************************
* �� �� ��: mpu6050.c
* ��    ��: Amo [ www.amoMcu.com ��Ī��Ƭ��]
* ��    ��: 2014-04-08
* ��    ��: 1.0
* ��    ��: mpu6050����������
****************************************************************************/

#include "stm32g431xx.h"
#include "i2c.h"
#include "MPU6050.h"

#define I2C_PORT_NUM        1
#define BUFSIZE             14  //ȷ�ϴ�С������


volatile uint8_t I2CMasterBuffer[I2C_PORT_NUM][BUFSIZE];
volatile uint8_t I2CSlaveBuffer[I2C_PORT_NUM][BUFSIZE];
volatile uint32_t I2CReadLength[I2C_PORT_NUM];
volatile uint32_t I2CWriteLength[I2C_PORT_NUM];

void DelayI2C(uint32_t m)
{
  uint32_t i;
  
  for(; m != 0; m--)    
       for (i=0; i<5; i++);
}

void MPU6050_Read(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char readNum)
{
    unsigned char i; 
    for(i = 0; i<readNum; i++)
    {
        //��ĳ��IIC����������
        I2CSlaveBuffer[PORT_USED][i] = Read_Add(REG_Address + i, SlaveAddress);
    }
}

unsigned char MPU6050_Read_1BYTE(unsigned char SlaveAddress,unsigned char REG_Address)
{
    return Read_Add(REG_Address, SlaveAddress);
}


void MPU6050_Write(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data) 
{
    //��ĳ��IIC����дָ���ĳ����ַ���������
    Write_Add(REG_Address, REG_data, SlaveAddress);
}

void MPU6050_WriteBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) 
{
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
    uint8_t tmp;
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    MPU6050_Read(slaveAddr, regAddr, 1);
    tmp = I2CSlaveBuffer[PORT_USED][0];  
//    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    data <<= (bitStart - length + 1); // shift data into correct position
    data &= mask;   // zero all non-important bits in data
    tmp &= ~(mask); // zero all important bits in existing byte
    tmp |= data;    // combine data with existing byte
    MPU6050_Write(slaveAddr,regAddr,tmp);   
}

void MPU6050_WriteBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data) 
{
    uint8_t tmp;
    MPU6050_Read(slaveAddr, regAddr, 1);
    tmp = I2CSlaveBuffer[PORT_USED][0];  
    tmp = (data != 0) ? (tmp | (1 << bitNum)) : (tmp & ~(1 << bitNum));
    MPU6050_Write(slaveAddr,regAddr,tmp); 
}

void MPU6050_ReadBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data) 
{
    // 01101001 read byte
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted

#if 1 //  �����ȡ��ȥ�����ε�λ�������ƶ�
    uint8_t tmp;
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    MPU6050_Read(slaveAddr, regAddr, 1);
    tmp = I2CSlaveBuffer[PORT_USED][0]; 
//    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    tmp &= mask;
    tmp >>= (bitStart - length + 1);
    *data = tmp;
#else
    uint8_t tmp;
    MPU6050_Read(slaveAddr, regAddr, 1);
    tmp = I2CSlaveBuffer[PORT_USED][0]; 
    *data = tmp;
#endif
}

void MPU6050_Initialize(void)
{
    MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, 
        MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, MPU6050_CLOCK_PLL_XGYRO);
    MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG,
        MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, MPU6050_GYRO_FS_250);
    MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 
        MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, MPU6050_ACCEL_FS_2);
    MPU6050_WriteBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, 0);//(DISABLE); 
}

uint8_t MPU6050_GetDeviceID()
{
    uint8_t tmp;

    MPU6050_ReadBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_WHO_AM_I, \
        MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH, &tmp);
    
    return tmp<<(MPU6050_WHO_AM_I_BIT-MPU6050_WHO_AM_I_LENGTH+1);  // ע���������λ  amomcu
}

//���ٶȲ���, �¶Ȳ����� �����ǲ���
void MPU6050_GetRawAccelGyro(int16_t* ax, int16_t* ay, int16_t* az, int16_t* temperature, int16_t* gx, int16_t* gy, int16_t* gz) 
{
//    u8 tmpBuffer[14]; 
    //int i;
    MPU6050_Read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, 14); 

    /* Get acceleration */
 //   for(i=0; i<3; i++) 
//      AccelGyro[i]=((int16_t)((uint16_t)I2CSlaveBuffer[PORT_USED][2*i] << 8) + I2CSlaveBuffer[PORT_USED][2*i+1]);
   /* Get Angular rate */
//    for(i=4; i<7; i++)
//      AccelGyro[i-1]=((int16_t)((uint16_t)I2CSlaveBuffer[PORT_USED][2*i] << 8) + I2CSlaveBuffer[PORT_USED][2*i+1]); 
    *ax = (((int16_t)I2CSlaveBuffer[PORT_USED][0]) << 8) | ((int16_t)I2CSlaveBuffer[PORT_USED][1]); 
    *ay = (((int16_t)I2CSlaveBuffer[PORT_USED][2]) << 8) | ((int16_t)I2CSlaveBuffer[PORT_USED][3]); 
    *az = (((int16_t)I2CSlaveBuffer[PORT_USED][4]) << 8) | ((int16_t)I2CSlaveBuffer[PORT_USED][5]); 

    // �¶�
    *temperature = (((int16_t)I2CSlaveBuffer[PORT_USED][6]) << 8) | ((int16_t)I2CSlaveBuffer[PORT_USED][7]); 
    
    *gx = (((int16_t)I2CSlaveBuffer[PORT_USED][8]) << 8) | ((int16_t)I2CSlaveBuffer[PORT_USED][9]); 
    *gy = (((int16_t)I2CSlaveBuffer[PORT_USED][10]) << 8) | ((int16_t)I2CSlaveBuffer[PORT_USED][11]); 
    *gz = (((int16_t)I2CSlaveBuffer[PORT_USED][12]) << 8) | ((int16_t)I2CSlaveBuffer[PORT_USED][13]); 
}


#if 1
uint8_t bIfConnected = 0;
void accInit(void)
{
    unsigned char tempV = 0;
    IIC_Init();                     //IIC��ʼ��

    tempV = MPU6050_GetDeviceID();
    if(tempV == MPU6050_ADDRESS_AD0_LOW)          //�����Ƿ� mpu6050 �Ѿ�����
    {           
        bIfConnected = 1;
    }
    else
    {
        bIfConnected = 0;
    }
    
    if(bIfConnected)
    {
        //mpu6050���ӳɹ�

        //��ʼ��mpu6050
        MPU6050_Initialize();

        //��ȡmpu6050��ʵʱ����           
        //MPU6050_GetRawAccelGyro(&ax, &ay, &az, &gx, &gy, &gz);
        //sprintf(strTemp, "%d %d %d : %d %d %d", ax, ay, az, gx, gy, gz);
        //���ڴ�ӡ������
        //UartSendString(strTemp, strlen(strTemp));
    }
}
void accStop(void)
{

}
void accWriteReg(uint8 reg, uint8 val)
{
    if(bIfConnected)
    {
        MPU6050_Write(0x68<<1, reg, val);
    }
}
void accReadReg(uint8 reg, uint8 *pVal)
{
    if(bIfConnected)
    {
        *pVal = MPU6050_Read_1BYTE(0x68<<1, reg);
    }
}
void accReadAcc(int8 *pXVal, int8 *pYVal, int8 *pZVal)
{
    int16_t ax, ay, az;
    int16_t gx, gy, gz;    
    int16_t temperature;
    if(bIfConnected)
    {
        //��ȡmpu6050��ʵʱ����           
        MPU6050_GetRawAccelGyro(&ax, &ay, &az, &temperature, &gx, &gy, &gz);

        *pXVal = ax/100;
        *pYVal = ay/100;
        *pZVal = az/100;
    }
}

void accReadAccGro(int8 *pXAcc, int8 *pYAcc, int8 *pZAcc, int8 *pXGro, int8 *pYGro, int8 *pZGro)
{
    int16_t ax, ay, az;
    int16_t gx, gy, gz;    
    int16_t temperature;
    if(bIfConnected)
    {
        //��ȡmpu6050��ʵʱ����           
        MPU6050_GetRawAccelGyro(&ax, &ay, &az, &temperature, &gx, &gy, &gz);

        *pXAcc = ax/100;
        *pYAcc = ay/100;
        *pZAcc = az/100;

        *pXGro = gx/100;
        *pYGro = gy/100;
        *pZGro = gz/100;        
    }
}

#endif
