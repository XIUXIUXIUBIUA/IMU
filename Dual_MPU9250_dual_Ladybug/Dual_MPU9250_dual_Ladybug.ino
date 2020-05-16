/* 07/6/2017 Copyright Tlera Corporation
 *  
 *  Created by Kris Winer
 * 
 Demonstrate basic MPU-9250 functionality including parameterizing the register addresses, initializing the sensor, 
 getting properly scaled accelerometer, gyroscope, and magnetometer data out. 
 Addition of 9 DoF sensor fusion using open source Madgwick filter algorithm. 

 Sketch modified to read data from two MPU9250s, one at 0x68 and 0ne at 0x69, on the same I2C bus and get absolute orientation
 Sketch runs on the 3.3 V Ladybug STM32L432 Breakout Board.
 
 Library may be used freely and without limit with attribution.
*/
 
//#include "Wire.h"   
#include "MPU9250.h"
#include "BMP280.h"
#include <Wire_slave.h>
#include <flash_stm32.h>
#define SerialDebug false   // set to true to get Serial output for debugging
#define MAG_DATA_ADRESS		0x00
#define CALI_FLAG_ADRESS	0x50

// BMP280 FUNCTION SET



//


// MPU9250 Configuration
// Specify sensor full scale
/* Choices are:
 *  Gscale: GFS_250 == 250 dps, GFS_500 DPS == 500 dps, GFS_1000 == 1000 dps, and GFS_2000DPS == 2000 degrees per second gyro full scale
 *  Ascale: AFS_2G == 2 g, AFS_4G == 4 g, AFS_8G == 8 g, and AFS_16G == 16 g accelerometer full scale
 *  Mscale: MFS_14BITS == 0.6 mG per LSB and MFS_16BITS == 0.15 mG per LSB
 *  Mmode: Mmode == M_8Hz for 8 Hz data rate or Mmode = M_100Hz for 100 Hz data rate
 *  (1 + sampleRate) is a simple divisor of the fundamental 1000 kHz rate of the gyro and accel, so 
 *  sampleRate = 0x00 means 1 kHz sample rate for both accel and gyro, 0x04 means 200 Hz, etc.
 */
uint8_t Gscale = GFS_250DPS, Ascale = AFS_2G, Mscale = MFS_16BITS, Mmode = M_100Hz, sampleRate = 0x04;         
float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors
float motion = 0; // check on linear acceleration to determine motion
// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float pi = 3.141592653589793238462643383279502884f;
float GyroMeasError = pi * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = pi * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float beta = sqrtf(3.0f / 4.0f) * GyroMeasError;   // compute beta
float zeta = sqrtf(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
bool wakeup;

// Pin definitions
int  intPin1 = 9;  //  MPU9250 1 interrupt
int  intPin2 = 8;  //  MPU9250 2 interrupt
int  myLed  = 13; // red led

bool intFlag1 = false;
bool intFlag2 = false;
bool newMagData = false;

int16_t MPU9250Data1[7], MPU9250Data2[7]; // used to read all 14 bytes at once from the MPU9250 accel/gyro
int16_t magCount1[3], magCount2[3];    // Stores the 16-bit signed magnetometer sensor output
float   magCalibration1[3] = {0, 0, 0}, magCalibration2[3] = {0, 0, 0};  // Factory mag calibration and mag bias
float   temperature1, temperature2;    // Stores the MPU9250 internal chip temperature in degrees Celsius
float   SelfTest[6];    // holds results of gyro and accelerometer self test

// These can be measured once and entered here or can be calculated each time the device is powered on
float   gyroBias1[3] = {0.96, -0.21, 0.12}, accelBias1[3] = {0.00299, -0.00916, 0.00952};
float   gyroBias2[3] = {0.96, -0.21, 0.12}, accelBias2[3] = {0.00299, -0.00916, 0.00952};
float   magBias1[3] = {71.04, 122.43, -36.90}, magScale1[3]  = {1.01, 1.03, 0.96}; // Bias corrections for gyro and accelerometer
float   magBias2[3] = {71.04, 122.43, -36.90}, magScale2[3]  = {1.01, 1.03, 0.96}; // Bias corrections for gyro and accelerometer


uint32_t delt_t1, delt_t2 = 0;                      // used to control display output rate
uint32_t count1 = 0, sumCount1 = 0, count2 = 0, sumCount2 = 0;         // used to control display output rate
float pitch1, yaw1, roll1, pitch2, yaw2, roll2;                   // absolute orientation
float a12, a22, a31, a32, a33;            // rotation matrix coefficients for Euler angles and gravity components
float A12, A22, A31, A32, A33;            // rotation matrix coefficients for Euler angles and gravity components
float deltat1 = 0.0f, sum1 = 0.0f, deltat2 = 0.0f, sum2 = 0.0f;          // integration interval for both filter schemes
uint32_t lastUpdate1 = 0, lastUpdate2 = 0; // used to calculate integration interval
uint32_t Now1 = 0, Now2 = 0;                         // used to calculate integration interval

float ax1, ay1, az1, gx1, gy1, gz1, mx1, my1, mz1; // variables to hold latest sensor data values 
float ax2, ay2, az2, gx2, gy2, gz2, mx2, my2, mz2; // variables to hold latest sensor data values 
float lin_ax1, lin_ay1, lin_az1;             // linear acceleration (acceleration with gravity component subtracted)
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float lin_ax2, lin_ay2, lin_az2;             // linear acceleration (acceleration with gravity component subtracted)
float Q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion

// variable for communcating with arduino by I2C
uint8_t reg=0;
uint8_t mag_calibrate_flag = 0;
MPU9250 MPU9250(intPin1); // instantiate MPU9250 class
//一个将float类型弄成char类型以进行i2c通信的函数

char *memory_used = (char *)malloc(sizeof(float));
char *memory_used2 = (char *)malloc(sizeof(float));
void float2char(char *p,float x)
{
  // char *p;
  // p = (char *)malloc(sizeof(float));
  memcpy(p,&x,sizeof(float));
  // return p;
}

void receiveEvent(int num)
{
    while(Wire1.available())
    {
        reg = Wire1.read();
    }

}
void response_i2c()
{   
	// 浮点型数据的传输不能直接拆分字节发送
	// 采用内存复制的方式发送 用memcpy函数把float型数据的内存完全复制到char型数组中
	// 然后传输char型数组即可
    // reg == 0x01 读取3轴加速度数据，共12个字节
    // reg == 0x02 读取3轴角速度数据，共12个字节    
    // reg == 0x03 读取3轴磁力计数据，共12个字节 
    // reg == 0x04 读取姿态角Pitch、Roll、Yaw，共12个字节
    // reg == 0x05 磁力计校准，返回一个成功与否的信号   
    if(reg == 0x01)
    {
        ax1=0.13;
		float2char(memory_used,ax1);
        Wire1.write((uint8_t)memory_used[0]);
        Wire1.write((uint8_t)(memory_used[1]));
        Wire1.write((uint8_t)(memory_used[2]));
        Wire1.write((uint8_t)(memory_used[3]));

		float2char(memory_used,ay1);
        Wire1.write((uint8_t)memory_used[0]);
        Wire1.write((uint8_t)(memory_used[1]));
        Wire1.write((uint8_t)(memory_used[2]));
        Wire1.write((uint8_t)(memory_used[3]));

        float2char(memory_used,az1);
        Wire1.write((uint8_t)memory_used[0]);
        Wire1.write((uint8_t)(memory_used[1]));
        Wire1.write((uint8_t)(memory_used[2]));
        Wire1.write((uint8_t)(memory_used[3]));    
    }
    if(reg == 0x02)
    {
		float2char(memory_used,gx1);
        Wire1.write((uint8_t)memory_used[0]);
        Wire1.write((uint8_t)(memory_used[1]));
        Wire1.write((uint8_t)(memory_used[2]));
        Wire1.write((uint8_t)(memory_used[3]));   

        float2char(memory_used,gy1);
        Wire1.write((uint8_t)memory_used[0]);
        Wire1.write((uint8_t)(memory_used[1]));
        Wire1.write((uint8_t)(memory_used[2]));
        Wire1.write((uint8_t)(memory_used[3]));  
        
        float2char(memory_used,gz1);
        Wire1.write((uint8_t)memory_used[0]);
        Wire1.write((uint8_t)(memory_used[1]));
        Wire1.write((uint8_t)(memory_used[2]));
        Wire1.write((uint8_t)(memory_used[3]));        
    }
    if(reg == 0x03)
    {
        float2char(memory_used,mx1);
        Wire1.write((uint8_t)memory_used[0]);
        Wire1.write((uint8_t)(memory_used[1]));
        Wire1.write((uint8_t)(memory_used[2]));
        Wire1.write((uint8_t)(memory_used[3]));    

        float2char(memory_used,my1);
        Wire1.write((uint8_t)memory_used[0]);
        Wire1.write((uint8_t)(memory_used[1]));
        Wire1.write((uint8_t)(memory_used[2]));
        Wire1.write((uint8_t)(memory_used[3]));    
        
        float2char(memory_used,mz1);
        Wire1.write((uint8_t)memory_used[0]);
        Wire1.write((uint8_t)(memory_used[1]));
        Wire1.write((uint8_t)(memory_used[2]));
        Wire1.write((uint8_t)(memory_used[3]));         
    }

    if(reg == 0x04)
    {
        float2char(memory_used,pitch1);
        Wire1.write((uint8_t)memory_used[0]);
        Wire1.write((uint8_t)(memory_used[1]));
        Wire1.write((uint8_t)(memory_used[2]));
        Wire1.write((uint8_t)(memory_used[3]));   

        float2char(memory_used,roll1);
        Wire1.write((uint8_t)memory_used[0]);
        Wire1.write((uint8_t)(memory_used[1]));
        Wire1.write((uint8_t)(memory_used[2]));
        Wire1.write((uint8_t)(memory_used[3]));   

        float2char(memory_used,yaw1);
        Wire1.write((uint8_t)memory_used[0]);
        Wire1.write((uint8_t)(memory_used[1]));
        Wire1.write((uint8_t)(memory_used[2]));
        Wire1.write((uint8_t)(memory_used[3]));         
    }
    if(reg == 0x05)
    {	
		//磁力计校准在信号到达的时候将标志位置位，然后在loop中进行校准
		//关键在于怎么保存校准数据
		//将校准数据存到flash中，然后将flash中的某个地址认定为校准标志位
		//将其置位，表示flash中已经存有校准数据
		//每次上电的时候检查flash中的标志位，如果已经存有校准数据，则直接读取校准数据
		//否则不读取
        mag_calibrate_flag = 1;
    }

}
void setup_i2c()
{
    Wire1.begin(0x50);
    Wire1.onRequest(response_i2c);
    Wire1.onReceive(receiveEvent);
}

void myinthandler1()
{
  intFlag1 = true;
}

void myinthandler2()
{
  intFlag2 = true;
}

void setup()
{

  	Serial.begin(115200);
  	delay(300);
	Serial.print("begin setting wire");
 	// Wire.begin(); // set master mode, default on SDA/SCL for Ladybug   
  
	//Wire.setClock(400000); // I2C frequency at 400 kHz
	delay(300);
	// Wire1.begin(0x50);
	setup_i2c();
	
	// delay(300);
	// bmp_begin();
	// delay(300);
	// //MPU9250.I2Cscan(); // should detect BME280 at 0x77, MPU9250 at 0x71 
	// // Set up the interrupt pin, it's set as active high, push-pull
	// pinMode(PB8, INPUT);
	// /* Configure the MPU9250 */
	// // Read the WHO_AM_I register, this is a good test of communication
	// // Serial.println("MPU9250 9-axis motion sensor...");
	// uint8_t c = MPU9250.getMPU9250ID(MPU1);
	// Serial.print("MPU9250_1 "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x71, HEX);
	// delay(200);
	// if (c == 0x71) // WHO_AM_I should always be 0x71 for MPU9250, 0x73 for MPU9255 
	// {  
	// 	// Serial.println("MPU9250 are online...");
		
	// 	MPU9250.resetMPU9250(MPU1); // start by resetting MPU9250_1
		
	// 	MPU9250.SelfTest(MPU1, SelfTest); // Start by performing self test and reporting values
	// 	// Serial.println("Self Test for MPU9250 :");
	// 	// Serial.print("x-axis self test: acceleration trim within : "); Serial.print(SelfTest[0],1); Serial.println("% of factory value");
	// 	// Serial.print("y-axis self test: acceleration trim within : "); Serial.print(SelfTest[1],1); Serial.println("% of factory value");
	// 	// Serial.print("z-axis self test: acceleration trim within : "); Serial.print(SelfTest[2],1); Serial.println("% of factory value");
	// 	// Serial.print("x-axis self test: gyration trim within : "); Serial.print(SelfTest[3],1); Serial.println("% of factory value");
	// 	// Serial.print("y-axis self test: gyration trim within : "); Serial.print(SelfTest[4],1); Serial.println("% of factory value");
	// 	// Serial.print("z-axis self test: gyration trim within : "); Serial.print(SelfTest[5],1); Serial.println("% of factory value");
	// 	// delay(1000);

	// 	// get sensor resolutions, only need to do this once, same for both MPU9250s for now
	// 	aRes = MPU9250.getAres(Ascale);
	// 	gRes = MPU9250.getGres(Gscale);
	// 	mRes = MPU9250.getMres(Mscale);

	// 	// Comment out if using pre-measured, pre-stored offset biases
	// 	// 每次都校准加速度和角速度，必须平放着启动飞机
	// 	MPU9250.calibrateMPU9250(MPU1, gyroBias1, accelBias1); // Calibrate gyro and accelerometers, load biases in bias registers
	// 	//Serial.println("MPU1 accel biases (mg)"); Serial.println(1000.*accelBias1[0]); Serial.println(1000.*accelBias1[1]); Serial.println(1000.*accelBias1[2]);
	// 	//Serial.println("MPU1 gyro biases (dps)"); Serial.println(gyroBias1[0]); Serial.println(gyroBias1[1]); Serial.println(gyroBias1[2]);

	// 	delay(200); 
	
	// 	MPU9250.initMPU9250(MPU1, Ascale, Gscale, sampleRate); 

		
	// 	// Read the WHO_AM_I register of the magnetometer, this is a good test of communication
	// 	byte e = MPU9250.getAK8963CID(MPU1);  // Read WHO_AM_I register for AK8963
	// 	Serial.print("AK8963 1 "); Serial.print("I AM "); Serial.print(e, HEX); Serial.print(" I should be "); Serial.println(0x48, HEX);

	// 	delay(1000); 
		
	// 	// Get magnetometer calibration from AK8963 ROM
	// 	MPU9250.initAK8963Slave(MPU1, Mscale, Mmode, magCalibration1); Serial.println("AK8963 1 initialized for active data mode...."); // Initialize device 1 for active mode read of magnetometer
	// 	Serial.println("Calibration values for mag 1: ");
	// 	Serial.print("X-Axis sensitivity adjustment value "); Serial.println(magCalibration1[0], 2);
	// 	Serial.print("Y-Axis sensitivity adjustment value "); Serial.println(magCalibration1[1], 2);
	// 	Serial.print("Z-Axis sensitivity adjustment value "); Serial.println(magCalibration1[2], 2);

	
	// 	// Comment out if using pre-measured, pre-stored offset biases
	// 	//MPU9250.magcalMPU9250(MPU1, magBias1, magScale1);
	// 	//Serial.println("AK8963 1 mag biases (mG)"); Serial.println(magBias1[0]); Serial.println(magBias1[1]); Serial.println(magBias1[2]); 
	// 	//Serial.println("AK8963 1 mag scale (mG)"); Serial.println(magScale1[0]); Serial.println(magScale1[1]); Serial.println(magScale1[2]); 
		
	// 	//delay(2000); // add delay to see results before serial spew of data
		
		
	// 	attachInterrupt(PB8, myinthandler1, RISING);  // define interrupt for intPin output of MPU9250 1

	
	// 	}
	// 	else
	// 	{
	// 		Serial.print("Could not connect to MPU9250 1: 0x"); Serial.println(c, HEX);
	// 		while(1) ; // Loop forever if communication doesn't happen
	// 	}

	// 	//读取flash标志位，查看有没有磁偏角校准数据
		

	
}

void loop()
{  	
	Serial.println("The program is running...");
// 	if(mag_calibrate_flag == 1)
// 	{
// 		MPU9250.magcalMPU9250(MPU1, magBias1, magScale1);
// 		//校准完后将数据写入到flash中的固定位置，
// 		mag_calibrate_flag = 0;
// 		FLASH_Unlock();
// 		uint32_t flash_address = (uint32_t)0x0800d000 + (uint16_t)MAG_DATA_ADRESS<<8; //写入flash的地址
// 		FLASH_ErasePage(flash_address);    //页擦除
// 		unsigned char* p;
//    uint16_t data;
// 		p = (unsigned char *)&magBias1;
// 		for(int i=0;i<6;i=i+2)//有12个字节要写入到flash中
// 		{
// 			data = (uint16_t)(p[i]|p[i+1]<<8);	//?没问题吧,注意一下能合成不
// 			FLASH_ProgramHalfWord(flash_address+i, data); //写入2个字节的数据
// 		}
// 		flash_address = (uint32_t)0x0800d000 + (uint16_t)CALI_FLAG_ADRESS<<8; //写入flash的地址
// 		data = 0;
// 		FLASH_ProgramHalfWord(flash_address, data); 
// 		FLASH_Lock();	
// 	}

//    // If intPin1 goes high, either all data registers have new data
// 	if(intFlag1 == true) {   // On interrupt, read data
// 		intFlag1 = false;     // reset newData flag
		
// 		MPU9250.readMPU9250Data(MPU1, MPU9250Data1); // INT cleared on any read
	
// 		// Now we'll calculate the accleration value into actual g's
// 		ax1 = (float)MPU9250Data1[0]*aRes - accelBias1[0];  // get actual g value, this depends on scale being set
// 		ay1 = (float)MPU9250Data1[1]*aRes - accelBias1[1];   
// 		az1 = (float)MPU9250Data1[2]*aRes - accelBias1[2];  

// 		// Calculate the gyro value into actual degrees per second
// 		gx1 = (float)MPU9250Data1[4]*gRes;  // get actual gyro value, this depends on scale being set
// 		gy1 = (float)MPU9250Data1[5]*gRes;  
// 		gz1 = (float)MPU9250Data1[6]*gRes; 
  
// //    	if( MPU9250.checkNewMagData() == true) { // wait for magnetometer data ready bit to be set
// 		MPU9250.readMagData(MPU1, magCount1);  // Read the x/y/z adc values
	
// 		// Calculate the magnetometer values in milliGauss
// 		// Include factory calibration per data sheet and user environmental corrections
// 		mx1 = (float)magCount1[0]*mRes*magCalibration1[0] - magBias1[0];  // get actual magnetometer value, this depends on scale being set
// 		my1 = (float)magCount1[1]*mRes*magCalibration1[1] - magBias1[1];  
// 		mz1 = (float)magCount1[2]*mRes*magCalibration1[2] - magBias1[2];  
// 		mx1 *= magScale1[0];
// 		my1 *= magScale1[1];
// 		mz1 *= magScale1[2]; 
// 	//    }
	
  
//     for(uint8_t i = 0; i < 10; i++) { // iterate a fixed number of times per data read cycle
//     Now1 = micros();
//     deltat1 = ((Now1 - lastUpdate1)/1000000.0f); // set integration time by time elapsed since last filter update
//     lastUpdate1 = Now1;

//     sum1 += deltat1; // sum for averaging filter update rate
//     sumCount1++;

//     MadgwickQuaternionUpdate1(q,beta,deltat1,-ax1, +ay1, +az1, gx1*pi/180.0f, -gy1*pi/180.0f, -gz1*pi/180.0f,  my1,  -mx1, mz1);
//     }

//     /* end of MPU9250 1 interrupt handling */
//    }


//       // If intPin2 goes high, either all data registers have new data

  



//     // Read RTC

    
//     if(SerialDebug) {
//     Serial.print("ax1 = "); Serial.print((int)1000*ax1);  
//     Serial.print(" ay1 = "); Serial.print((int)1000*ay1); 
//     Serial.print(" az1 = "); Serial.print((int)1000*az1); Serial.println(" mg");
//     Serial.print("gx1 = "); Serial.print( gx1, 2); 
//     Serial.print(" gy1 = "); Serial.print( gy1, 2); 
//     Serial.print(" gz1 = "); Serial.print( gz1, 2); Serial.println(" deg/s");
//     Serial.print("mx1 = "); Serial.print( (int)mx1 ); 
//     Serial.print(" my1 = "); Serial.print( (int)my1 ); 
//     Serial.print(" mz1 = "); Serial.print( (int)mz1 ); Serial.println(" mG");

//     Serial.println("MPU9250 1");
//     Serial.print("q0 = "); Serial.print(q[0]);
//     Serial.print(" qx = "); Serial.print(q[1]); 
//     Serial.print(" qy = "); Serial.print(q[2]); 
//     Serial.print(" qz = "); Serial.println(q[3]); 


//     temperature1 = ((float) MPU9250Data1[3]) / 333.87f + 21.0f; // Gyro chip temperature in degrees Centigrade
//     // Print temperature in degrees Centigrade      
//     Serial.print("Gyro 1 temperature is ");  Serial.print(temperature1, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C
//    }               
   
//     a12 =   2.0f * (q[1] * q[2] + q[0] * q[3]);
//     a22 =   q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
//     a31 =   2.0f * (q[0] * q[1] + q[2] * q[3]);
//     a32 =   2.0f * (q[1] * q[3] - q[0] * q[2]);
//     a33 =   q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
//     pitch1 = -asinf(a32);
//     roll1  = atan2f(a31, a33);
//     yaw1   = atan2f(a12, a22);
//     pitch1 *= 180.0f / pi;
//     yaw1   *= 180.0f / pi; 
//     yaw1   += 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
//     if(yaw1 < 0) yaw1   += 360.0f; // Ensure yaw stays between 0 and 360
//     roll1  *= 180.0f / pi;
//     lin_ax1 = ax1 + a31;
//     lin_ay1 = ay1 + a32;
//     lin_az1 = az1 - a33;

//     if(SerialDebug) {
//     Serial.print("MPU9250 1 Yaw, Pitch, Roll: ");
//     Serial.print(yaw1, 2);
//     Serial.print(", ");
//     Serial.print(pitch1, 2);
//     Serial.print(", ");
//     Serial.println(roll1, 2);

//     Serial.print("Grav_x, Grav_y, Grav_z: ");
//     Serial.print(-a31*1000.0f, 2);
//     Serial.print(", ");
//     Serial.print(-a32*1000.0f, 2);
//     Serial.print(", ");
//     Serial.print(a33*1000.0f, 2);  Serial.println(" mg");
//     Serial.print("Lin_ax, Lin_ay, Lin_az: ");
//     Serial.print(lin_ax1*1000.0f, 2);
//     Serial.print(", ");
//     Serial.print(lin_ay1*1000.0f, 2);
//     Serial.print(", ");
//     Serial.print(lin_az1*1000.0f, 2);  Serial.println(" mg");
//     }
   
//     float height = bmp_readAltitude(1013.25);//高度数据
//     //Output for spreadsheet
//     //Serial.print(millis()); Serial.print(", ");
//     Serial.print(pitch1, 2); Serial.print(", "); Serial.print(roll1, 2);Serial.print(", "); Serial.print(yaw1, 2); Serial.print(", "); Serial.println(height, 2); 
    
} /* end of alarm handling */



//===================================================================================================================
//====== Set of useful functions
//===================================================================================================================



