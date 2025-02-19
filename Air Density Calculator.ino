/*
 *
 * Project: Ghost
 * Experiment: 1
 * Start date: 1/28/2023
 * Version: 1.0 (2/5/2023)
 * Air Density Calculator
 * Components: Arduino R3 (MCU), M5Stack ENV III (SENSOR)
 * Purpose: Calculate Air Density based on temperature, humidity, and air pressure measurements
 * Parameters:
 * Temperature [-40,120]˚C; Highest Accuracy [0,60] ±0.2˚C; working temp [0,40]˚C 
 * Humidity 10-90%RH ±2%
 * Air Pressure [300,1100]hPa; 0.06Pa resolution; ±3.9Pa
 * Calculation Error: ±#%
 * SHT30:0x44 (temperature and humidity)
 * QMP6988:0x70 (air pressure)
 * Programming Goal:
 * 1. Initiate and run communication between MCU and SENSOR
 * 2. Measure temperautre, humidity. and air pressure
 * 3. Calculate Air density
 * 4. Communicate air density to monitor
 * 
 */ 
//QMP6988
//QMP6988 Implementation Registry List
#define QMP6988 0x70    //low mode
#define TEMP_TXD0 0xfc  //temp data[8:1] in 24bit
#define TEMP_TXD1 0xfb  //temp data[16:9] in 24 bit
#define TEMP_TXD2 0xfa  //temp data[24:17] in 24bit
#define PRESS_TXD0 0xf9 //pressure data[8:1] in 24 bit
#define PRESS_TXD1 0xf8 //pressure data[16:9] in 24 bit
#define PRESS_TXD2 0xf7 //pressure data[24:17] in 24 bit
#define IO_SETUP 0xf5
/*  b[7:5] t_standby[2:0]: standby time setting
 *  (000,1ms)(001,5ms)(010,50ms)(011,250ms)(100,500ms)(101,1s)(110,2s)(111,4s)
 *  b[2] spi3_sdim:select output type of SDI terminal
 *  b[0] spi3w: SPI mode setting (3 wire or 4)
 */
#define CTRL_MEAS 0xf4
/*  b[7:5] temp_average[2:0] Temperature average timing
 *  (000,skip)(001,1)(010,2)(011,4)(100,8)(101,16)(110,32)(111,64)
 *  b[4:2] pressure_average[2:0] Pressure average timing
 *  (000,skip)(001,1)(010,2)(011,4)(100,8)(101,16)(110,32)(111,64)
 *  b[1:0] power_mode[1:0] power mode setting
 *  (00,sleep)(01,force)(10,force)(11,normal)
 */
#define DEVICE_STAT 0xf3
/*  b[3] measure: status of measurement
 *  (0,finish a measurement waiting for next measurement)(1,on a measurement - waiting for finish the data store)
 *  b[0] otp_update: status of OTP data access
 *  (0,no accessing OTP data)(1,while accessing OTP data)
 */
#define I2C_SET 0xf2
/*  b[2:0] master code setting at I2C HS mode
 *  (000,0x08)(001,0x09)(010,0x0a)(011,0x0b)(100,0x0c)(101,0x0d)(110,0x0e)(111,0x0f)
 */
#define IIR_CNT 0xf1
/*  b[2:0] IIR filter coefficient setting
 *  (000,off)(001,n=2)(010,n=4)(011,n=8)(100,n=16)(101++,n=32)
 */
#define RESET 0xe0          //when inputting e6h a soft(ware) reset will occur
#define CHIP_ID 0xd1        //chip id: 5ch (0x5c)

//The following are compensation coefficients
#define COE_b00_a0_ex 0xb8  //b[7:4] b00 b[3:0] a0
#define COE_a2_0 0xb7       //a2[7:0]
#define COE_a2_1 0xb6       //a2[15:8]
#define COE_a1_0 0xb5       //a1[7:0]
#define COE_a1_1 0xb4       //a1[15:8]
#define COE_a0_0 0xb3       //a0[11:4]
#define COE_a0_1 0xb2       //a0[19:12]
#define COE_bp3_0 0xb1      //bp3[7:0]
#define COE_bp3_1 0xb0      //bp3[15:8]
#define COE_b21_0 0xaf      //b21[7:0]
#define COE_b21_1 0xae      //b21[15:8]
#define COE_b12_0 0xad      //b12[7:0]
#define COE_b12_1 0xac      //b12[15:8]
#define COE_bp2_0 0xab      //bp2[7:0]
#define COE_bp2_1 0xaa      //bp2[15:8]
#define COE_b11_0 0xa9      //b11[7:0]
#define COE_b11_1 0xa8      //b11[15:8]
#define COE_bp1_0 0xa7      //bp1[7:0]
#define COE_bp1_1 0xa6      //bp1[15:8]
#define COE_bt2_0 0xa5      //bt2[7:0]
#define COE_bt2_1 0xa4      //bt2[15:8]
#define COE_bt1_0 0xa3      //bt1[7:0]
#define COE_bt1_1 0xa2      //bt1[15:8]
#define COE_b00_0 0xa1      //b00[11:4]
#define COE_b00_1 0xa0      //b00[19:12]

//compensation coefficient variables
byte coe[25] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};//zeroed to clean the memory location
/* mapping coe to it's respective compensation measure
 * {b00_1, b00_0, bt1_1, bt1_0, bt2_1, bt2_0, bp1_1, bp1_0, b11_1, b11_0, bp2_1, bp2_0,
 *b12_1, b12_0, b21_1, b21_0, bp3_1, bp3_0, a0_1, a0_0, a1_1, a1_0, a2_1, a2_0, b00_a0_ex};
 */

// raw data variables
byte temp0[3] = {0,0,0};
byte press[3] = {0,0,0};

//concatenated temperature data variables
byte a0_ex = 0;
double a1 = 0;
double a2 = 0;
double a0 = 0;
double tr = 0; //double
double dt = 0; //double

//concatenated pressure data variables
double dp = 0; //double
double b00 = 0;
double pr = 0; //double
double bt1 = 0;
double bp1 = 0;
double b11 = 0;
double bt2 = 0;
double bp2 = 0;
double b12 = 0;
double b21 = 0;
double bp3 = 0;
double b00_ex = 0;
//end QMP6988
#include <Wire.h> //load i2c protocol library
#include <SHTSensor.h> //load sht3x library
SHTSensor sht;
float temp;
float rh;
int loopref = 0; //count the number of cycles the system has run

void setup() {
Wire.begin(); //begin i2c protocol as a controller
Serial.begin(9600);
delay(1000);
if (sht.init()){
  Serial.print("init() success\n");
}
else{
  Serial.print("sht.init() failed\n");
  sht.setAccuracy(SHTSensor::SHT_ACCURACY_MEDIUM);
}


}

void loop() {
  loopref++;
  Serial.print("Cycle: ");
  Serial.println(loopref);
  if (sht.readSample()) {
  }
  else {
   Serial.print("Error in readSample()\n");
  }
temp = (sht.getTemperature()*9/5)+32;
Serial.print("Temperature: ");
Serial.print(temp);
Serial.println("°F");
float rh = sht.getHumidity();
Serial.print("Humidity: ");
Serial.print(rh);
Serial.println("%");

//1. Configure IO mode setting.
Wire.beginTransmission(QMP6988);
Wire.write(IO_SETUP);
Wire.write(0x00);
Wire.endTransmission();

//2. Read compensation coefficients - sufficient to perform once after POR
Wire.beginTransmission(QMP6988);
Wire.write(COE_b00_1);
Wire.endTransmission();
Wire.requestFrom(QMP6988, 25);
for(int i = 0; i < 25; i++){
  coe[i] = Wire.read();
}
//3. Configure average times and power mode
Wire.beginTransmission(QMP6988);
Wire.write(CTRL_MEAS);
Wire.write(0b11111101); //temp average 1 (001xxxxx), pressure average 1 (xxx001xx), forced power mode (xxxxxx01) or (xxxxxx10)
Wire.endTransmission(); //011~111 will receive complete 24-bit data accuracy

//4. Read raw temp data
Wire.beginTransmission(QMP6988);
Wire.write(TEMP_TXD2);
Wire.endTransmission();
Wire.requestFrom(QMP6988,3);
for(int i = 0; i < 3; i++){
  temp0[i] = Wire.read();
}    

//5. Read raw pressure data
Wire.beginTransmission(QMP6988);
Wire.write(PRESS_TXD2);
Wire.endTransmission();
Wire.requestFrom(QMP6988, 3);
for(int i = 0; i < 3; i++){
  press[i] = Wire.read();
}

/* 6. Calculate compensated temperature celsius
 * formula: Tr = a0+a1Dt+a2Dt^2
 * compensation coefficient formula: K = A + ((S * OTP)/32767)
 * compensation coefficient formula for a0: K = OTP/16
 * raw temp data
 */
dt = ((long(temp0[0]) << 16) | (int(temp0[1]) << 8) | temp0[2]) - pow(2,23);
//a0
a0_ex = coe[24] &= 0b00001111;
a0 = ((long(coe[18])<<12) + (int(coe[19]) << 4) + a0_ex);
a0 = a0 / 16;
//a1
a1 = (int(coe[20]) << 8) | coe[21];
a1 = -6.03 * pow(10,-3) + ((4.30 * pow(10,-4) * a1)/32767);
//a2
a2 = (int(coe[22]) << 8) | coe[23];
a2 = -1.90 * pow(10,-11) + ((1.20 * pow(10,-10) * a2)/32767);
//calculate tr
tr = a0 + a1 * dt + a2 * sq(dt);

/* 7. Calculate compensated pressure kPa
 * formula: Pr = b00+bt1Tr+bp1Dp+b11TrDp+bt2Tr^2+bp2Dp^2+b12DpTr^2+b21Dp^2Tr+bp3Dp^3
 * compensation coefficient formula: K = A + ((S * OTP)/32767)
 * compensation coefficient formula for b00: K = OTP/16
 * raw pressure data
 */
dp = (long(press[0]) << 16) + (int(press[1]) << 8) + press[2] - pow(2,23);
//b00
b00 = (long(coe[0]) <<12) + (int(coe[1]) << 4) + (coe[24] >> 4);
b00 = b00 /16;
//bt1
bt1 = (int(coe[2]) << 8) + coe[3];
bt1 = 1.00 * pow(10,-1) + ((9.10 * pow(10,-2) * bt1)/32767);
//bp1
bp1 = (int(coe[6]) <<8) + coe[7];
bp1 = 3.30 * pow(10,-2) + ((1.90 * pow(10,-2) * bp1)/32767);
//b11
b11 = (int(coe[8]) << 8) + coe[9];
b11 = 2.10 * pow(10,-7) + ((1.40 * pow(10,-7) * b11)/32767);
//bt2
bt2 = (int(coe[4]) << 8) + coe[5];
bt2 = 1.20 * pow(10,-8) + ((1.20 * pow(10,-6) * bt2)/32767);
//bp2
bp2 = (int(coe[10]) << 8) | coe[11];
bp2 = 6.30 * pow(10,-10) + ((3.50 * pow(10,-10) * bp2)/32767); //errata the chart labeled A = --6.30E-10
//b12
b12 = (int(coe[13]) << 8) + coe[14];
b12 = 2.90 * pow(10,-13) + ((7.60 * pow(10,-13) * b12)/32767);
//b21
b21 = (int(coe[14]) << 8) + coe[15];
b21 = 2.10 * pow(10,-15) + ((1.20 * pow(10,-14) * b21)/32767);
//bp3
bp3 = (int(coe[16]) << 8) + coe[17];
bp3 = 1.30 * pow(10,-16) + ((7.90 * pow(10,-17) * bp3)/32767);
//calculation Pr = b00+bt1Tr+bp1Dp+b11TrDp+bt2Tr^2+bp2Dp^2+b12DpTr^2+b21Dp^2Tr+bp3Dp^3
pr = b00 + bt1 * tr + bp1 * dp + b11 * tr * dp + bt2 * sq(tr) + bp2 * sq(dp) + b12 * dp * sq(tr) + bp3 * pow(dp,3);
double pa = pr/5; // divided by 5 because readings were 5x what they should be within approx. 10% accuracy i.e. +/-10% based off tests
float kpa = pr/1000;
Serial.print("Air Pressure: ");
Serial.print(kpa, 2);
Serial.println(" kPa");

/* Air Density Calculation
 * 1.
 * P1 = 6.1078*10*(7.5Tc)/(Tc+237.3)
 * Tc:=Temperature celsius
 * P1:= saturation vapor pressure
 * 2.
 * Pv := P1*RH
 * rh:= relative humidity
 * pv:= actual vapor pressure in Pa 
 * 3.
 * Pd=P-Pv
 * P:=total air pressure
 * Pd:=dry air pressure in Pa
 * 4.
 * rho=(Pd/(RdTk))+(Pv/(RvTk))
 * Tk:=temperature in kelvins
 * Rd:=specific gas constant for dry air in J/kgK
 * Rv:=specific gas constant for water vapor in J/kgK
 * Rd=287.058 J/kgK
 * Rv=461.495 J/kgK
 */

//1.
float p1 = 6.1078 * 10 * ((7.5 * temp)/(temp+273.15)); //edit replaced 237.3 with 273.15 correct conversion from C to K
//2.
float pv = p1 * rh;
//3.
float pd = pa - pv;
//4.
float rd = 287.058;
float rv = 461.495;
float rho = (pd/(rd*(temp+273.15)))+(pr/(rv*(temp+273.15)));
Serial.print("Air Density: ");
Serial.print(rho,3);
Serial.println(" kg/m^3");
Serial.println();
delay(10000);


}