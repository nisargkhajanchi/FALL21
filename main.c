include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif


//#include <TimerOne.h>
//#include <String.h>
//#include <RF24.h>
//#include <RF24Network.h>
#include <PinChangeInt.h>
#include <SPI.h>

//#include <avr/sleep.h>
//#include <Sleep_n0m1.h>
#include <SRAM_23LC.h>

//Sleep sleep;
MPU6050 mpu;
//RF24 radio(6,5);                    // CE = 10,CSN = 9 //nRF24L01(+) radio attached using Getting Started board 
//RF24Network network(radio); 

#define SPI_PERIPHERAL    SPI
#define CHIP_SELECT_PIN   9
#define BuzzPin           10
#define BoostEn           A3
#define LED               5
#define Alarm             8
#define ShutDownPin       7

// Network uses that radio
//const uint16_t this_node = 01;        // Address of our node in Octal format
//const uint16_t other_node = 00;       // Address of the other node in Octal format
//const uint16_t Rawdata_node = 02;       // Address of the other node in Octal format

//const uint64_t pipe = 0xE8E8F0F0E1LL;

SRAM_23LC SRAM(&SPI_PERIPHERAL, CHIP_SELECT_PIN, SRAM_23K640);
unsigned int SRAM_ADD =0, SRAMStrt_ADD= 0, SRAMEnd_ADD = 0;
//const unsigned long interval = 1; //ms  // How often to send 'hello world to the other unit
//unsigned long last_sent;             // When did we last send?
//unsigned long packets_sent;          // How many have we sent already

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[56]; // FIFO storage buffer

const char SPI_CS_PIN = 10;

//unsigned char DC_EN = 26;
unsigned char MPUInt = 03;

unsigned char ArraySize = 500 ;
volatile unsigned int ExpoDownCount = 0, ExpoUpCount = 0;
volatile unsigned char ExpoDownContStartAdd = 0, ExpoDownEndAdd =0,  ExpoDownMid0Add = 0,ExpoDownMid1Add = 0, ExpoDownMid2Add = 0;
volatile unsigned char HighPickContStartAdd = 0, HighPickEndAdd =0,  HighPickMid0Add = 0;
volatile float Yyam,Xyam,LineSlop,YyamMid0,YyamMid1,YyamMid2;
volatile float HighPickYyam,HighPickXyam,HighPickLineSlop,HighPickYyamMid0;
float ACCxyz_4AVG[2];
float Expo_Find = 0;

volatile float gForceX;
volatile float gForceY;
volatile float gForceZ;
volatile float rotX ;
volatile float rotY ;
volatile float rotZ;

volatile float rotXZ,rotXZ_MAX,rotXZ_S_Max;
volatile float ACCxyz ,ACCxyz_Pr,ACCxyz_Dif,Wxyz;
volatile float ACCxyz_S[200];
volatile float rotXZ_S[2];

unsigned int j=0,k=0;
unsigned char rotXZ_CntMax = 0,rotXZInterval = 4;
unsigned char ArrayNumber_MAX = 0;
unsigned char ArrayNumber_MIN = 0;
//unsigned char Timer1_Tick,ACnt = 0;

long debouncing_time = 15; //Debouncing Time in Milliseconds
volatile unsigned long last_micros;

volatile bool/* Gx_F, Gy_F, Gz_Y, RoX_F, RoY_F, RoZ_F,MPUInt_F,adcDone,Timer_F*/ StrAry_F , RawSample_F,SRAM_Read_F,mpuInterrupt;
volatile bool Pick_F = 0, Vally_F =0, ExpoDownImapactPick_F = 0,HighPick_F = 0,rotXZ_CntMAX_F = 0,RoundRecrd = 0 , PreRecord = 0, Alarm_F = 0, ShutDown_F = 0;

typedef union _data {
    float ACCxyz_Temp;
    char  temp[sizeof(float)];
} Cnvrt;
Cnvrt t;

void setup()
{
    // Initialize the digital pin as an output.

    Serial.begin(57600);      // Start the serial terminal
    Serial.println(F("System Intialize"));

    pinMode(LED, OUTPUT);
    pinMode(BoostEn, OUTPUT);
    // Pin 04 has an LED connected on most Arduino boards

    pinMode(Alarm, INPUT_PULLUP);  // Configure the pin as an input, and turn on the pullup resistor.
    // See http://arduino.cc/en/Tutorial/DigitalPins
    attachPinChangeInterrupt(Alarm, AlarmFunction, FALLING);

    pinMode(ShutDownPin, INPUT_PULLUP);  // Configure the pin as an input, and turn on the pullup resistor.
    // See http://arduino.cc/en/Tutorial/DigitalPins
    attachPinChangeInterrupt(ShutDownPin, ShutDown, FALLING);


    //sleep.pwrDownMode(); //set sleep mode
    //sleep.sleepDelay(10000); //sleep for: sleepTime short time
    //delay(20000); // Dealy 10 Second
    Serial.print(F("System Volt"));
    Serial.println( readVcc(), DEC );



    setupMPU();
    //Setup_MPU6050INT();

    //Timer1.attachInterrupt( timerIsr ); // attach the service routine here
    //Timer1.initialize(125);
    SPI.begin();
    //radio.begin();
    //radio.openWritingPipe(pipe);
    //network.begin(/*channel*/ 90, /*node address*/ this_node);
    SRAM.begin();

    SRAM.writeByte (00,0xAA);
    if(0xAA == SRAM.readByte(00))
        Serial.print(F("SRAM ok"));
    else
        Serial.print(F("SRAM error"));
    //CAN_INIT();
    Serial.print(F("System Done"));
    //set_sleep_mode(SLEEP_MODE_PWR_SAVE);
    sei(); // allow interrupts

}


void setupMPU(){

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)

    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    /* Serial.println(F("\nSend any character to begin DMP programming and demo: "));
     while (Serial.available() && Serial.read()); // empty buffer
     while (!Serial.available());                 // wait for data
     while (Serial.available() && Serial.read()); // empty buffer again*/

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN),MPU_dmpDataReady_Int , RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison

        packetSize = mpu.dmpGetFIFOPacketSize();
        Serial.print(F("DLPFMode :"));
        Serial.println(mpu.getDLPFMode());
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }


}


float Array_MAX_Sample(float Array_Max[])
{
    float Max =  Array_Max[0] ;
    for (int i=1; i < ArraySize; i++){
        if (Max < Array_Max[i]){
            Max = Array_Max[i];
            ArrayNumber_MAX = i;
        }
    }
    return Max;
}


float Array_MIN_Sample(float Array_Min[])
{
    float Min =  Array_Min[0];

    for (int i=1; i < ArraySize; i++){
        if (Min >  Array_Min[i]){
            Min =  Array_Min[i];
            ArrayNumber_MIN = i;
        }
    }
    return Min;
}

void recordAccelRegisters() {
    Wire.beginTransmission(0b1101000); //I2C address of the MPU
    Wire.write(0x3B); //Starting register for Accel Readings
    Wire.endTransmission();
    Wire.requestFrom(0b1101000,6); //Request Accel Registers (3B - 40)
    while(Wire.available() < 6);

    /*Rawdata.gForceX = (( Wire.read()<<8|Wire.read())/16384.0); //Store first two bytes into accelX
    Rawdata.gForceY = (( Wire.read()<<8|Wire.read())/16384.0); //Store middle two bytes into accelY
    Rawdata.gForceZ = (( Wire.read()<<8|Wire.read())/16384.0); //Store last two bytes into accelZ
    ACCxyz = sqrt((Rawdata.gForceX*Rawdata.gForceX) + (Rawdata.gForceY*Rawdata.gForceY) + (Rawdata.gForceZ*Rawdata.gForceZ));*/

    gForceX = (( Wire.read()<<8|Wire.read())/16384.0); //Store first two bytes into accelX
    gForceY = (( Wire.read()<<8|Wire.read())/16384.0); //Store middle two bytes into accelY
    gForceZ = (( Wire.read()<<8|Wire.read())/16384.0); //Store last two bytes into accelZ
    ACCxyz = sqrt((gForceX * gForceX) + (gForceY * gForceY) + (gForceZ * gForceZ));


}

void recordGyroRegisters() {
    Wire.beginTransmission(0b1101000); //I2C address of the MPU
    Wire.write(0x43); //Starting register for Gyro Readings
    Wire.endTransmission();
    Wire.requestFrom(0b1101000,6); //Request Gyro Registers (43 - 48)
    while(Wire.available() < 6);

    rotX = ((Wire.read()<<8|Wire.read())/131.0); //Store first two bytes into accelX
    rotY = ((Wire.read()<<8|Wire.read())/131.0); //Store middle two bytes into accelY
    rotZ = ((Wire.read()<<8|Wire.read())/131.0); //Store last two bytes into accelZ
    //Wxyz = sqrt((rotX*rotX) + (rotY*rotY) + (rotZ*rotZ));
    rotXZ = sqrt((rotX*rotX) + (rotZ*rotZ));
}
void MPU_Sleep(unsigned char MPU_6C )
{
    Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
    Wire.write(0x6C); //Accessing the register 6B - Power Management (Sec. 4.28)
    Wire.write(MPU_6C); //Setting SLEEP register to 0. (Required; see Note on p. 9)
    Wire.endTransmission();

    Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
    Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
    Wire.write(0b00101000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
    Wire.endTransmission();
}
void MPU_Sleep_Disable()
{
    Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
    Wire.write(0x6C); //Accessing the register 6B - Power Management (Sec. 4.28)
    Wire.write(0b10000000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
    Wire.endTransmission();

    Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
    Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
    Wire.write(0b00001000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
    Wire.endTransmission();
}
long readVcc() {
    long result;
    // Read 1.1V reference against AVcc
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
    delay(2); // Wait for Vref to settle
    ADCSRA |= _BV(ADSC); // Convert
    while (bit_is_set(ADCSRA,ADSC));
    result = ADCL;
    result |= ADCH<<8;
    result = 1125300L / result; // Back-calculate AVcc in mV
    return result;
}


void mpuReadData(){
    while (!mpuInterrupt && fifoCount < packetSize) {

        // other program behavior stuff here
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        //Serial.println(F("FIFO overflow!"));

        // otherwise, check for DMP data ready interrupt (this should happen frequently)
    }

    else if (mpuIntStatus & 0x02){
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        recordGyroRegisters();
        recordAccelRegisters();
    }


}
void loop()
{
    //float MAX,MIN,ACCxyz_MIN, ACCxyz_MAX, AVG ;
    //float ACCxyz_PS = 0;
    //unsigned char CntRept = 0;




    if( Alarm_F == 1){
        Alarm_F = 0;
        digitalWrite(BoostEn, HIGH);
        for(int x = 0; x<2; x++)
        {
            tone ( BuzzPin, 2000, 500);
            delay(800);
            noTone(BuzzPin);
            delay(200);
            tone ( BuzzPin, 2000, 500);
            delay(800);
            noTone(BuzzPin);
            delay(200);

        }
        digitalWrite(BoostEn, LOW);
    }

    while (!mpuInterrupt && fifoCount < packetSize) {

        // other program behavior stuff here
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        //Serial.println(F("FIFO overflow!"));

        // otherwise, check for DMP data ready interrupt (this should happen frequently)
    }

    else if (mpuIntStatus & 0x02){
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        recordGyroRegisters();
        recordAccelRegisters();
        ACCxyz_Dif = ACCxyz_Pr-ACCxyz;
        ACCxyz_Pr = ACCxyz ;
        t.ACCxyz_Temp = ACCxyz;
        //Serial.print(t.ACCxyz_Temp);
        //Serial.print(" ");
        //Serial.println(rotXZ);


        if( ShutDown_F == 0){

            analogWrite(LED, 220);
            SRAM.writeByte( SRAM_ADD ,t.temp[0]);
            SRAM_ADD++;
            SRAM.writeByte( SRAM_ADD ,t.temp[1] );
            SRAM_ADD++;
            SRAM.writeByte( SRAM_ADD ,t.temp[2] );
            SRAM_ADD++;
            SRAM.writeByte( SRAM_ADD ,t.temp[3] );
            SRAM_ADD++;
            Serial.print(rotXZ);
            Serial.print(F(" "));
            Serial.print(t.ACCxyz_Temp);
            Serial.print(F(" "));
            if(rotXZ_MAX <  rotXZ ){
                rotXZ_MAX = rotXZ;
                Serial.print(rotXZ_MAX);
                Serial.print(F(" "));
            }


            if((SRAM_ADD == 400) || (SRAM_ADD == 800) || (SRAM_ADD == 1200) || ( SRAM_ADD == 1600)){
                j=j+1;
                Serial.print(SRAM_ADD);
                Serial.print(" ");
                Serial.print(j);
                Serial.print(" ");

                if(j < 5)  {
                    if(rotXZ_MAX >= 30) {
                        if(SRAM_ADD == 1600){
                            RoundRecrd = 1;
                            Serial.print(F(" RoundRCD ON"));
                        }
                        if(SRAM_ADD==400) {
                            Serial.print(F(" PreRecord ON"));
                            PreRecord = 1;
                            SRAMEnd_ADD = 1200;
                            rotXZInterval = 3;
                        }
                        rotXZ_CntMax =  rotXZ_CntMax + 1;

                        Serial.print(rotXZ_CntMax);
                        Serial.print(" ");

                        Serial.print(rotXZ_MAX);
                        Serial.print(" ");

                        rotXZ_MAX = 0;
                    }
                }
            }
            if(j == rotXZInterval)
            {
                j =0;
                StrAry_F = 0;
                rotXZ_MAX = 0;


                if(( rotXZ_CntMax == 1 ) || ( rotXZ_CntMax == 2 )) {
                    if( RoundRecrd == 0){
                        rotXZ_CntMAX_F = 1 ;
                        rotXZ_CntMax =0;
                        SRAMStrt_ADD = 0;
                        SRAMEnd_ADD = 1600;
                        Serial.println(F("rotXZ flag ON"));

                    } else {
                        RoundRecrd = 0;
                        SRAMStrt_ADD = 800;
                        SRAMEnd_ADD  = 2400;
                        rotXZ_CntMAX_F = 1 ;
                        rotXZ_CntMax =0;
                        Serial.print(F(" While Loop Begin"));
                        while(SRAM_ADD <= SRAMEnd_ADD ) {
                            mpuReadData();
                            Serial.println(SRAM_ADD);
                            t.ACCxyz_Temp = ACCxyz;
                            Serial.print(SRAM_ADD);
                            Serial.print(" ");
                            Serial.print(ACCxyz);
                            SRAM.writeByte( SRAM_ADD ,t.temp[0]);
                            SRAM_ADD++;
                            SRAM.writeByte( SRAM_ADD ,t.temp[1] );
                            SRAM_ADD++;
                            SRAM.writeByte( SRAM_ADD ,t.temp[2] );
                            SRAM_ADD++;
                            SRAM.writeByte( SRAM_ADD ,t.temp[3] );
                            SRAM_ADD++;
                        }
                    }
                    if(PreRecord == 1) {

                        Serial.print(F(" PreRecord ON"));
                        PreRecord = 0;
                        rotXZ_CntMAX_F = 1 ;
                        rotXZ_CntMax =0;
                        Serial.print(F(" Adjusting Data"));
                        for(int i = SRAMStrt_ADD;i < SRAMEnd_ADD; i++) {
                            t.temp[0] = SRAM.readByte(i);
                            SRAM.writeByte( 1600 + i ,t.temp[0]);
                            i++;
                            t.temp[1] = SRAM.readByte(i);
                            SRAM.writeByte( 1600 + i ,t.temp[1]);
                            i++;
                            t.temp[2] = SRAM.readByte(i);
                            SRAM.writeByte( 1600 + i ,t.temp[2]);
                            i++;
                            t.temp[3] = SRAM.readByte(i);
                            SRAM.writeByte( 1600 + i ,t.temp[3]);
                        }
                        SRAMStrt_ADD = 1200;
                        SRAMEnd_ADD = 2800;
                        SRAM_ADD = 2804;
                    }
                } else {
                    SRAM_ADD = 0;
                    rotXZ_CntMax = 0;
                    rotXZInterval = 4;
                }
            }
            Serial.println(" ");

        }
    }
    if ((SRAM_ADD >= SRAMEnd_ADD) && ( rotXZ_CntMAX_F == 1)){
        //Serial.print("----------Printing------------");
        rotXZ_CntMAX_F = 0;
        for(int i = SRAMStrt_ADD;i < SRAM_ADD; i++) {


            t.temp[0] = SRAM.readByte(i);
            i++;
            t.temp[1] = SRAM.readByte(i);
            i++;
            t.temp[2] = SRAM.readByte(i);
            i++;
            t.temp[3] = SRAM.readByte(i);

            ACCxyz_4AVG[j] = t.ACCxyz_Temp;
            //Serial.println(ACCxyz_4AVG[j]);
            j = j+1;
            if( j > 1) {
                j=0;
                ACCxyz = (ACCxyz_4AVG[0]+ACCxyz_4AVG[1]/*+ACCxyz_4AVG[2]+ACCxyz_4AVG[3]*/)/2;
                ACCxyz_S[k] = ACCxyz ;
                //Serial.println(ACCxyz_S[k]);
                k++;
            }
        }

        Serial.println(k);
        if(k>=200){
            SRAM_ADD = 0;
            for(int i = 0; i< 200; i++){

                Serial.print(i);
                Serial.print("  ");

                Serial.print(ACCxyz_S[i],3);
                Serial.print(" ");

                if(ACCxyz_S[i] > ACCxyz_S[i+1]){
                    ExpoDownCount = ExpoDownCount + 1;

                    if (( ExpoDownCount == 1) &&(HighPick_F == 0)) {

                        ExpoDownContStartAdd = i;
                    }

                    if(( ExpoDownCount > 1) && (HighPick_F == 0) && (ExpoDownImapactPick_F == 0)){
                        ExpoUpCount = 0;
                        Pick_F = 0;
                    }

                    if(( ExpoUpCount > 1) && (HighPick_F == 1 ) && (ExpoDownImapactPick_F == 0) && (Vally_F == 0 )){

                        Vally_F = 1;
                        HighPickEndAdd  = i;

                        HighPickMid0Add = (HighPickContStartAdd + HighPickEndAdd)/2 ;

                        HighPickYyam = ( ACCxyz_S[HighPickEndAdd] - ACCxyz_S[HighPickContStartAdd]);
                        HighPickXyam = (HighPickEndAdd-HighPickContStartAdd)*0.01  ;
                        HighPickLineSlop = HighPickYyam/HighPickXyam ;

                        Serial.print(ExpoUpCount);
                        Serial.print(" ");
                        Serial.print(HighPickContStartAdd);
                        Serial.print(" ");
                        Serial.print(HighPickEndAdd);
                        Serial.print(" | ");

                        Serial.print(ACCxyz_S[HighPickContStartAdd], 3);
                        Serial.print(" ");
                        Serial.print(ACCxyz_S[HighPickEndAdd],3);
                        Serial.print(" | ");

                        Serial.print(HighPickYyam,3);
                        Serial.print(" ");
                        Serial.print(HighPickXyam,3);
                        Serial.print(" ");
                        Serial.print(HighPickLineSlop,3);
                        Serial.print(" | ");

                        ExpoUpCount = 0;
                        ExpoDownCount = 0;
                        HighPick_F = 1;

                        if(( HighPickLineSlop >= 12) && ( ACCxyz_S[HighPickEndAdd] > 2.5)) {

                            Serial.print(F( " Impact "));

                            HighPick_F = 0;
                            ExpoDownImapactPick_F = 1;

                            digitalWrite(BoostEn, HIGH);
                            while(Alarm_F == 0){
                                for(int i=700;i<800;i++){
                                    tone(BuzzPin,i);
                                    delay(15);
                                }
                                for(int i=800;i>700;i--){
                                    tone(BuzzPin,i);
                                    delay(15);
                                }
                            }
                            digitalWrite(BoostEn, LOW);
                            Alarm_F = 0;

                        }
                    }
                }

                else if (ACCxyz_S[i] < ACCxyz_S[i+1]) {
                    ExpoUpCount = ExpoUpCount + 1;

                    if(( ExpoUpCount > 2) && ( Pick_F == 0 ) && (HighPick_F == 0) ){

                        Pick_F = 1;

                        ExpoDownEndAdd  = i-2;

                        ExpoDownMid0Add = (ExpoDownContStartAdd + ExpoDownEndAdd)/2 ;
                        ExpoDownMid1Add = (ExpoDownContStartAdd+ExpoDownMid0Add)/2;
                        ExpoDownMid2Add = (ExpoDownMid0Add+ExpoDownEndAdd)/2;

                        Yyam = ( ACCxyz_S[ExpoDownEndAdd] - ACCxyz_S[ExpoDownContStartAdd]);
                        Xyam = (ExpoDownEndAdd-ExpoDownContStartAdd)*0.01  ;
                        LineSlop = Yyam/Xyam ;

                        YyamMid0 = ((ACCxyz_S[ExpoDownEndAdd]) + (ACCxyz_S[ExpoDownContStartAdd]))/2;
                        YyamMid1 = ((ACCxyz_S[ExpoDownContStartAdd]) + YyamMid0)/2;
                        YyamMid2 = ((ACCxyz_S[ExpoDownEndAdd]) + YyamMid0)/2;

                        Serial.print(ExpoDownCount);
                        Serial.print(" ");
                        Serial.print(ExpoDownContStartAdd);
                        Serial.print(" ");
                        Serial.print(ExpoDownEndAdd);
                        Serial.print(" | ");

                        Serial.print(ACCxyz_S[ExpoDownContStartAdd], 3);
                        Serial.print(" ");
                        Serial.print(ACCxyz_S[ExpoDownEndAdd],3);
                        Serial.print(" | ");

                        Serial.print(Yyam,3);
                        Serial.print(" ");
                        Serial.print(Xyam);
                        Serial.print(" ");
                        Serial.print(LineSlop);
                        Serial.print(" | ");

                        Serial.print(ExpoDownMid0Add);
                        Serial.print(" ");
                        Serial.print(ExpoDownMid1Add);
                        Serial.print(" ");
                        Serial.print(ExpoDownMid2Add);
                        Serial.print(" | ");

                        Serial.print(ACCxyz_S[ExpoDownMid0Add],3);
                        Serial.print(" ");
                        Serial.print(ACCxyz_S[ExpoDownMid1Add],3);
                        Serial.print(" ");
                        Serial.print(ACCxyz_S[ExpoDownMid2Add],3);
                        Serial.print(" | ");

                        Serial.print(YyamMid0,3);
                        Serial.print(" ");
                        Serial.print(YyamMid1,3);
                        Serial.print(" ");
                        Serial.print(YyamMid2,3);
                        Serial.print(" | ");

                        Serial.print(ACCxyz_S[ExpoDownMid0Add] - YyamMid0,3);
                        Serial.print(" ");
                        Serial.print(ACCxyz_S[ExpoDownMid1Add] - YyamMid1,3);
                        Serial.print(" ");
                        Serial.print(ACCxyz_S[ExpoDownMid2Add] - YyamMid2,3);
                        Serial.print(" | ");

                        YyamMid0 = ACCxyz_S[ExpoDownMid0Add] - YyamMid0;
                        YyamMid1 = ACCxyz_S[ExpoDownMid1Add] - YyamMid1;
                        YyamMid2 = ACCxyz_S[ExpoDownMid2Add] - YyamMid2;

                        if((Yyam < -0.2) && ( ExpoDownCount >= 12 ) && ( ExpoDownCount <= 50) && (LineSlop <= -0.5) && (LineSlop >= -3.0 )) {
                            if(/*( YyamMid0 > 0 ) &&*/ ( YyamMid0 > 0 )) {

                                Serial.println(F(" "));
                                Serial.print(F("It is Expo Down---->"));
                                Serial.println(F("HighPickFind trigger"));
                                //RadioData  = 10.0 ;
                                //radio.write(&RadioData,sizeof(RadioData));
                                HighPick_F = 1;  // Pick count Flag ON.
                                ExpoUpCount = 0; // Just reseted the up count..
                                i = i-1;
                            }
                        }
                        //radio.write(&LineSlop,sizeof(LineSlop));
                        ExpoDownCount   = 0;
                    }

                    if (( ExpoUpCount == 1) && (HighPick_F == 1)){

                        Serial.print (F(" HighPickFind Started "));
                        HighPickContStartAdd = i;
                        ExpoDownCount  = 0;
                    }
                    if(( ExpoUpCount > 1) && (HighPick_F == 1) && (ExpoDownImapactPick_F == 0)){
                        Vally_F = 0;
                    }
                }
                Serial.println(" ");
            }
        }
        k=0;
        j=0;
        ExpoDownImapactPick_F =0;
        Pick_F =0;
        SRAMStrt_ADD = 0;
        SRAMEnd_ADD = 1600;
        HighPick_F = 0;
        ExpoDownCount = 0;
        ExpoUpCount =0;
        ExpoDownContStartAdd = 0;
        t.ACCxyz_Temp = 0;
        Expo_Find = 0;
        rotXZInterval = 4;
        for(int x=0; x<3000;x++){
            SRAM.writeByte(x , 0xff);
        }
    }
}

/// --------------------------
/// Custom ISR Timer Routine
/// --------------------------
/*void timerIsr()
{     
    int ACCxyz_raw_temp;
   
    RawSample_F =1;
    recordAccelRegisters();
      //recordGyroRegisters();
    //if(SRAM_ADD < 1000)
    //{
    
    /* bool ok = network.write(header,&ACCxyz_Raw_Back,sizeof(ACCxyz_Raw_Back));
      if (ok)
      Serial.println("RF ok.");
      else
      Serial.println("RF failed.");
     
     char High_Byte = (char)highByte(ACCxyz_raw_temp);
     char Low_Byte = (char)lowByte(ACCxyz_raw_temp );

     SRAM.writeByte(SRAM_ADD, High_Byte);
     SRAM_ADD++;
     SRAM.writeByte(SRAM_ADD, Low_Byte);
     SRAM_ADD++;*/

//}

/*if(SRAM_ADD == 999)
{
  SRAM_ADD = 0;
 for(int i = 0;i< 1000;i++)
 {
   char High_Byte =  SRAM.readByte (i);
   i++;
   char Low_Byte =  SRAM.readByte (i);
   ACCxyz_Raw_Back = (((High_Byte * 256) + Low_Byte)/16384);

 }
}


  //MPUInt_F = 1;
 // Toggle LED
// digitalWrite( LED, digitalRead( LED ) ^ 1 );
}*/

void MPU_dmpDataReady_Int()
{

    mpuInterrupt = true;
    //Serial.print("Int");


}
void AlarmFunction() {
    if( ShutDown_F == 0){
        Alarm_F = 1;
    }

}

void ShutDown() {

    if((long)(micros() - last_micros) >= debouncing_time * 1000) {
        if( ShutDown_F == 0){
            ShutDown_F = 1;
            digitalWrite(LED, HIGH);
        } else {
            ShutDown_F = 0;
        }
    }
    last_micros = micros();
}
  