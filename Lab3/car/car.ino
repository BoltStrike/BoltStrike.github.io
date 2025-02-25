#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU

//#define USE_SPI       // Uncomment this to use SPI


#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"
#include <ArduinoBLE.h>
#include <Wire.h>
#include "SparkFun_VL53L1X.h" //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X

//Necessary shutdown pin for xshut
#define SHUTDOWN_PIN 0
#define INTERRUPT_PIN 3 //not used

SFEVL53L1X tof1;
SFEVL53L1X tof2(Wire, SHUTDOWN_PIN, INTERRUPT_PIN);

//////////// BLE UUIDs ////////////
#define BLE_UUID_TEST_SERVICE "350447e4-252a-49ab-b5a3-46cfae2701aa"

#define BLE_UUID_RX_STRING "9750f60b-9c9c-4158-b620-02ec9521cd99"

#define BLE_UUID_TX_FLOAT "27616294-3063-4ecc-b60b-3470ddef2938"
#define BLE_UUID_TX_STRING "f235a225-6735-4d73-94cb-ee5dfce9ba83"
//////////// BLE UUIDs ////////////


#define SERIAL_PORT Serial

#define SPI_PORT SPI // Your desired SPI port.       Used only when "USE_SPI" is defined
#define CS_PIN 2     // Which pin you connect CS to. Used only when "USE_SPI" is defined

#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 1

#ifdef USE_SPI
ICM_20948_SPI myICM; // If using SPI create an ICM_20948_SPI object
#else
ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object
#endif

//////////// Global Variables ////////////
BLEService testService(BLE_UUID_TEST_SERVICE);

BLECStringCharacteristic rx_characteristic_string(BLE_UUID_RX_STRING, BLEWrite, MAX_MSG_SIZE);

BLEFloatCharacteristic tx_characteristic_float(BLE_UUID_TX_FLOAT, BLERead | BLENotify);
BLECStringCharacteristic tx_characteristic_string(BLE_UUID_TX_STRING, BLERead | BLENotify, MAX_MSG_SIZE);

// RX
RobotCommand robot_cmd(":|");

// TX
EString tx_estring_value;
float tx_float_value = 0.0;

long interval = 500;
static long previousMillis = 0;
unsigned long currentMillis = 0;

float pitch_a;
float roll_a;
int last_time;
float dt;
float pitch_g = 0;
float roll_g = 0;
float yaw_g = 0;

int count = 0;

enum CommandTypes
{
    PING,
    GET_IMU_DATA,
    GET_TOF_DATA,
    START_COLLECTION,
};

BLEDevice central;

#define DATA_LENGTH 128
int iter = 0;
int tof_iter1 = 0;
int tof_iter2 = 0;
int collecting = 0;
float imu_data[DATA_LENGTH][9];
int timestamps[DATA_LENGTH];
int tof[DATA_LENGTH][2];
int tofstamps[DATA_LENGTH];

void
handle_command()
{   
    // Set the command string from the characteristic value
    robot_cmd.set_cmd_string(rx_characteristic_string.value(),
                             rx_characteristic_string.valueLength());

    bool success;
    int cmd_type = -1;

    // Get robot command type (an integer)
    /* NOTE: THIS SHOULD ALWAYS BE CALLED BEFORE get_next_value()
     * since it uses strtok internally (refer RobotCommand.h and 
     * https://www.cplusplus.com/reference/cstring/strtok/)
     */
    success = robot_cmd.get_command_type(cmd_type);

    // Check if the last tokenization was successful and return if failed
    if (!success) {
        return;
    }

    // Handle the command type accordingly
    switch (cmd_type) {
        /*
         * Write "PONG" on the GATT characteristic BLE_UUID_TX_STRING
         */
        case PING:
        {
            tx_estring_value.clear();
            tx_estring_value.append("PONG");
            tx_characteristic_string.writeValue(tx_estring_value.c_str());

            Serial.print("Sent back: ");
            Serial.println(tx_estring_value.c_str());

            break;
        }
        case GET_IMU_DATA:
        {
          for (int i = 0; i < DATA_LENGTH; i++){
            tx_estring_value.clear();
            tx_estring_value.append("pitch:");
            tx_estring_value.append(imu_data[i][0]);
            tx_estring_value.append("roll:");
            tx_estring_value.append(imu_data[i][1]);
            tx_estring_value.append("filtp:");
            tx_estring_value.append(imu_data[i][2]);
            tx_estring_value.append("filtr:");
            tx_estring_value.append(imu_data[i][3]);
            tx_estring_value.append("gp:");
            tx_estring_value.append(imu_data[i][4]);
            tx_estring_value.append("gr:");
            tx_estring_value.append(imu_data[i][5]);
            tx_estring_value.append("gy:");
            tx_estring_value.append(imu_data[i][6]);
            tx_estring_value.append("cp:");
            tx_estring_value.append(imu_data[i][7]);
            tx_estring_value.append("cr:");
            tx_estring_value.append(imu_data[i][8]);
            tx_estring_value.append("T:");
            tx_estring_value.append(timestamps[i]);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
          }
          Serial.println(count);
          break;
        }
        case GET_TOF_DATA:
        {
          for (int i = 0; i < DATA_LENGTH; i++){
            tx_estring_value.clear();
            tx_estring_value.append("tof1:");
            tx_estring_value.append(tof[i][0]);
            tx_estring_value.append("tof2:");
            tx_estring_value.append(tof[i][1]);
            tx_estring_value.append("T:");
            tx_estring_value.append(tofstamps[i]);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
          }
          break;
        }
        case START_COLLECTION:
        {
          collecting = 1;
          iter = 0;
          tof_iter1 = 0;
          tof_iter2 = 0;
          count = 0;
          break;
        }
        default:
            Serial.print("Invalid Command Type: ");
            Serial.println(cmd_type);
            break;
      }
}

void setup()
{
  //Blink 3 times on boot
  pinMode(LED_BUILTIN, OUTPUT);

  for(int i = 0; i<3; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW); 
    delay(500);
  }


  SERIAL_PORT.begin(115200);
  while (!SERIAL_PORT)
  {
  };

  BLE.begin();

  // Set advertised local name and service
  BLE.setDeviceName("Artemis BLE");
  BLE.setLocalName("Artemis BLE");
  BLE.setAdvertisedService(testService);

  // Add BLE characteristics
  testService.addCharacteristic(tx_characteristic_float);
  testService.addCharacteristic(tx_characteristic_string);
  testService.addCharacteristic(rx_characteristic_string);

  // Add BLE service
  BLE.addService(testService);

  Serial.print("Advertising BLE with MAC: ");
  Serial.println(BLE.address());

  BLE.advertise();

#ifdef USE_SPI
  SPI_PORT.begin();
#else
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
#endif

  //myICM.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  bool initialized = false;
  while (!initialized)
  {

#ifdef USE_SPI
    myICM.begin(CS_PIN, SPI_PORT);
#else
    myICM.begin(WIRE_PORT, AD0_VAL);
#endif

    SERIAL_PORT.print(F("Initialization of the sensor returned: "));
    SERIAL_PORT.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      SERIAL_PORT.println("Trying again...");
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }

  //Initialize tof sensors
  Wire.begin();

  pinMode(SHUTDOWN_PIN, OUTPUT);
  digitalWrite(SHUTDOWN_PIN, LOW); //Turn off 2nd sensor with xshut
  tof1.setI2CAddress(0x30); //give 1st sensor different address
  digitalWrite(SHUTDOWN_PIN, HIGH); // Trun on second sensor

  Serial.begin(115200);
  Serial.println("VL53L1X Qwiic Test");

  if (tof1.begin() != 0) //Begin returns 0 on a good init
  {
    Serial.println("Sensor1 failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }
  if (tof2.begin() != 0) //Begin returns 0 on a good init
  {
    Serial.println("Sensor2 failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }
  Serial.println("TOF Sensors online!");
  tof1.setDistanceModeShort();
  tof2.setDistanceModeShort();
}

void
write_data()
{
    currentMillis = millis();
    if (currentMillis - previousMillis > interval) {

        tx_float_value = tx_float_value + 0.5;
        //tx_characteristic_float.writeValue(tx_float_value);

        if (tx_float_value > 10000) {
            tx_float_value = 0;
            
        }

        previousMillis = currentMillis;
    }
}

void
read_data()
{
    // Query if the characteristic value has been written by another BLE device
    if (rx_characteristic_string.written()) {
        handle_command();
    }
}

void loop()
{

  if (!central) {
    central = BLE.central();
    if (central) {
      Serial.print("Connected to: ");
      Serial.println(central.address());
    }

  }
  else {
      // While central is connected
      if (central.connected()) {
          // Send data
          write_data();

          // Read data
          read_data();
      }
  }

  if(collecting) {
      //collect TOF data
      tof1.startRanging(); //Write configuration bytes to initiate measurement
      tof2.startRanging(); //Write configuration bytes to initiate measurement
      if (tof1.checkForDataReady() && tof_iter1 < DATA_LENGTH) {
        tof[tof_iter1][0] = tof1.getDistance(); //Get the result of the measurement from the sensor
        tof1.clearInterrupt();
        tof1.stopRanging();
        tofstamps[tof_iter1] = millis();
        tof_iter1 ++;
      }
      if (tof2.checkForDataReady() && tof_iter2 < DATA_LENGTH) {
        tof[tof_iter2][1] = tof2.getDistance(); //Get the result of the measurement from the sensor
        tof2.clearInterrupt();
        tof2.stopRanging();
        tof_iter2 ++;
      }


  }

  if (myICM.dataReady())
  {
    myICM.getAGMT();         // The values are only updated when you call 'getAGMT'
                             //    printRawAGMT( myICM.agmt );     // Uncomment this to see the raw values, taken directly from the agmt structure
    //printScaledAGMT(&myICM); // This function takes into account the scale settings from when the measurement was made to calculate the values with units
    


    //Collect IMU Data for transfer
    if(collecting && iter<DATA_LENGTH) {
      timestamps[iter] = millis();
        pitch_a = atan2(myICM.accX(),myICM.accZ())*180/M_PI;
        roll_a = atan2(myICM.accY(),myICM.accZ())*180/M_PI; 

        pitch_a = (((pitch_a + 88) * 180) / 176) - 90; //two point corrections
        roll_a = (((roll_a + 88) * 180) / 176) - 90;

        imu_data[iter][0] = pitch_a;
        imu_data[iter][1] = roll_a; 

        //Low pass filter, about 0.25 Hz
        //0.25 = 1/2piRC -- RC = 2/pi
        //alpha = T/(T+RC) -- T=30ms right now, 2.5 ms without delay
        const float alpha = 0.04;
        if(iter == 0) {
          //low pass init
          imu_data[iter][2] = pitch_a;
          imu_data[iter][3] = roll_a;

          //comp filter init
          imu_data[iter][7] = 0;
          imu_data[iter][8] = 0;

          //gyro
          last_time = micros();
          pitch_g = 0;
          roll_g = 0;
          yaw_g = 0;
        }
        else {
          //lowpass
          imu_data[iter][2] = alpha*pitch_a + (1-alpha)*imu_data[iter-1][2];
          imu_data[iter][3] = alpha*roll_a + (1-alpha)*imu_data[iter-1][3];

          imu_data[iter-1][2] = imu_data[iter][2];
          imu_data[iter-1][3] = imu_data[iter][3];

          //gyro
          dt = (micros()-last_time)/1000000.;
          last_time = micros();
          pitch_g = pitch_g + myICM.gyrY()*dt;
          roll_g = roll_g + myICM.gyrX()*dt;
          yaw_g = yaw_g + myICM.gyrZ()*dt;


          //complimentary filter
          imu_data[iter][7] = ((imu_data[iter-1][7]+myICM.gyrY()*dt)*(1-alpha))+(pitch_a*alpha);
          imu_data[iter][8] = ((imu_data[iter-1][8]+myICM.gyrX()*dt))*(1-alpha)+(roll_a*alpha);
        }

        //gyro
        imu_data[iter][4] = -pitch_g; //invert as gyro reading is opposite of model reading
        imu_data[iter][5] = roll_g;
        imu_data[iter][6] = yaw_g;

        //Comp filter
        
        iter ++;
        delay(30);
    }

    //Sampling rate


    //printAccRollPitch(myICM);
  }
  else
  {
    count ++;
  }
}

// Below here are some helper functions to print the data nicely!

void printPaddedInt16b(int16_t val)
{
  if (val > 0)
  {
    SERIAL_PORT.print(" ");
    if (val < 10000)
    {
      SERIAL_PORT.print("0");
    }
    if (val < 1000)
    {
      SERIAL_PORT.print("0");
    }
    if (val < 100)
    {
      SERIAL_PORT.print("0");
    }
    if (val < 10)
    {
      SERIAL_PORT.print("0");
    }
  }
  else
  {
    SERIAL_PORT.print("-");
    if (abs(val) < 10000)
    {
      SERIAL_PORT.print("0");
    }
    if (abs(val) < 1000)
    {
      SERIAL_PORT.print("0");
    }
    if (abs(val) < 100)
    {
      SERIAL_PORT.print("0");
    }
    if (abs(val) < 10)
    {
      SERIAL_PORT.print("0");
    }
  }
  SERIAL_PORT.print(abs(val));
}

void printRawAGMT(ICM_20948_AGMT_t agmt)
{
  SERIAL_PORT.print("RAW. Acc [ ");
  printPaddedInt16b(agmt.acc.axes.x);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.acc.axes.y);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.acc.axes.z);
  SERIAL_PORT.print(" ], Gyr [ ");
  printPaddedInt16b(agmt.gyr.axes.x);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.gyr.axes.y);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.gyr.axes.z);
  SERIAL_PORT.print(" ], Mag [ ");
  printPaddedInt16b(agmt.mag.axes.x);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.mag.axes.y);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.mag.axes.z);
  SERIAL_PORT.print(" ], Tmp [ ");
  printPaddedInt16b(agmt.tmp.val);
  SERIAL_PORT.print(" ]");
  SERIAL_PORT.println();
}

void printAccRollPitch(ICM_20948_I2C myICM) {
  float pitch_a = atan2(myICM.accY(),myICM.accZ())*180/M_PI; 
  float roll_a  = atan2(myICM.accX(),myICM.accZ())*180/M_PI; 

  pitch_a = (((pitch_a + 88) * 180) / 176) - 90; //two point corrections
  roll_a = (((roll_a + 88) * 180) / 176) - 90;
  
  Serial.print(", pitch_a:");
  Serial.print(pitch_a);
  Serial.print(", roll_a:");
  Serial.println(roll_a);
}

void printFormattedFloat(float val, uint8_t leading, uint8_t decimals)
{
  float aval = abs(val);
  if (val < 0)
  {
    SERIAL_PORT.print("-");
  }
  else
  {
    SERIAL_PORT.print(" ");
  }
  for (uint8_t indi = 0; indi < leading; indi++)
  {
    uint32_t tenpow = 0;
    if (indi < (leading - 1))
    {
      tenpow = 1;
    }
    for (uint8_t c = 0; c < (leading - 1 - indi); c++)
    {
      tenpow *= 10;
    }
    if (aval < tenpow)
    {
      SERIAL_PORT.print("0");
    }
    else
    {
      break;
    }
  }
  if (val < 0)
  {
    SERIAL_PORT.print(-val, decimals);
  }
  else
  {
    SERIAL_PORT.print(val, decimals);
  }
}

#ifdef USE_SPI
void printScaledAGMT(ICM_20948_SPI *sensor)
{
#else
void printScaledAGMT(ICM_20948_I2C *sensor)
{
#endif
  SERIAL_PORT.print("Scaled. Acc (mg) [ ");
  printFormattedFloat(sensor->accX(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->accY(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->accZ(), 5, 2);
  SERIAL_PORT.print(" ], Gyr (DPS) [ ");
  printFormattedFloat(sensor->gyrX(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->gyrY(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->gyrZ(), 5, 2);
  SERIAL_PORT.print(" ], Mag (uT) [ ");
  printFormattedFloat(sensor->magX(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->magY(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->magZ(), 5, 2);
  SERIAL_PORT.print(" ], Tmp (C) [ ");
  printFormattedFloat(sensor->temp(), 5, 2);
  SERIAL_PORT.print(" ]");
  SERIAL_PORT.println();
}
