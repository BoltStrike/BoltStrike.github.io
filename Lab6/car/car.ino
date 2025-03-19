#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU

//#define USE_SPI       // Uncomment this to use SPI


#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"
#include <ArduinoBLE.h>
#include <Wire.h>
#include "SparkFun_VL53L1X.h" //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X
//#include <VL53L1X.h>
//Necessary shutdown pin for xshut
#define SHUTDOWN_PIN 0
#define INTERRUPT_PIN 3 //not used

SFEVL53L1X tof1;
SFEVL53L1X tof2(Wire, SHUTDOWN_PIN, INTERRUPT_PIN);
//VL53L1X tof1;
//VL53L1X tof2(Wire, SHUTDOWN_PIN, INTERRUPT_PIN);

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

//Imu global vars, maybe move local
float pitch_a;
float roll_a;
int last_time;
float dt;
float pitch_g = 0;
float roll_g = 0;
float yaw_g = 0;


//IMU data
icm_20948_DMP_data_t data;


enum CommandTypes
{
    PING,
    GET_IMU_DATA,
    GET_TOF_DATA,
    START_COLLECTION,
    DRIVE,
    GET_PID,
    SET_KPID,
    ORIENT,
    SET_OPID,
    SET_TARGET
};

BLEDevice central;

//tof and collection variables
#define DATA_LENGTH 128
int iter = 0;
int tof_iter1 = 0;
int tof_iter2 = 0;
int collecting = 0;
float adjust = 1.52;
#define PID_LENGTH 4096

float imu_data[DATA_LENGTH][9];
int timestamps[DATA_LENGTH];
float tof[DATA_LENGTH][2];
int tofstamps[DATA_LENGTH];



class Car {
public:
    float fronttof = -5640;
    float last_frontof = -5640;
    int righttof;
    int dt;

    int driving = 0;
    int orient = 0;
    int last_time = 0;
    int start_time = 0;

    int e_hist[PID_LENGTH];
    int t_hist[PID_LENGTH];
    int motor_hist[PID_LENGTH];
    int target_hist[PID_LENGTH][2];


    int e_pos;
    float I=0;
    float dF=0;
    int last_drive=0;
    int deadtime = 0;

    float SKP = 0.06;
    float SKI = 0.003;
    float SKD = 0.016;

    float OKP = 0.06;
    float OKI = 0.003;
    float OKD = 0.016;


    //extrapolation values
    float slope = -5640;
    int last_tof = 0;

    //IMU
    volatile double yaw = 0;

    //targets
    int dist_target = 304; //304 mm
    int orient_target = 90; //90 degrees


};

Car mycar;


#define MOTORLF 3
#define MOTORLB 14
#define MOTORRF 16
#define MOTORRB 15

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

          break;
        }
        case DRIVE:
        {
          mycar.driving = 1;
          mycar.start_time = millis();
          mycar.e_pos = 0;
          break;
        }
        case GET_PID:
        {
          for (int i = 0; i < mycar.e_pos; i++) {
                tx_estring_value.clear();
                tx_estring_value.append("Prop:");
                tx_estring_value.append(mycar.e_hist[i]);
                tx_estring_value.append("Motor:");
                tx_estring_value.append(mycar.motor_hist[i]);
                tx_estring_value.append("Time:");
                tx_estring_value.append(mycar.t_hist[i]);
                tx_estring_value.append("dist:");
                tx_estring_value.append(mycar.target_hist[i][0]);
                tx_estring_value.append("ang:");
                tx_estring_value.append(mycar.target_hist[i][1]);
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
          }
          break;
        }
        case SET_KPID:
        {
            float P, I, D;

            success = robot_cmd.get_next_value(P);
            if (!success)
              return;

            success = robot_cmd.get_next_value(I);
            if (!success)
              return;

            success = robot_cmd.get_next_value(D);
            if (!success)
                return;
            mycar.SKP = P;
            mycar.SKI = I;
            mycar.SKD = D;
            break;
        }
        case ORIENT:
        {
          mycar.orient = 1;
          mycar.start_time = millis();
          mycar.e_pos = 0;
          break;
        }
        case SET_OPID:
        {
            float P, I, D;

            success = robot_cmd.get_next_value(P);
            if (!success)
              return;

            success = robot_cmd.get_next_value(I);
            if (!success)
              return;

            success = robot_cmd.get_next_value(D);
            if (!success)
                return;
            mycar.OKP = P;
            mycar.OKI = I;
            mycar.OKD = D;
            break;
        }
        case SET_TARGET:
        {
          float dist_targ, orient_targ;
          success = robot_cmd.get_next_value(dist_targ);
          if (!success)
            return;

          success = robot_cmd.get_next_value(orient_targ);
          if (!success)
            return;
          mycar.dist_target = int(dist_targ);
          mycar.orient_target = double(orient_targ);
          break;
        }
        default:
            Serial.print("Invalid Command Type: ");
            Serial.println(cmd_type);
            break;
      }
}

int sign(int x) {
    return (x > 0) - (x < 0);
}


float calc_pid(int dist, int setpoint, float KP = mycar.SKP, float KI = mycar.SKI, float KD = mycar.SKD, int mode = 0, float alpha = 0.002) {

  //proportional terms
  int e = dist-setpoint;
  if(mode == 1) { 
    if(e > 180) { //indicates its better to go around the other way
      e -= 360;
    } else if(e < -180) {
      e += 360;
    }
  }
  mycar.e_hist[mycar.e_pos] = e;
  mycar.t_hist[mycar.e_pos] = millis();
  //mycar.target_hist[mycar.e_pos][0] = mycar.dist_target;
  mycar.target_hist[mycar.e_pos][1] = mycar.orient_target;
  mycar.e_pos ++;

  //Serial.println(mycar.dt);

  mycar.I += e * float(mycar.dt)/1000; // integral term
  if(mycar.I > 1000) {
    mycar.I = 1000;
  }
  else if (mycar.I < -1000){
    mycar.I = -1000;
  }

  //Serial.println(mycar.dt);

  if (mycar.e_pos >= PID_LENGTH) { 
    mycar.e_hist[0] = mycar.e_hist[PID_LENGTH-2];
    mycar.e_hist[1] = mycar.e_hist[PID_LENGTH-1];
    mycar.t_hist[0] = mycar.t_hist[PID_LENGTH-2];
    mycar.t_hist[1] = mycar.t_hist[PID_LENGTH-1];
    mycar.motor_hist[0] = mycar.motor_hist[PID_LENGTH-2];
    mycar.motor_hist[1] = mycar.motor_hist[PID_LENGTH-1];
    for(int i = 2; i < PID_LENGTH; i++) {
      mycar.e_hist[i] = 0;
    }
    mycar.e_pos = 2;
  }

  if (mycar.e_pos > 1) {

    float d = (mycar.e_hist[mycar.e_pos-1] - mycar.e_hist[mycar.e_pos-2]) / (float(mycar.dt)/1000);
    
    //Serial.println(mycar.e_hist[mycar.e_pos-1]);
    //Serial.println(mycar.e_hist[mycar.e_pos-2]);
    //Serial.println(mycar.dt/1000);
    mycar.dF = d*alpha + (1-alpha)* mycar.dF;
    mycar.target_hist[mycar.e_pos][0] = mycar.dF;
  }
  else {
    mycar.dF = 0;
  }

  //Serial.println(mycar.dF);
  return KP*e + KD*mycar.dF + KI*mycar.I; //PID sum
}

void drivefb(int value) {


  if (value > 0) {
    //startup to avoid deadband
    if(sign(mycar.last_drive) != sign(value) && value < 70) {
      analogWrite(MOTORLF, 70);
      analogWrite(MOTORRF, 70*adjust);
      analogWrite(MOTORLB, 0);
      analogWrite(MOTORRB, 0);
      mycar.deadtime = millis();
      //Serial.println("Deadband Start");
    }
    else if(millis()-mycar.deadtime > 30) {
      analogWrite(MOTORLF, value);
      analogWrite(MOTORRF, value*adjust);
      analogWrite(MOTORLB, 0);
      analogWrite(MOTORRB, 0);
      //Serial.println("Deadband End");
    }
  }
  else {
    if(sign(mycar.last_drive) != sign(value) && value > -70) {
      analogWrite(MOTORLB, 70);
      analogWrite(MOTORRB, 70*adjust);
      analogWrite(MOTORLF, 0);
      analogWrite(MOTORRF, 0);
      mycar.deadtime = millis();
    }
    else if(millis()-mycar.deadtime > 30) {
      analogWrite(MOTORLB, -1*value);
      analogWrite(MOTORRB, -1*value*adjust);
      analogWrite(MOTORLF, 0);
      analogWrite(MOTORRF, 0);
    }
  }

  mycar.last_drive = value;
}

void driveturn(int value) {


  if (value > 0) {
    //startup to avoid deadband
    if(sign(mycar.last_drive) != sign(value) && value < 140) {
      analogWrite(MOTORLF, 140);
      analogWrite(MOTORRB, 140*adjust);
      analogWrite(MOTORLB, 0);
      analogWrite(MOTORRF, 0);
      mycar.deadtime = millis();
      //Serial.println("Deadband Start");
      mycar.motor_hist[mycar.e_pos-1] = 140;
    }
    else if(millis()-mycar.deadtime > 30) {
      analogWrite(MOTORLF, value);
      analogWrite(MOTORRB, value*adjust);
      analogWrite(MOTORLB, 0);
      analogWrite(MOTORRF, 0);
      //Serial.println("Deadband End");
      mycar.motor_hist[mycar.e_pos-1] = value;
    }
    else {
      mycar.motor_hist[mycar.e_pos-1] = 140;
    }
  }
  else {
    if(sign(mycar.last_drive) != sign(value) && value > -140) {
      analogWrite(MOTORLB, 140);
      analogWrite(MOTORRF, 140*adjust);
      analogWrite(MOTORLF, 0);
      analogWrite(MOTORRB, 0);
      mycar.deadtime = millis();
      mycar.motor_hist[mycar.e_pos-1] = -140;
    }
    else if(millis()-mycar.deadtime > 30) {
      analogWrite(MOTORLB, -1*value);
      analogWrite(MOTORRF, -1*value*adjust);
      analogWrite(MOTORLF, 0);
      analogWrite(MOTORRB, 0);
      mycar.motor_hist[mycar.e_pos-1] = value;
    }
    else {
      mycar.motor_hist[mycar.e_pos-1] = -140;
    }
  }

  mycar.last_drive = value;
}

void stopmotor() {
  analogWrite(MOTORLB, 0);
  analogWrite(MOTORRB, 0);
  analogWrite(MOTORLF, 0);
  analogWrite(MOTORRF, 0);
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

  bool success = true;

  // Initialize the DMP
  success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);

  // Enable the DMP Game Rotation Vector sensor
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);

  // Set the DMP output data rate (ODR): value = (DMP running rate / ODR ) - 1
  // E.g. for a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok); // Set to the maximum

  // Enable the FIFO queue
  success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);

  // Enable the DMP
  success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);

  // Reset DMP
  success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);

  // Reset FIFO
  success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

  //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok); // Set to the maximum

  // Check success
  if (!success) {
      Serial.println("Enabling DMP failed!");
      while (1) {
          // Freeze
      }
  }


  //Initialize tof sensors
  Wire.begin();

  pinMode(SHUTDOWN_PIN, OUTPUT);
  digitalWrite(SHUTDOWN_PIN, LOW); //Turn off 2nd sensor with xshut
  tof1.setI2CAddress(0x30); //give 1st sensor different address
  //tof1.setAddress(0x30); //give 1st sensor different address
  digitalWrite(SHUTDOWN_PIN, HIGH); // Trun on second sensor

  Serial.begin(115200);
  Serial.println("VL53L1X Qwiic Test");

  if (tof1.begin()!=0) //Begin returns 0 on a good init
  {
    Serial.println("Sensor1 failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }
  if (tof2.begin()!=0) //Begin returns 0 on a good init
  {
    Serial.println("Sensor2 failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }
  Serial.println("TOF Sensors online!");
  tof1.setDistanceModeLong();
  tof2.setDistanceModeLong();
  tof1.setTimingBudgetInMs(33);
  tof2.setTimingBudgetInMs(33);
  tof1.startRanging(); //Write configuration bytes to initiate measurement
  tof2.startRanging(); //Write configuration bytes to initiate measurement

  /*tof1.setDistanceMode(VL53L1X::Short);
  tof1.setMeasurementTimingBudget(20000);
  tof2.setDistanceMode(VL53L1X::Short);
  tof2.setMeasurementTimingBudget(20000);*/

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

void collect_tof() {
  //collect TOF data

  if (tof1.checkForDataReady()) {
    //Serial.println("here");
    int new_time = millis();
    int dist = tof1.getDistance();
    if(mycar.fronttof != -5640) {
      float diff_time = new_time - mycar.last_tof;
      float diff_meas = dist - mycar.last_frontof;
      mycar.slope = diff_meas/diff_time;
    }

    mycar.last_tof = new_time;
    mycar.fronttof = dist; //Get the result of the measurement from the sensor
    mycar.last_frontof = dist;

    tof1.clearInterrupt();
    tof1.stopRanging();
    tof1.startRanging();
    if(tof_iter1 < DATA_LENGTH && collecting) {
      tof[tof_iter1][0] = mycar.fronttof;
      tofstamps[tof_iter1] = millis();
      tof_iter1 ++;
    }
  }
  else { //extrapolate tof
    if(mycar.slope != -5640) {
      float dif_time = millis() - mycar.last_tof;
      mycar.fronttof = mycar.last_frontof + dif_time*mycar.slope;
      if(tof_iter1 < DATA_LENGTH && collecting) {
        tof[tof_iter1][0] = mycar.fronttof;
        tofstamps[tof_iter1] = millis();
        tof_iter1 ++;
      }
    }
  }
  if (tof2.checkForDataReady()) {
    mycar.righttof = tof2.getDistance(); //Get the result of the measurement from the sensor
    tof2.clearInterrupt();
    tof2.stopRanging();
    tof2.startRanging();
    if(tof_iter2 < DATA_LENGTH && collecting) {
      tof[tof_iter2][1] = mycar.righttof;
      tof_iter2 ++;
    }
  }
}

void get_DMP() {
    // Is valid data available?
  if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) {
      // We have asked for GRV data so we should receive Quat6
      if ((data.header & DMP_header_bitmap_Quat6) > 0) {
          double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
          double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
          double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30

          double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

          double qw = q0; // See issue #145 - thank you @Gord1
          double qx = q2;
          double qy = q1;
          double qz = -q3;

          // roll (x-axis rotation)
          /*double t0 = +2.0 * (qw * qx + qy * qz);
          double t1 = +1.0 - 2.0 * (qx * qx + qy * qy);
          double roll = atan2(t0, t1) * 180.0 / PI;
          Serial.print("Roll: ");
          Serial.print(roll);

          // pitch (y-axis rotation)
          double t2 = +2.0 * (qw * qy - qx * qz);
          t2 = t2 > 1.0 ? 1.0 : t2;
          t2 = t2 < -1.0 ? -1.0 : t2;
          double pitch = asin(t2) * 180.0 / PI;
          Serial.print("  Pitch: ");
          Serial.print(pitch);*/

          // yaw (z-axis rotation)
          double t3 = +2.0 * (qw * qz + qx * qy);
          double t4 = +1.0 - 2.0 * (qy * qy + qz * qz);
          double yaw = atan2(t3, t4) * 180.0 / PI;
          mycar.yaw = yaw;
          //Serial.print("  Yaw: ");
          //Serial.println(yaw);

      }
  }
}

void collect_imu() {
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
  //delay(30);
}

void loop()
{
  collect_tof();


  mycar.dt = millis() - mycar.last_time;
  mycar.last_time = millis();

  myICM.readDMPdataFromFIFO(&data);
  
  if(mycar.driving || mycar.orient) {
    //set for one foot
    if(millis() - mycar.start_time < 15000) {
      //Serial.println(millis() - mycar.start_time);
      //Serial.println(mycar.fronttof);
      float pid;
      if(mycar.driving) {
        pid = calc_pid(mycar.fronttof, mycar.dist_target);
      }
      else {
        get_DMP();
        if(!isnan(mycar.yaw)) { //Cover DMP returning nan on start
          pid = calc_pid(mycar.yaw, mycar.orient_target, mycar.OKP, mycar.OKI, mycar.OKD, 1);
        }
      }
      //if(abs(mycar.e_hist[mycar.e_pos-1]) > 5) {
      //Serial.println(mycar.e_hist[mycar.e_pos-1]);
      //int anval = (int)(pid/3000 * 166); //166 is max speed, as 166 * 1.52 scaling = 255 
      int anval = pid;
      if(anval > 166) {
        anval = 166;
      } 
      else if(anval < -166) {
        anval = -166;
      }
      
      if(mycar.driving){
        if (anval > 0 && anval < 50) {
          anval = 50;
        }
        else if (anval < 0 && anval > -50) {
          anval = -50;
        }
        drivefb(anval);
      }

      if(mycar.orient) {
        if (anval > 0 && anval < 110) {
          anval = 110;
        }
        else if (anval < 0 && anval > -110) {
          anval = -110;
        }
        driveturn(anval);
      }

      //Serial.println(anval);
      //mycar.motor_hist[mycar.e_pos-1] = anval;
        //transmit histor

        //Serial.print("PID value: ");
        //Serial.println(pid);
        //Serial.print("Drive value: ");
        //Serial.println(anval);
      //}
      /*if (mycar.e_pos > 25) {
        //Serial.println("E stop")
        float sum = 0;
        for (int j =1; j < 26; j++) {
          sum += mycar.e_hist[mycar.e_pos-j];
        }
        sum = sum/25;
        if(abs(sum) < 0.5){
          mycar.driving = 0;
          mycar.orient = 0;
          stopmotor();
        }
      }*/
      
    }
    else {
      //Serial.println("T stop");
      mycar.driving = 0;
      stopmotor();
    }

  }


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

  /*if (myICM.dataReady())
  {
    myICM.getAGMT();         // The values are only updated when you call 'getAGMT'
                             //    printRawAGMT( myICM.agmt );     // Uncomment this to see the raw values, taken directly from the agmt structure
    //printScaledAGMT(&myICM); // This function takes into account the scale settings from when the measurement was made to calculate the values with unit

    //Collect IMU Data for transfer
    if(collecting && iter<DATA_LENGTH) {
      collect_imu();
      Serial.println("collecting imu");
    }

    //printAccRollPitch(myICM);
  }*/




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

