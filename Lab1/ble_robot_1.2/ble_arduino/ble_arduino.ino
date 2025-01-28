
#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"
#include <ArduinoBLE.h>

//////////// BLE UUIDs ////////////
#define BLE_UUID_TEST_SERVICE "350447e4-252a-49ab-b5a3-46cfae2701aa"

#define BLE_UUID_RX_STRING "9750f60b-9c9c-4158-b620-02ec9521cd99"

#define BLE_UUID_TX_FLOAT "27616294-3063-4ecc-b60b-3470ddef2938"
#define BLE_UUID_TX_STRING "f235a225-6735-4d73-94cb-ee5dfce9ba83"
//////////// BLE UUIDs ////////////

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

char *byte_str;

#define LEN 128
int time_arr[LEN];
int temp_arr[LEN];
//////////// Global Variables ////////////

enum CommandTypes
{
    PING,
    SEND_TWO_INTS,
    SEND_THREE_FLOATS,
    ECHO,
    DANCE,
    SET_VEL,
    GET_TIME_MILLIS,
    START_TIME_LOOP,
    SEND_TIME_DATA,
    GET_TEMP_READINGS,
    INT_LOOP,
    INIT_VAR,
    VAR_BYTE_RESPONSE,
};

void
loop_millis()
{
    unsigned long startMillis = millis();
    //Five seconds, can increase
    int i = 0;
    while(millis() - startMillis < 5000) {

      //Array Implementation (Part 6)
      time_arr[i] = int(millis());

      //Temp Implementation (Part 7)

      temp_arr[i] = analogReadTemp();    // raw ADC counts from die temperature sensor
      //float temp_f = getTempDegF();  //can swap to float array and use this for farenheit

      i ++;
      if (i == LEN){ 
        break;
      }

      //Continuous send implementation (Part 5)
      /*
      tx_estring_value.clear();
      tx_estring_value.append("T:");
      tx_estring_value.append(int(millis()));
      tx_characteristic_string.writeValue(tx_estring_value.c_str());

      Serial.print("Sent back: ");
      Serial.println(tx_estring_value.c_str());*/
    }
}

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
            tx_estring_value.clear();
            tx_estring_value.append("PONG");
            tx_characteristic_string.writeValue(tx_estring_value.c_str());

            Serial.print("Sent back: ");
            Serial.println(tx_estring_value.c_str());

            break;
        /*
         * Extract two integers from the command string
         */
        case SEND_TWO_INTS:
            int int_a, int_b;

            // Extract the next value from the command string as an integer
            success = robot_cmd.get_next_value(int_a);
            if (!success)
                return;

            // Extract the next value from the command string as an integer
            success = robot_cmd.get_next_value(int_b);
            if (!success)
                return;

            Serial.print("Two Integers: ");
            Serial.print(int_a);
            Serial.print(", ");
            Serial.println(int_b);
            
            break;
        /*
         * Extract three floats from the command string
         */
        case SEND_THREE_FLOATS:

            float float_a, float_b, float_c;

            success = robot_cmd.get_next_value(float_a);
            if (!success)
              return;

            success = robot_cmd.get_next_value(float_b);
            if (!success)
              return;

            success = robot_cmd.get_next_value(float_c);
            if (!success)
                return;

            Serial.print("Three Floats: ");
            Serial.print(float_a);
            Serial.print(", ");
            Serial.print(float_b);
            Serial.print(", ");
            Serial.println(float_c);

            break;
        /*
         * Add a prefix and postfix to the string value extracted from the command string
         */
        case ECHO:
          {
            char char_arr[MAX_MSG_SIZE];
            char ret_arr[MAX_MSG_SIZE] = "Robot says -> ";

            // Extract the next value from the command string as a character array
            success = robot_cmd.get_next_value(char_arr);
            if (!success)
                return;

            strcat(ret_arr, char_arr);
            strcat(ret_arr, " :)");

            tx_estring_value.clear();
            tx_estring_value.append(ret_arr);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());

            Serial.print("Sent back: ");
            Serial.println(tx_estring_value.c_str());
            
            break;
          }
        /*
         * DANCE
         */
        case DANCE:
            Serial.println("Look Ma, I'm Dancin'!");

            break;
        
        /*
         * SET_VEL
         */
        case SET_VEL:

            break;
        
        case GET_TIME_MILLIS:
          {
            //char milli_arr[10];
            //itoa(millis(), milli_arr, 10);

            //char milli_str[20]="T:";
            //strcat(milli_str, milli_arr);

            tx_estring_value.clear();
            tx_estring_value.append("T:");
            tx_estring_value.append(int(millis()));
            tx_characteristic_string.writeValue(tx_estring_value.c_str());

            Serial.print("Sent back: ");
            Serial.println(tx_estring_value.c_str());

            break;
          }
        case START_TIME_LOOP:
          {
            loop_millis();
            break;
          }
        case SEND_TIME_DATA:
          {
            for (int i = 0; i < LEN; i++ ){
              tx_estring_value.clear();
              tx_estring_value.append("T:");
              tx_estring_value.append(time_arr[i]);
              tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }
            Serial.print("Sent time stamp array");
            break;
          }
        case GET_TEMP_READINGS:
        {
          for (int i = 0; i < LEN; i++ ){
            tx_estring_value.clear();
            tx_estring_value.append("Time:");
            tx_estring_value.append(time_arr[i]);
            tx_estring_value.append("Temp:");
            tx_estring_value.append(temp_arr[i]);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
          }
          Serial.print("Sent time stamp and temperature arrays");
          break;
        }
        case INT_LOOP:
        {
          int i;
          while(i<10000){
             tx_characteristic_float.writeValue(i);
             i++;
             //Serial.print(i);
          }
          Serial.print("Done");
          break;
        }
        case INIT_VAR:
        {
          int len;

          // Extract the next value from the command string as an integer
          success = robot_cmd.get_next_value(len);
          if (success) {
            if (byte_str){
              //free(byte_str);
            }
            byte_str = (char *)malloc(len);
            //Serial.println(len);
            //Init variable string length
            for (int i = 0; i < len; i++){
              byte_str[i] = 'a';
            }
          }
          break;
        }
        case VAR_BYTE_RESPONSE:
        {
          tx_estring_value.clear();
          tx_estring_value.append(byte_str);
          tx_characteristic_string.writeValue(tx_estring_value.c_str());
          break;
        }
        /* 
         * The default case may not capture all types of invalid commands.
         * It is safer to validate the command string on the central device (in python)
         * before writing to the characteristic.
         */
        default:
            Serial.print("Invalid Command Type: ");
            Serial.println(cmd_type);
            break;
    }
}

void
setup()
{

    Serial.begin(115200);

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

    // Initial values for characteristics
    // Set initial values to prevent errors when reading for the first time on central devices
    tx_characteristic_float.writeValue(0.0);

    /*
     * An example using the EString
     */
    // Clear the contents of the EString before using it
    tx_estring_value.clear();

    // Append the string literal "[->"
    tx_estring_value.append("[->");

    // Append the float value
    tx_estring_value.append(9.0);

    // Append the string literal "<-]"
    tx_estring_value.append("<-]");

    // Write the value to the characteristic
    tx_characteristic_string.writeValue(tx_estring_value.c_str());

    // Output MAC Address
    Serial.print("Advertising BLE with MAC: ");
    Serial.println(BLE.address());

    BLE.advertise();
}

void
write_data()
{
    currentMillis = millis();
    if (currentMillis - previousMillis > interval) {

        tx_float_value = tx_float_value + 0.5;
        tx_characteristic_float.writeValue(tx_float_value);

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


void
loop()
{
    // Listen for connections
    BLEDevice central = BLE.central();

    // If a central is connected to the peripheral
    if (central) {
        Serial.print("Connected to: ");
        Serial.println(central.address());

        // While central is connected
        while (central.connected()) {
            // Send data
            write_data();

            // Read data
            read_data();
        }

        Serial.println("Disconnected");
    }
}
