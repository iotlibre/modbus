
#ifndef MODBUS_SUPERCAL_531_H
#define MODBUS_SUPERCAL_531_H

#include <Arduino.h>
#include <ModbusMaster.h>


#define MAX485_DE      D5
#define MAX485_RE_NEG  D6
#define LED_STATUS  D3
#define DEVICE_DIRECTION  5

/*
Information before Modbus software implementation:
DATA:    [=====     ]  52.4% (used 42916 bytes from 81920 bytes)
PROGRAM: [=         ]  10.3% (used 432488 bytes from 4194304 bytes)
*/

// Definitios of the MODBUS query
// To read all data it is necessary two different queries
#define NUMBER_OF_REGISTERS 5
#define DIRECTION_TO_READ_1 202   //Energia acumulada
#define REGISTERS_TO_READ_1 1
#define DIRECTION_TO_READ_2 802   //potencia kw
#define REGISTERS_TO_READ_2 1
#define DIRECTION_TO_READ_3 812   //caudal
#define REGISTERS_TO_READ_3 1
#define DIRECTION_TO_READ_4 822   //temparatura
#define REGISTERS_TO_READ_4 2



#define PARAMETER_LIST "Energia","Potencia","Caudal","Ta_alta","Ta_baja"

#define TRANSMISSION_LIST "Energia","Potencia","Caudal","Ta_alta","Ta_baja"

// Divider factor to move to correct units
#define FACTOR_LIST 1,1,1,100,100  //divider

String array_parameters[] = {PARAMETER_LIST};
String transmission_list[] = {TRANSMISSION_LIST};
int factor_list[] = {FACTOR_LIST};
int tx_mark[NUMBER_OF_REGISTERS];
// registro de resultadoS para transmitir al servidor
float tx_values[NUMBER_OF_REGISTERS];

// variables to calculate the number of member in the array
int number_of_parameters = 0;
int number_of_tx_list = 0;


// For secuencial execution of functions (temporal thread)
uint32_t t_last_tx = 0;
int modbus_state = 4;
int system_status = 0; // to manage the led

// instantiate ModbusMaster object
ModbusMaster node;

void preTransmission()
{
  digitalWrite(MAX485_RE_NEG, 1);
  digitalWrite(MAX485_DE, 1);
}
void postTransmission()
{
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
}


// ************************************
// *******    _FUNCTIONS      ********
// ************************************

// Sedical high memory -> more significative part
long two_register_to_long(uint16_t low_16, uint16_t high_16){
  uint32_t both_32 = 0x00000000;
  uint32_t sign_32 = 0x00000000;
  long both_long = 0;

  both_32 = ((uint32_t)high_16) << 16;
  both_32 = both_32 | (uint32_t)low_16;
  sign_32 = both_32 & 0x80000000;
  both_long = (long)both_32;

  // Serial.println("_two_register_to_long_");
  // Serial.print("HEX: ");
  // Serial.print((unsigned long)both_32,HEX);
  // Serial.print(" - sign: ");
  // Serial.print((unsigned long)sign_32,HEX); // 0 o 80000000
  // Serial.print(" - long: ");
  // Serial.println(both_long);

  return both_long;
}

/* n: number from 0 to number of parameters received in the last modbus query
it depends on the registers received with reasImputRegisters
first: first position in the results array: tx_values[]
*/

void result_to_register(int n, int first){
  long both_long = 0;
  both_long = two_register_to_long(node.getResponseBuffer(n*2),node.getResponseBuffer((n*2)+1));
  tx_values[n+first] = (float)both_long/(float)factor_list[n+first];
  // Serial.println("_result_to_tx_register_");
  // Serial.print(n+first);
  // Serial.print("  both_long: ");
  // Serial.print(both_long);
  // Serial.print(" - float: ");
  // Serial.println(tx_values[n+first]);
}

/*
default values when modus is not available
*/
void default_value_to_register(int first, int last){
  Serial.println("_default_value_to_register_");
  for (int i =first; i < (last); i++) {
    tx_values[i] = 1.12;
    Serial.print("tx_values[] ");
    Serial.print(i);
    Serial.print(" -> ");
    Serial.println(tx_values[i]);
  }
}

void parameter_list_mark_to_tx(){
  Serial.println("_parameter_list_mark_to_tx_");
  //Size of all pointers / Size of a pointer
  number_of_parameters = sizeof(array_parameters)/sizeof(array_parameters[0]);
  number_of_tx_list = sizeof(transmission_list)/sizeof(transmission_list[0]);
  Serial.print("number_of_parameters   :");
  Serial.println(number_of_parameters);
  Serial.print("number_of_tx_list   :");
  Serial.println(number_of_tx_list);

  for (int pl = 0; pl < number_of_parameters; pl++) {         //parameter_list
    tx_mark[pl] = 0;
    for (int tl = 0; tl < number_of_tx_list; tl++) {    //transmission_list
      int r = array_parameters[pl].equalsIgnoreCase(transmission_list[tl]);
      if (r) {
        tx_mark[pl] = 1;
        Serial.print("realtion_pl_tl   :");
        Serial.print(pl);
        Serial.print(" = ");
        Serial.println(tl);
        break;
      }
    }
  }
}

String compose_msj_to_tx() {
  String message_to_tx_ = "";
  for (int i = 0; i < number_of_parameters; i++) {
    if (tx_mark[i]) {
      message_to_tx_ += array_parameters[i];
      message_to_tx_ += ":";
      message_to_tx_ += tx_values[i];
      message_to_tx_ += ",";
    }
  }
  if (message_to_tx_.endsWith(",")) {
    message_to_tx_.remove((message_to_tx_.length()-1));
  }
  return message_to_tx_;
}

void led_system_status(){
  if (system_status == 0) {system_status = 1;}
  else{system_status = 0;}
  digitalWrite(LED_STATUS, system_status);
}

// ************************************
// *******    MODBUS_SETUP      *******
// ************************************


void modbus_setup()
{
  Serial.println("_modbus_setup_");
  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  pinMode(LED_STATUS, OUTPUT);

  // Init in receive mode
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
  digitalWrite(LED_STATUS, 0);

  // Modbus communication runs at 115200 baud
  // Modbus slave ID 3
  node.begin(DEVICE_DIRECTION, Serial);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);

  parameter_list_mark_to_tx();
}

// ************************************
// *******    MODBUS_THREAD    ********
// ************************************

String modbus_loop()
{
  // sizeof(uint16_t) --> 2
  // sizeof(uint32_t) --> 4
  // sizeof(unsigned long) --> 4
  // sizeof(long) --> 4
  // Serial.println(mayor_32) --> 2147483648

  String msj_to_tx = "nook";
  uint8_t result;

// ***************** modbus setup *************************

  if (modbus_state == 1) {
    modbus_state = 2;
    // result = node.readInputRegisters(0x03E8, 3);
    result = node.readInputRegisters(822, 4);
    Serial.print("_readInputRegisters_822_");
    Serial.print("    modbus_state -->  ");
    Serial.println(modbus_state);
    if (result == node.ku8MBSuccess){
      Serial.println("lectura de los registros ES VALIDA");
      Serial.print("0x00: ");
      Serial.println(node.getResponseBuffer(0x00),HEX);
      Serial.print("0x01: ");
      Serial.println(node.getResponseBuffer(0x01),HEX);
      Serial.print("0x02: ");
      Serial.println(node.getResponseBuffer(0x02),HEX);
      Serial.print("0x03: ");
      Serial.println(node.getResponseBuffer(0x03),HEX);
    }
    else {Serial.println("the reading is NOT CORRECT");}
  }

  else if (modbus_state == 2) {
    modbus_state = 3;
    // result = node.readInputRegisters(0x03E8, 3);
    result = node.readInputRegisters(812, 2);
    Serial.print("_readInputRegisters_812_");
    Serial.print("    modbus_state -->  ");
    Serial.println(modbus_state);
    if (result == node.ku8MBSuccess){
      Serial.println("lectura de los registros ES VALIDA");
      Serial.print("0x00: ");
      Serial.println(node.getResponseBuffer(0x00),HEX);
      Serial.print("0x01: ");
      Serial.println(node.getResponseBuffer(0x01),HEX);
    }
    else {Serial.println("the reading is NOT CORRECT");}
  }


  else if (modbus_state == 3) {
    modbus_state = 4;
    // result = node.readInputRegisters(0x03E8, 3);
    result = node.readInputRegisters(202, 2);
    Serial.print("_readInputRegisters_202_");
    Serial.print("    modbus_state -->  ");
    Serial.println(modbus_state);
    if (result == node.ku8MBSuccess){
      Serial.println("The reading is CORRECT");
      Serial.print("0x00: ");
      Serial.println(node.getResponseBuffer(0x00),HEX);
      Serial.print("0x01: ");
      Serial.println(node.getResponseBuffer(0x01),HEX);
    }
    else {Serial.println("the reading is NOT CORRECT");}
  }


/* reading of register. Firs query
*/

  else if (modbus_state == 4) {
    modbus_state = 5;
    result = node.readInputRegisters(DIRECTION_TO_READ_1, (REGISTERS_TO_READ_1*2));
    Serial.println("");
    Serial.print("_readInputRegisters_E_a: ");
    Serial.print(result);
    Serial.print("    modbus_state --> ");
    Serial.println(modbus_state);
    if (result == node.ku8MBSuccess)
    {
      for (int i =0; i < (REGISTERS_TO_READ_1); i++) {
      // The first result go to the first position asigned in tx_values
        result_to_register(i,0);

      }
      for (int i = 0; i < (REGISTERS_TO_READ_1); i++) {
        Serial.print(array_parameters[i]);
        Serial.print(" ---> ");
        Serial.println(tx_values[i]);
      }
      led_system_status();
    }
    else {
      Serial.println("the reading is NOT CORRECT");
      default_value_to_register(0,REGISTERS_TO_READ_1);
    }
  }


  else if (modbus_state == 5) {
    modbus_state = 6;
    result = node.readInputRegisters(DIRECTION_TO_READ_2, (REGISTERS_TO_READ_2*2));
    Serial.println("");
    Serial.print("_readInputRegisters_2_P_i: ");
    Serial.print(result);
    Serial.print("    modbus_state -->  ");
    Serial.println(modbus_state);
    if (result == node.ku8MBSuccess){
      for (int i =0; i < (REGISTERS_TO_READ_2); i++) {
        // The first result go to the first position asigned in tx_values
        result_to_register(i,REGISTERS_TO_READ_1);
      }
      for (int i = REGISTERS_TO_READ_1; i < (REGISTERS_TO_READ_1+REGISTERS_TO_READ_2); i++) {
        Serial.print(array_parameters[i]);
        Serial.print(" ---> ");
        Serial.println(tx_values[i]);
      }
      led_system_status();
    }
    else {
      Serial.println("the reading is NOT CORRECT");
      default_value_to_register(REGISTERS_TO_READ_1,(REGISTERS_TO_READ_1+REGISTERS_TO_READ_2));
    }
  }

  else if (modbus_state == 6) {
    modbus_state = 7;
    result = node.readInputRegisters(DIRECTION_TO_READ_3, (REGISTERS_TO_READ_3*2));
    Serial.println("");
    Serial.print("_readInputRegisters_F_i: ");
    Serial.print(result);
    Serial.print("    modbus_state -->  ");
    Serial.println(modbus_state);
    if (result == node.ku8MBSuccess){
      for (int i =0; i < (REGISTERS_TO_READ_3); i++) {
        // The first result go to the first position asigned in tx_values
        result_to_register(i,REGISTERS_TO_READ_1+REGISTERS_TO_READ_2);
      }
      for (int i = REGISTERS_TO_READ_1+REGISTERS_TO_READ_2;
               i < (REGISTERS_TO_READ_1+REGISTERS_TO_READ_2+REGISTERS_TO_READ_3); i++) {
        Serial.print(array_parameters[i]);
        Serial.print(" ---> ");
        Serial.println(tx_values[i]);
      }
      led_system_status();
    }
    else {
      Serial.println("the reading is NOT CORRECT");
      default_value_to_register(REGISTERS_TO_READ_1+REGISTERS_TO_READ_2,
                               (REGISTERS_TO_READ_1+REGISTERS_TO_READ_2+REGISTERS_TO_READ_3));
    }
  }

  else if (modbus_state == 7) {
    modbus_state = 8;
    result = node.readInputRegisters(DIRECTION_TO_READ_4, (REGISTERS_TO_READ_4*2));
    Serial.println("");
    Serial.print("_readInputRegisters_T_h_T_l: ");
    Serial.print(result);
    Serial.print("    modbus_state -->  ");
    Serial.println(modbus_state);
    if (result == node.ku8MBSuccess){
      for (int i =0; i < (REGISTERS_TO_READ_4); i++) {
        // The first result go to the first position asigned in tx_values
        result_to_register(i,REGISTERS_TO_READ_1 + REGISTERS_TO_READ_2 + REGISTERS_TO_READ_3);
      }
      for (int i = REGISTERS_TO_READ_1+REGISTERS_TO_READ_2+REGISTERS_TO_READ_3;
               i < (REGISTERS_TO_READ_1+REGISTERS_TO_READ_2+REGISTERS_TO_READ_3+REGISTERS_TO_READ_4); i++) {
        Serial.print(array_parameters[i]);
        Serial.print(" ---> ");
        Serial.println(tx_values[i]);
      }
      led_system_status();
    }
    else {
      Serial.println("the reading is NOT CORRECT");
      default_value_to_register(REGISTERS_TO_READ_1+REGISTERS_TO_READ_2+REGISTERS_TO_READ_3,
                               (REGISTERS_TO_READ_1+REGISTERS_TO_READ_2+REGISTERS_TO_READ_3+REGISTERS_TO_READ_4));
    }
  }


  else {
    modbus_state = 4;
    msj_to_tx = compose_msj_to_tx();
    Serial.print("_compose_msj_to_tx_");
    Serial.print("    modbus_state --> ");
    Serial.println(modbus_state);
    led_system_status();
  }
  return msj_to_tx;

}

#endif // MODBUS_SUPERCAL_531_H
