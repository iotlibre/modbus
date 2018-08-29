
#ifndef MODBUS_CVM1D_H
#define MODBUS_CVM1D_H

#include <Arduino.h>
#include <ModbusMaster.h>


#define MAX485_DE      D5
#define MAX485_RE_NEG  D6

/*
Information before Modbus software implementation:
DATA:    [=====     ]  52.4% (used 42916 bytes from 81920 bytes)
PROGRAM: [=         ]  10.3% (used 432488 bytes from 4194304 bytes)
*/

// Definitios of the MODBUS query
// To read all data it is necessary two different queries
#define NUMBER_OF_REGISTERS 24
#define DIRECTION_TO_READ_1 0x0000
#define REGISTERS_TO_READ_1 15
#define DIRECTION_TO_READ_2 0x001E
#define REGISTERS_TO_READ_2 9



#define PARAMETER_LIST "V_1","A_1","Kw_1","Kvar_1","PF_1",\
                       "V_2","A_2","Kw_2","Kvar_2","PF_2",\
                       "V_3","A_3","Kw_3","Kvar_3","PF_3",\
                       "Kw_III","KvarL_III","KvarC_III","Cos_III","PFIII",\
                       "Hz","V12","V23","V31"

#define TRANSMISSION_LIST "Kw_1","Kvar_1",\
                          "Kw_2","Kvar_2",\
                          "Kw_3","Kvar_3","V23"

// Divider factor to move to correct units
#define FACTOR_LIST 10,1000,1,1,100,\
                    10,1000,1,1,100,\
                    10,1000,1,1,100,\
                    1,1,1,100,100,\
                    10,10,10,10

// arrays needed to read and manage the modbus information
String array_parameters[] = {PARAMETER_LIST};
String transmission_list[] = {TRANSMISSION_LIST};
int factor_list[] = {FACTOR_LIST};
int tx_mark[NUMBER_OF_REGISTERS];
float tx_values[NUMBER_OF_REGISTERS];

// variables to calculate the number of member in the array
int number_of_parameters = 0;
int number_of_tx_list = 0;



/* For secuencial execution of functions (temporal thread)
The system will execute only one function of the thread each 20 ms
after that execution the program will continue
once the 20 has passed, the program will execute the following function of the thread
t_last_tx variable to know the time from the last function execution
modbus_state variable to indicate the next functio to be executed
*/
uint32_t t_last_tx =0;
int modbus_state = 1;

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


long two_register_to_long(uint16_t high_16, uint16_t low_16){
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

// ************************************
// *******    MODBUS_SETUP      *******
// ************************************


void modbus_setup()
{
  Serial.println("_modbus_setup_");
  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  // Init in receive mode
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);

  // Modbus communication runs at 115200 baud
  // Modbus slave ID 3
  node.begin(3, Serial);
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
    // primary voltage          1(Dec)          00000001 (Hex)
    node.setTransmitBuffer(0x00, 0x0000);
    node.setTransmitBuffer(0x01, 0x0001);
    // secondary voltage      1(Dec)          0001 (Hex)
    node.setTransmitBuffer(0x02, 0x0001);
    // primary current    50(Dec) 32 Hex) 5000(Dec) 1388(Hex)
    node.setTransmitBuffer(0x03, 0x0032);
    // Sin uso
    node.setTransmitBuffer(0x04, 0x0000);
    // Cálculo de armónicos      00 Respecto el Valor Eficaz
    node.setTransmitBuffer(0x05, 0x0000);
    node.writeMultipleRegisters(0x044C, 6);
    Serial.print("");
    Serial.print("_setTransmitBuffer_044C_energy_configuration_");
    Serial.print("    modbus_state -->  ");
    Serial.println(modbus_state);
    Serial.println("_writeMultipleRegisters_044C_");
  }

  else if (modbus_state == 2) {
    modbus_state = 3;
    result = node.readInputRegisters(0x044C, 6);
    Serial.println("");
    Serial.print("_readInputRegisters_044C_energy_configuration_");
    Serial.print("    modbus_state -->  ");
    Serial.println(modbus_state);
    if (result == node.ku8MBSuccess)
    {
      Serial.println("The reading is CORRECT");
      Serial.print("0x00: ");
      Serial.println(node.getResponseBuffer(0x00));
      Serial.print("0x01: ");
      Serial.println(node.getResponseBuffer(0x01));
      Serial.print("0x02: ");
      Serial.println(node.getResponseBuffer(0x02));
      Serial.print("0x03-primario corriente: ");
      Serial.println(node.getResponseBuffer(0x03));
      Serial.print("0x04: ");
      Serial.println(node.getResponseBuffer(0x04));
      Serial.print("0x05: ");
      Serial.println(node.getResponseBuffer(0x05));
    }
    else {Serial.println("the reading is NOT CORRECT");}
  }

  else if (modbus_state == 3) {
    modbus_state = 4;
    result = node.readInputRegisters(0x03E8, 3);
    Serial.print("_readInputRegisters_03E8_");
    Serial.print("    modbus_state -->  ");
    Serial.println(modbus_state);
    if (result == node.ku8MBSuccess){
      Serial.println("The reading is CORRECT");
      Serial.print("0x00 protocolo + periferico: ");
      Serial.println(node.getResponseBuffer(0x00),HEX);
      Serial.print("0x01 velocidad (4=19200) + paridad (0=No): ");
      Serial.println(node.getResponseBuffer(0x01),HEX);
      Serial.print("0x02 longitud bits + bits stop: ");
      Serial.println(node.getResponseBuffer(0x02),HEX);
    }
    else {Serial.println("the reading is NOT CORRECT");}
  }


/* reading of register. Firs query
*/

  else if (modbus_state == 4) {
    modbus_state = 5;
    result = node.readInputRegisters(DIRECTION_TO_READ_1, (REGISTERS_TO_READ_1*2));
    Serial.println("");
    Serial.print("_readInputRegisters_1_: ");
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
    }
    else {
      Serial.println("the reading is NOT CORRECT");
      default_value_to_register(0,REGISTERS_TO_READ_1);
    }
  }

  /* reading of register. Second query
  */
  else if (modbus_state == 5) {
    modbus_state = 6;
    result = node.readInputRegisters(DIRECTION_TO_READ_2, (REGISTERS_TO_READ_2*2));
    Serial.println("");
    Serial.print("_readInputRegisters_2_: ");
    Serial.print(result);
    Serial.print("    modbus_state -->  ");
    Serial.println(modbus_state);
    if (result == node.ku8MBSuccess){
      for (int i =0; i < (REGISTERS_TO_READ_2); i++) {
        // The first result go to the first position asigned in tx_values
        result_to_register(i,REGISTERS_TO_READ_1);
      }
      for (int i = REGISTERS_TO_READ_1; i < (NUMBER_OF_REGISTERS); i++) {
        Serial.print(array_parameters[i]);
        Serial.print(" ---> ");
        Serial.println(tx_values[i]);
      }
    }
    else {
      Serial.println("the reading is NOT CORRECT");
      default_value_to_register(REGISTERS_TO_READ_1,NUMBER_OF_REGISTERS);
    }
  }
  else {
    modbus_state = 4;
    msj_to_tx = compose_msj_to_tx();
    Serial.print("_compose_msj_to_tx_");
    Serial.print("    modbus_state --> ");
    Serial.println(modbus_state);
  }
  return msj_to_tx;

}

#endif // MODBUS_CVM1D_H
