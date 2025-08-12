/**
 * @brief Implementation for hwrev2 debug logger over UART
 * @author DIY Labs
 */

#include <Arduino.h>
#include "hwrev2_uart_logger.hpp"

hw_rev_2_UARTLogger::hw_rev_2_UARTLogger(VehicleConfig cfg){

  _config = cfg;
  _baudRate = _config.constantsConfig.debugSerialBaudRate;

}

void hw_rev_2_UARTLogger::init(){

  Serial1.begin(_baudRate);

}

void hw_rev_2_UARTLogger::sendMessage(String sender, LogType type, String message){

  String log;

  log += "[";
  log += sender;
  log += " : ";
  log += _stringFromType(type);
  log += " : ";
  log += millis();
  log += "]   ";
  
  log += message;

  Serial1.println(log);

}

void hw_rev_2_UARTLogger::sendString(String string){

  Serial1.print(string);

}

String hw_rev_2_UARTLogger::_stringFromType(LogType type){

  String typeString;

  switch(type){

    case INFO:
      typeString = "INFO";  
      break;

    case WARNING:
      typeString = "WARN";  
      break;

    case ERROR:
      typeString = "ERR";  
      break;

  }

  return typeString;

}