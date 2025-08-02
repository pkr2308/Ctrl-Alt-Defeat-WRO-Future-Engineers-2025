#include <hwrev1_serial_log.hpp>

hw_rev_1_SerialLog::hw_rev_1_SerialLog(VehicleConfig cfg){

  _config = cfg;

}

void hw_rev_1_SerialLog::init(){
 
  Serial1.setRX(_config.pinConfig.uart1RX);
  Serial1.setTX(_config.pinConfig.uart1TX);

  sendMessage("hw_rev_1_SerialLog", "Logger initialised");

}

void hw_rev_1_SerialLog::sendMessage(String sender, String message){

  String log = "[" + sender + "] " + message;
  Serial1.println(log);
 
}
