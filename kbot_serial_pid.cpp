#include "kbot_serial_pid.h"

KBOTSERIAL::KBOTSERIAL(){}

void KBOTSERIAL::begin(){
    EEPROM.begin(EEPROM_SIZE);
}

void KBOTSERIAL::writeFlashData(uint8_t _address,uint16_t _buffer_write){
  uint8_t _myArray[2];
  _myArray[0] = (_buffer_write >> 8) & 0xFF;
  _myArray[1] = _buffer_write & 0xFF;
  EEPROM.write(_address++,_myArray[0]);
  EEPROM.write(_address,_myArray[1]);
  EEPROM.commit();
}

float KBOTSERIAL::readFlashData(uint8_t _address){
  uint8_t _myArray[2];
  uint16_t _buffer_read = 0;
  float _buffer_read_f = 0.0;
  _myArray[0] = EEPROM.read(_address++);
  _myArray[1] = EEPROM.read(_address);
  _buffer_read = _myArray[0];
  _buffer_read = (_buffer_read << 8) | _myArray[1];
  _buffer_read_f = (float)_buffer_read/100.0;
  return _buffer_read_f; 
}

void KBOTSERIAL::initPIDParameter(PID_Data_Typedef *_pid_data, uint8_t _address){
    if((int)readFlashData(_address) > 650) writeFlashData(_address,0);
    _pid_data->kp = readFlashData(_address);
    if((int)readFlashData(_address += 2) > 650) writeFlashData(_address,0);
    _pid_data->kp = readFlashData(_address);
    if((int)readFlashData(_address += 2) > 650) writeFlashData(_address,0);
    _pid_data->kp = readFlashData(_address);
}