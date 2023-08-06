#ifndef KBOT_SERIAL_PID_H__
#define KBOT_SERIAL_PID_H__

#include <Arduino.h>
#include <EEPROM.h>
#include "motor_control.h"

#define EEPROM_SIZE 12

class KBOTSERIAL{
    public:
        KBOTSERIAL();
        void begin();
        void initPIDParameter(PID_Data_Typedef *_pid_data,uint8_t _address);
        void writeFlashData(uint8_t _address,uint16_t _buffer_write);
        float readFlashData(uint8_t _address);
};

#endif