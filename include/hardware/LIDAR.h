#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <TFLI2C.h>


class LIDAR
{
public:
    explicit LIDAR(TwoWire &wirePort = Wire2)
      : _wire(&wirePort) {}


    void setup()
    {   
        _wire->begin();
        if(!_tfluna.Set_Frame_Rate( _tfFrame, _addr)){
            _tfluna.printStatus();
        }
    }

    void reset(){
        _tfluna.Soft_Reset(_addr);
    }

    void update(){
        if (_tfluna.getData(_distance, _signalStrength, _temperature, _addrRecieved)){// writes to the variables
            _tfluna.Get_Time(tfTime, _addr); // updates internal clock
            _hasData = true;
        } else {
            _tfluna.printStatus();         
            _hasData = false;
        }
    }

    int16_t getDistance(){ return _distance; }       // in cm
    int16_t getSignalStrength(){ return _signalStrength; } // in flux
    int16_t getTemperature(){ return _temperature; }     // in deg C
    uint16_t getTime(){ return tfTime; }          // in ms
    bool hasData(){ return _hasData; }

private:
    uint8_t _addr;
    uint16_t _tfFrame = TFL_DEF_FPS;   // default frame rate
    uint16_t _addr = TFL_DEF_ADR;    // default I2C address

    uint8_t _addrRecieved;
    int16_t _distance;
    int16_t _signalStrength;
    int16_t _temperature;
    uint16_t tfTime{0};    // device clock in milliseconds
    bool _hasData{false};

    TwoWire *_wire;
    TFLI2C _tfluna;
};