#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <TFLI2C.h>


class LIDAR
{
public:
    explicit LIDAR(TwoWire &wirePort = Wire2, uint8_t drdy_pin = 26)
      : _wire(&wirePort), _drdy_pin(drdy_pin) {}


    void setup()
    {   
        _wire->begin();
        _wire->setClock(400000); // 400 kHz
        pinMode(_drdy_pin, INPUT);
        
        _tfluna.setWirePort(*_wire); // <-- IMPORTANT: make library use Wire2

        if(!_tfluna.Set_Frame_Rate( _tfFrame, _addr)){
            _tfluna.printStatus();
        }
        _tfluna.Set_Cont_Mode(_addr); // set to continuous mode
        _tfluna.Save_Settings(_addr); // save frame rate setting
        delay(100);
    }

    void reset(){
        _tfluna.Soft_Reset(_addr);
    }

    void update(){
        if (digitalRead(_drdy_pin) == HIGH){

            if (_tfluna.getData(_distance, _signalStrength, _temperature, _addr)){// writes to the variables
                _tfluna.Get_Time(_tfTime, _addr); // updates internal clock
                _hasData = true;
            } else {
                _tfluna.printStatus();         
                _hasData = false;
            }
        } else {
            _hasData = false;
        }
    }

    int16_t getDistance(){ return _distance; }       // in cm
    int16_t getSignalStrength(){ return _signalStrength; } // in flux
    int16_t getTemperature(){ return _temperature; }     // in deg C / 100
    uint16_t getTime(){ return _tfTime; }          // in ms
    bool hasData(){ return _hasData; }

private:

   
    uint16_t _tfFrame = TFL_DEF_FPS;   // default frame rate
    uint16_t _addr = TFL_DEF_ADR;    // default I2C address

    int16_t _distance;
    int16_t _signalStrength;
    int16_t _temperature;
    uint16_t _drdyTimeout = 50; // ms
    uint16_t _tfTime; // ms
    bool _hasData = false;

    TwoWire *_wire;
    uint8_t _drdy_pin;
    TFLI2C _tfluna;
};