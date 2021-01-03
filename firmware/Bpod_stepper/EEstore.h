/*
EEstore
Copyright (C) 2021 Florian Rau

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, version 3.

This program is distributed  WITHOUT ANY WARRANTY and without even the
implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef EEstore_h
#define EEstore_h

#include "Arduino.h"
#include <EEPROM.h>
#include <util/crc16.h>

template <class T>

class EEstore {
  public:

    EEstore(T *data) {
      EEstore(0,data);
    }

    EEstore(const uint16_t address, T *data) {
      _addressCRC  = address;
      _addressSize = address + 2;
      _addressData = address + 4;
      _data = data;
      _size = sizeof(*_data);
      loadData();
    }

    void write() {
      EEPROM.put(_addressData,*_data);
      EEPROM.put(_addressSize,_size);
      EEPROM.put(_addressCRC,crc16());
    }

  private:
    T* _data;
    uint16_t _addressCRC;
    uint16_t _addressSize;
    uint16_t _addressData;
    uint16_t _size;

    void loadData() {
      uint16_t crc;
      bool crcValid, sizeValid;
      
      crcValid  = EEPROM.get(_addressCRC,crc) == crc16();
      sizeValid = EEPROM.get(_addressSize,_size) == _size;
      if (crcValid && sizeValid) {
        EEPROM.get(_addressData,*_data);
      } else {
        write();
      }
    }
  
    uint16_t crc16() {
      uint16_t i, crc = 0;
      for (i = _addressSize; i < _addressData + _size; i++) {
        crc = _crc16_update(crc, EEPROM.read(i));
      }
      return crc;
    }
};
 
#endif
