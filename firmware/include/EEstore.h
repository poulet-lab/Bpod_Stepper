/*
EEstore
Copyright (C) 2021 Florian Rau

This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, version 3.

This program is distributed  WITHOUT ANY WARRANTY and without even the implied
warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public License along with
this program.  If not, see <http://www.gnu.org/licenses/>.

_______________________________________________________________________________


REVISION HISTORY

version 1.0.0   initial release (thank you: Florian Uekermann)
version 1.0.1   default address: 0

_______________________________________________________________________________
*/

#pragma once

#include "Arduino.h"
#include <EEPROM.h>
#include <util/crc16.h>

template <class T>
class EEstore {
  EEstore(T &dataRef) {
    this->data = dataRef;
    this->crc  = calcCRC();
  }
  
  public:
    uint16_t crc;
    T data;

    uint16_t calcCRC() {
      uint16_t crc = 0;
      uint8_t* bytes = (uint8_t*) &this->data;
      for (size_t i = 0; i < sizeof(T); i++)
        crc = _crc16_update(crc,bytes[i]);
      return crc;
    }

    static void getOrDefault(const int address, T &dataRef) {
      EEstore<T> storage(dataRef);
      EEPROM.get(address,storage);
      if (storage.crc == storage.calcCRC())
        dataRef = storage.data;
      else
        EEstore<T>::set(address, dataRef);
    }

    static void getOrDefault(T &dataRef) {
      getOrDefault(0, dataRef);
    }

    static void set(const int address, T &dataRef) {
      EEstore<T> storage(dataRef);
      EEPROM.put(address,storage);
    }

    static void set(T &dataRef) {
      set(0, dataRef);
    }
};
