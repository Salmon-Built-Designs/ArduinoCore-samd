/*
  Copyright (c) 2016 Arduino LLC.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <Arduino.h>
#include <wiring_private.h>

#include "utility/DMA.h"
#include "utility/SAMD21_I2SDevice.h"

static I2SDevice_SAMD21G18x i2sd(*I2S);

#include "I2S.h"

int I2SClass::_beginCount = 0;

I2SClass::I2SClass(uint8_t deviceIndex, uint8_t clockGenerator, uint8_t sdPin, uint8_t sckPin, uint8_t fsPin) :
  _deviceIndex(deviceIndex),
  _clockGenerator(clockGenerator),
  _sdPin(sdPin),
  _sckPin(sckPin),
  _fsPin(fsPin),

  _dmaChannel(-1),

  _onTransmit(NULL)
{
}

int I2SClass::begin(int mode, long sampleRate, int bitsPerSample, int driveClock)
{
  switch (mode) {
    case I2S_PHILIPS_MODE:
      break;

    default:
      return 1;
  }

  switch (bitsPerSample) {
    case 8:
    case 16:
    case 32:
      break;

    default:
      return 1;
  }

  DMA.begin();

  _dmaChannel = DMA.allocateChannel();

  if (_dmaChannel < 0) {
    return 1;
  }

  if (_beginCount == 0) {
    // enable the I2S interface
    PM->APBCMASK.reg |= PM_APBCMASK_I2S;

    // reset the device
    i2sd.reset();
  }

  _beginCount++;

  enableClock(sampleRate * 2 * bitsPerSample);

  i2sd.disable();
  i2sd.set1BitDelay(_deviceIndex);
  i2sd.setNumberOfSlots(_deviceIndex, 1);
  i2sd.setSlotSize(_deviceIndex, bitsPerSample);

  pinPeripheral(_sckPin, PIO_COM);
  pinPeripheral(_fsPin, PIO_COM);

  i2sd.setSlotAdjustedLeft(_deviceIndex);
  i2sd.setClockUnit(_deviceIndex);
  i2sd.setTxMode(_deviceIndex);

  pinPeripheral(_sdPin, PIO_COM);

  i2sd.enable();
  i2sd.enableClockUnit(_deviceIndex);
  i2sd.enableSerializer(_deviceIndex);

  DMA.incSrc(_dmaChannel);
  DMA.onTransferComplete(_dmaChannel, I2SClass::onDmaTransferComplete);
  DMA.onTransferError(_dmaChannel, I2SClass::onDmaTransferError);
  DMA.setTriggerSource(_dmaChannel, i2sd.dmaTriggerSource(_deviceIndex));
  DMA.setTransferWidth(_dmaChannel, bitsPerSample);

  return 0;
}

void I2SClass::end()
{
  if (_dmaChannel > 0) {
    DMA.freeChannel(_dmaChannel);
  }

  i2sd.disableSerializer(_deviceIndex);
  i2sd.disableClockUnit(_deviceIndex);

  pinMode(_sdPin, INPUT);
  pinMode(_fsPin, INPUT);
  pinMode(_sckPin, INPUT);


  disableClock();

  _beginCount--;

  if (_beginCount == 0) {
    i2sd.disable();

    // disable the I2S interface
    PM->APBCMASK.reg &= ~PM_APBCMASK_I2S;
  }
}

int I2SClass::available()
{
  return 0;
}

int I2SClass::read()
{
  return 0;
}

int I2SClass::peek()
{
  return 0;
}

void I2SClass::flush()
{
}

size_t I2SClass::write(uint8_t data)
{
  return write((int32_t)data);
}

size_t I2SClass::write(const uint8_t *buffer, size_t size)
{
  return write((const void*)buffer, size);
}

size_t I2SClass::availableForWrite()
{
  return 2;
}

size_t I2SClass::write(int sample)
{
  return write((int32_t)sample);
}

size_t I2SClass::write(int32_t sample)
{
  while(!i2sd.txReady(_deviceIndex));

  i2sd.writeData(_deviceIndex, sample);

  i2sd.clearTxReady(_deviceIndex);

  return 1;
}

size_t I2SClass::write(const void *buffer, size_t size)
{
  return 0;
}

void I2SClass::onTransmit(void(*function)(void))
{
  _onTransmit = function;
}

void I2SClass::enableClock(int divider) {
  while (GCLK->STATUS.bit.SYNCBUSY);
  GCLK->GENDIV.bit.ID = _clockGenerator;
  GCLK->GENDIV.bit.DIV = SystemCoreClock / divider;

  while (GCLK->STATUS.bit.SYNCBUSY);
  GCLK->GENCTRL.bit.ID = _clockGenerator;
  GCLK->GENCTRL.bit.SRC = GCLK_GENCTRL_SRC_DFLL48M_Val;
  GCLK->GENCTRL.bit.IDC = 1;
  GCLK->GENCTRL.bit.GENEN = 1;

  while (GCLK->STATUS.bit.SYNCBUSY);
  GCLK->CLKCTRL.bit.ID = i2sd.glckId(_deviceIndex);
  GCLK->CLKCTRL.bit.GEN = _clockGenerator;
  GCLK->CLKCTRL.bit.CLKEN = 1;

  while (GCLK->STATUS.bit.SYNCBUSY);
}

void I2SClass::disableClock() {
  while (GCLK->STATUS.bit.SYNCBUSY);
  GCLK->GENCTRL.bit.ID = _clockGenerator;
  GCLK->GENCTRL.bit.SRC = GCLK_GENCTRL_SRC_DFLL48M_Val;
  GCLK->GENCTRL.bit.IDC = 1;
  GCLK->GENCTRL.bit.GENEN = 0;

  while (GCLK->STATUS.bit.SYNCBUSY);
  GCLK->CLKCTRL.bit.ID = i2sd.glckId(_deviceIndex);
  GCLK->CLKCTRL.bit.GEN = _clockGenerator;
  GCLK->CLKCTRL.bit.CLKEN = 0;

  while (GCLK->STATUS.bit.SYNCBUSY);
}

void I2SClass::onDmaTransferComplete(int channel)
{
  if (I2S._dmaChannel == channel) {
    I2S.onTransferComplete();
  }
}

void I2SClass::onDmaTransferError(int channel)
{
  if (I2S._dmaChannel == channel) {
    I2S.onTransferError();
  }
}

void I2SClass::onTransferComplete(void)
{
}

void I2SClass::onTransferError(void)
{
}

/*

+--------+--------------+-----------+-----------------+
| I2S    | Pad          | Zero      | MKR1000         |
+--------+--------------+-----------+-----------------+
| SD[0]  | PA07 or PA19 | 9 or 12   | A6 or 10 (MISO) |
| MCK[0] | PA09 or PB17 | 3 or ?    | 12 (SCL) or ?   |
| SCK[0] | PA10 or PA20 | 1 or 6    | 2 or 6          |
| FS[0]  | PA11 or PA21 | 0 or ?    | 3 or 7          |
+--------+--------------------------+-----------------+
| SD[1]  | PA08 or PB16 | 4 or ?    | 11 (SDA) or ?   |
| MCK[1] | PB10         | 23 (MOSI) | 4               |
| SCK[1] | PB11         | 24 (SCK)  | 5               |
| FS[1]  | PB12         | ?         | ?               |
+--------+--------------+-----------+-----------------+
*/


I2SClass I2S(0, GCLK_CLKCTRL_GEN_GCLK3_Val, 9, 1, 0);

// I2SClass I2S(0, GCLK_CLKCTRL_GEN_GCLK3_Val, A6, 2, 3);
