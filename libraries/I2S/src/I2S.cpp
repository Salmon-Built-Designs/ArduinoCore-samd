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

  _state(I2S_STATE_IDLE),
  _dmaChannel(-1),
  _bitsPerSample(0),
  _dmaTransferInProgress(false),

  _onTransmit(NULL),
  _onReceive(NULL)
{
}

int I2SClass::begin(int mode, long sampleRate, int bitsPerSample)
{
  return begin(mode, sampleRate, bitsPerSample, true);
}

int I2SClass::begin(int mode, int bitsPerSample)
{
  return begin(mode, 0, bitsPerSample, false);
}

int I2SClass::begin(int mode, long sampleRate, int bitsPerSample, bool driveClock)
{
  if (_state != I2S_STATE_IDLE) {
    return 1;
  }

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
      _bitsPerSample = bitsPerSample;
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

  if (driveClock) {
    enableClock(sampleRate * 2 * bitsPerSample);

    i2sd.setSerialClockSelectMasterClockDiv(_deviceIndex);
    i2sd.setFrameSyncSelectSerialClockDiv(_deviceIndex);
  } else {
    i2sd.setSerialClockSelectPin(_deviceIndex);
    i2sd.setFrameSyncSelectPin(_deviceIndex);
  }

  i2sd.disable();
  i2sd.set1BitDelay(_deviceIndex);
  i2sd.setNumberOfSlots(_deviceIndex, 1);
  i2sd.setSlotSize(_deviceIndex, bitsPerSample);

  pinPeripheral(_sckPin, PIO_COM);
  pinPeripheral(_fsPin, PIO_COM);

  i2sd.setSlotAdjustedLeft(_deviceIndex);
  if (driveClock) {
    i2sd.setClockUnit(_deviceIndex);
  }

  pinPeripheral(_sdPin, PIO_COM);

  i2sd.enable();

  _doubleBuffer.reset();

  return 0;
}

void I2SClass::end()
{
  if (_dmaChannel > -1) {
    DMA.freeChannel(_dmaChannel);
  }

  _state = I2S_STATE_IDLE;
  _dmaTransferInProgress = false;

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
  if (_state != I2S_STATE_RECEIVER) {
    enableReceiver();
  }

  uint8_t enableInterrupts = ((__get_PRIMASK() & 0x1) == 0);
  size_t avail;

  // disable interrupts,
  __disable_irq();

  avail = _doubleBuffer.available();

  if (_dmaTransferInProgress == false && _doubleBuffer.available() == 0) {
    _dmaTransferInProgress = true;

    DMA.transfer(_dmaChannel, i2sd.data(_deviceIndex), _doubleBuffer.data(), _doubleBuffer.availableForWrite());

    _doubleBuffer.swap();
  }

  if (enableInterrupts) {
    // re-enable the interrupts
    __enable_irq();
  }

  return avail;
}

int I2SClass::read()
{
  int sample = 0;

  read(&sample, _bitsPerSample / 8);

  if (_bitsPerSample == 16 && (sample & 0x8000)) {
    // sign extend value
    sample |= 0xffff0000;
  }

  return sample;
}

int I2SClass::peek()
{
  uint8_t enableInterrupts = ((__get_PRIMASK() & 0x1) == 0);
  int sample = 0;

  // disable interrupts,
  __disable_irq();

  _doubleBuffer.peek(&sample, _bitsPerSample / 8);

  if (enableInterrupts) {
    // re-enable the interrupts
    __enable_irq();
  }

  if (_bitsPerSample == 16 && (sample & 0x8000)) {
    // sign extend value
    sample |= 0xffff0000;
  }

  return sample;
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
  if (_state != I2S_STATE_TRANSMITTER) {
    enableTransmitter();
  }

  uint8_t enableInterrupts = ((__get_PRIMASK() & 0x1) == 0);
  size_t space;

  // disable interrupts,
  __disable_irq();

  space = _doubleBuffer.availableForWrite();

  if (enableInterrupts) {
    // re-enable the interrupts
    __enable_irq();
  }

  return space;
}

int I2SClass::read(void* buffer, size_t size)
{
  if (_state != I2S_STATE_RECEIVER) {
    enableReceiver();
  }

  uint8_t enableInterrupts = ((__get_PRIMASK() & 0x1) == 0);

  // disable interrupts,
  __disable_irq();

  int read = _doubleBuffer.read(buffer, size);

  if (_dmaTransferInProgress == false && _doubleBuffer.available() == 0) {
    _dmaTransferInProgress = true;

    DMA.transfer(_dmaChannel, i2sd.data(_deviceIndex), _doubleBuffer.data(), _doubleBuffer.availableForWrite());

    _doubleBuffer.swap();
  }

  if (enableInterrupts) {
    // re-enable the interrupts
    __enable_irq();
  }

  return read;
}

size_t I2SClass::write(int sample)
{
  return write((int32_t)sample);
}

size_t I2SClass::write(int32_t sample)
{
  if (_state != I2S_STATE_TRANSMITTER) {
    enableTransmitter();
  }

  while(!i2sd.txReady(_deviceIndex));

  i2sd.writeData(_deviceIndex, sample);

  i2sd.clearTxReady(_deviceIndex);

  return 1;
}

size_t I2SClass::write(const void *buffer, size_t size)
{
  if (_state != I2S_STATE_TRANSMITTER) {
    enableTransmitter();
  }

  uint8_t enableInterrupts = ((__get_PRIMASK() & 0x1) == 0);
  size_t written;

  // disable interrupts,
  __disable_irq();

  written = _doubleBuffer.write(buffer, size);

  if (_dmaTransferInProgress == false && _doubleBuffer.available()) {
    _dmaTransferInProgress = true;

    DMA.transfer(_dmaChannel, _doubleBuffer.data(), i2sd.data(_deviceIndex), _doubleBuffer.available());

    _doubleBuffer.swap();
  }

  if (enableInterrupts) {
    // re-enable the interrupts
    __enable_irq();
  }

  return written;
}

void I2SClass::onTransmit(void(*function)(void))
{
  _onTransmit = function;
}

void I2SClass::onReceive(void(*function)(void))
{
  _onReceive = function;
}

void I2SClass::enableClock(int divider)
{
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

void I2SClass::disableClock()
{
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

void I2SClass::enableTransmitter()
{
  i2sd.setTxMode(_deviceIndex);
  i2sd.enableClockUnit(_deviceIndex);
  i2sd.enableSerializer(_deviceIndex);

  DMA.incSrc(_dmaChannel);
  DMA.onTransferComplete(_dmaChannel, I2SClass::onDmaTransferComplete);
  DMA.setTriggerSource(_dmaChannel, i2sd.dmaTriggerSource(_deviceIndex));
  DMA.setTransferWidth(_dmaChannel, _bitsPerSample);

  _state = I2S_STATE_TRANSMITTER;
}

void I2SClass::enableReceiver()
{
  i2sd.setRxMode(_deviceIndex);
  i2sd.enableClockUnit(_deviceIndex);
  i2sd.enableSerializer(_deviceIndex);

  DMA.incDst(_dmaChannel);
  DMA.onTransferComplete(_dmaChannel, I2SClass::onDmaTransferComplete);
  DMA.setTriggerSource(_dmaChannel, i2sd.dmaTriggerSource(_deviceIndex));
  DMA.setTransferWidth(_dmaChannel, _bitsPerSample);

  _state = I2S_STATE_RECEIVER;
}

void I2SClass::onTransferComplete(void)
{
  if (_state == I2S_STATE_TRANSMITTER) {
    if (_doubleBuffer.available()) {
      DMA.transfer(_dmaChannel, _doubleBuffer.data(), i2sd.data(_deviceIndex), _doubleBuffer.available());

      _doubleBuffer.swap();
    } else {
      _dmaTransferInProgress = false;
    }

    if (_onTransmit) {
      _onTransmit();
    }
  } else {
    if (_doubleBuffer.available() == 0) {
      DMA.transfer(_dmaChannel, i2sd.data(_deviceIndex), _doubleBuffer.data(), _doubleBuffer.availableForWrite());

      _doubleBuffer.swap(_doubleBuffer.availableForWrite());
    } else {
      _dmaTransferInProgress = false;
    }

    if (_onReceive) {
      _onReceive();
    }
  }
}

void I2SClass::onDmaTransferComplete(int channel)
{
#if I2S_INTERFACES_COUNT > 0
  if (I2S._dmaChannel == channel) {
    I2S.onTransferComplete();
  }
#endif
}

#if I2S_INTERFACES_COUNT > 0
I2SClass I2S(I2S_DEVICE, I2S_CLOCK_GENERATOR, PIN_I2S_SD, PIN_I2S_SCK, PIN_I2S_FS);
#endif
