#include <Arduino.h>

#include "DMA.h"

DMAClass::DMAClass() :
  _beginCount(0),
  _channelMask(0)
{
  memset(_transferCompleteCallbacks, 0x00, sizeof(_transferCompleteCallbacks));
  memset(_transferErrorCallbacks, 0x00, sizeof(_transferErrorCallbacks));

  memset(_descriptors, 0x00, sizeof(_descriptors));
  memset(_descriptorsWriteBack, 0x00, sizeof(_descriptorsWriteBack));
}

DMAClass::~DMAClass()
{
}

void DMAClass::begin()
{
  if (_beginCount == 0) {
    PM->AHBMASK.bit.DMAC_ = 1;
    PM->APBBMASK.bit.DMAC_ = 1;

    DMAC->CTRL.bit.SWRST = 1;

    DMAC->BASEADDR.bit.BASEADDR = (uint32_t)_descriptors;
    DMAC->WRBADDR.bit.WRBADDR = (uint32_t)_descriptorsWriteBack;

    DMAC->CTRL.bit.LVLEN0 = 1;
    DMAC->CTRL.bit.LVLEN1 = 1;
    DMAC->CTRL.bit.LVLEN2 = 1;
    DMAC->CTRL.bit.LVLEN3 = 1;
    DMAC->CTRL.bit.DMAENABLE = 1;

    NVIC_EnableIRQ(DMAC_IRQn);
    NVIC_SetPriority(I2S_IRQn, (1 << __NVIC_PRIO_BITS) - 1);  /* set Priority */
  }

  _beginCount++;
}

void DMAClass::end()
{
  _beginCount--;

  if (_beginCount == 0) {
    NVIC_DisableIRQ(DMAC_IRQn);

    DMAC->CTRL.bit.DMAENABLE = 0;

    PM->APBBMASK.bit.DMAC_ = 0;
    PM->AHBMASK.bit.DMAC_ = 0;
  }
}

int DMAClass::allocateChannel()
{
  int channel = -1;

  for (int i = 0; i < NUM_DMA_CHANNELS; i++) {
    if ((_channelMask & (1 << i)) == 0) {
      _channelMask |= (1 << i);

      memset((void*)&_descriptors[i], 0x00, sizeof(_descriptors[i]));

      DMAC->CHID.bit.ID = i;
      DMAC->CHCTRLA.bit.ENABLE = 0;
      DMAC->CHCTRLA.bit.SWRST = 1;

      channel = i;
      break;
    }
  }

  return channel;
}

void DMAClass::freeChannel(int channel)
{
  DMAC->CHID.bit.ID = channel;
  DMAC->CHCTRLA.bit.ENABLE = 0;

  _channelMask &= (1 << channel);
}

void DMAClass::setPriorityLevel(int channel, int level)
{
  DMAC->CHID.bit.ID = channel;
  DMAC->CHCTRLB.bit.LVL = level;
}

void DMAClass::setTriggerSource(int channel, int source)
{
  DMAC->CHID.bit.ID = channel;
  DMAC->CHCTRLB.bit.TRIGSRC = source;

  if (DMAC->CHCTRLB.bit.TRIGSRC) {
    DMAC->CHCTRLB.bit.TRIGACT = DMAC_CHCTRLB_TRIGACT_BEAT_Val;
  }
}

void DMAClass::setTransferWidth(int channel, int transferWidth)
{
  switch (transferWidth) {
    case 8:
    default:
      _descriptors[channel].BTCTRL.bit.BEATSIZE = DMAC_BTCTRL_BEATSIZE_BYTE_Val;
      break;

    case 16:
      _descriptors[channel].BTCTRL.bit.BEATSIZE = DMAC_BTCTRL_BEATSIZE_HWORD_Val;
      break;

    case 32:
      _descriptors[channel].BTCTRL.bit.BEATSIZE = DMAC_BTCTRL_BEATSIZE_WORD_Val;
      break;
  }
}

void DMAClass::incSrc(int channel)
{
  _descriptors[channel].BTCTRL.bit.STEPSEL = DMAC_BTCTRL_STEPSEL_SRC_Val;
  _descriptors[channel].BTCTRL.bit.SRCINC = 1;
}

void DMAClass::incDst(int channel)
{
  _descriptors[channel].BTCTRL.bit.STEPSEL = DMAC_BTCTRL_STEPSEL_DST_Val;
  _descriptors[channel].BTCTRL.bit.DSTINC = 1;
}

int DMAClass::transfer(int channel, void* src, void* dst, uint16_t size)
{
  if (_descriptors[channel].BTCTRL.bit.VALID) {
    return 1;
  }

  DMAC->CHID.bit.ID = channel;

  _descriptors[channel].BTCTRL.bit.EVOSEL = DMAC_BTCTRL_EVOSEL_DISABLE_Val;
  _descriptors[channel].BTCTRL.bit.BLOCKACT = DMAC_BTCTRL_BLOCKACT_NOACT_Val;

  int transferWidth;

  switch (_descriptors[channel].BTCTRL.bit.BEATSIZE) {
    case DMAC_BTCTRL_BEATSIZE_BYTE_Val:
    default:
      transferWidth = 1;
      break;

    case DMAC_BTCTRL_BEATSIZE_HWORD_Val:
      transferWidth = 2;
      break;

    case DMAC_BTCTRL_BEATSIZE_WORD_Val:
      transferWidth = 4;
      break;
  }

  _descriptors[channel].BTCTRL.bit.STEPSIZE = DMAC_BTCTRL_STEPSIZE_X1_Val;

  _descriptors[channel].SRCADDR.bit.SRCADDR = (uint32_t)src;
  _descriptors[channel].DSTADDR.bit.DSTADDR = (uint32_t)dst;
  _descriptors[channel].DESCADDR.bit.DESCADDR = 0;
  _descriptors[channel].BTCNT.bit.BTCNT = size / transferWidth;

  if (_descriptors[channel].BTCTRL.bit.SRCINC) {
    _descriptors[channel].SRCADDR.bit.SRCADDR += size;
  }

  if (_descriptors[channel].BTCTRL.bit.DSTINC) {
    _descriptors[channel].DSTADDR.bit.DSTADDR += size;
  }

  _descriptors[channel].BTCTRL.bit.VALID = 1;

  DMAC->CHINTENSET.bit.TERR = 1;
  DMAC->CHINTENSET.bit.TCMPL = 1;
  DMAC->CHINTENSET.bit.SUSP = 1;
  DMAC->CHCTRLA.bit.ENABLE = 1;


  if (DMAC->CHCTRLB.bit.TRIGSRC == 0) {
    // software trigger
    DMAC->SWTRIGCTRL.reg |= (1 << channel);
  }

  return 0;
}

void DMAClass::onTransferComplete(int channel, void(*function)(void))
{
  _transferCompleteCallbacks[channel] = function;
}

void DMAClass::onTransferError(int channel, void(*function)(void))
{
  _transferErrorCallbacks[channel] = function;
}

void DMAClass::onService()
{
  int channel = DMAC->INTPEND.bit.ID;

  DMAC->CHID.bit.ID = channel;

  _descriptors[channel].BTCTRL.bit.VALID = 0;

  if (DMAC->CHINTFLAG.bit.TERR) {
    DMAC->CHINTFLAG.bit.TERR = 1;

    if (_transferErrorCallbacks[channel]) {
      _transferErrorCallbacks[channel]();
    }
  }

  if (DMAC->CHINTFLAG.bit.TCMPL) {
    DMAC->CHINTFLAG.bit.TCMPL = 1;

    if (_transferCompleteCallbacks[channel]) {
      _transferCompleteCallbacks[channel]();
    }
  }

  if (DMAC->CHINTFLAG.bit.SUSP) {
    DMAC->CHINTFLAG.bit.SUSP = 1;
  }
}

extern "C" {
  void DMAC_Handler() {
    DMA.onService();
  }
}

DMAClass DMA;
