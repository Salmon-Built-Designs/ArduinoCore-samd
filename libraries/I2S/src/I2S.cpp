#include <Arduino.h>
#include <wiring_private.h>

#include "I2S.h"

I2SClass::I2SClass(SERCOM *p_sercom, uint8_t uc_index, uint8_t uc_pinSD, uint8_t uc_pinSCK, uint8_t uc_pinFS) :
	_p_sercom(p_sercom),
	_uc_index(uc_index),
	_uc_sd(uc_pinSD),
	_uc_sck(uc_pinSCK),
	_uc_fs(uc_pinFS)
{
}

int I2SClass::begin(int mode, long sampleRate, int bitsPerSample, int driveClock)
{
	switch (mode) {
		case I2S_PHILIPS_MODE:
			break;

		case I2S_RIGHT_JUSTIFIED_MODE:
		case I2S_LEFT_JUSTIFIED_MODE:
		case I2S_DSP_MODE:
		default:
			Serial.println("invalid mode");
			return 1;
	}

	switch (bitsPerSample) {
		case 8:
		case 16:
		case 24:
		case 32:
			break;

		default:
			Serial.println("invalid bits per sample");
			return 1;
	}

	while(_i2s->SYNCBUSY.bit.SWRST);
	_i2s->CTRLA.bit.SWRST = 1;

	PM->APBCMASK.reg |= PM_APBCMASK_I2S;

	while (GCLK->STATUS.bit.SYNCBUSY);
	GCLK->GENDIV.bit.ID = GCLK_CLKCTRL_GEN_GCLK3_Val;
	GCLK->GENDIV.bit.DIV = SystemCoreClock / (sampleRate * 2 * bitsPerSample);

	while (GCLK->STATUS.bit.SYNCBUSY);
	GCLK->GENCTRL.bit.ID = GCLK_CLKCTRL_GEN_GCLK3_Val;
	GCLK->GENCTRL.bit.SRC = GCLK_GENCTRL_SRC_DFLL48M_Val;
	GCLK->GENCTRL.bit.IDC = 1;
	GCLK->GENCTRL.bit.GENEN = 1;

	while (GCLK->STATUS.bit.SYNCBUSY);
	GCLK->CLKCTRL.bit.ID = (_uc_index == 0) ? I2S_GCLK_ID_0 : I2S_GCLK_ID_1;
	GCLK->CLKCTRL.bit.GEN = GCLK_CLKCTRL_GEN_GCLK3_Val;
	GCLK->CLKCTRL.bit.CLKEN = 1;

	while (GCLK->STATUS.bit.SYNCBUSY);

	while(_i2s->SYNCBUSY.bit.ENABLE);
	_i2s->CTRLA.bit.ENABLE = 0;

	// TODO: change these based on mode and drive clock
	_i2s->CLKCTRL[_uc_index].bit.MCKOUTINV = 0;
	_i2s->CLKCTRL[_uc_index].bit.SCKOUTINV = 0;
	_i2s->CLKCTRL[_uc_index].bit.FSOUTINV = 0;
	_i2s->CLKCTRL[_uc_index].bit.MCKEN = 0;
	_i2s->CLKCTRL[_uc_index].bit.MCKSEL = I2S_CLKCTRL_MCKSEL_GCLK_Val;
	_i2s->CLKCTRL[_uc_index].bit.SCKSEL = I2S_CLKCTRL_SCKSEL_MCKDIV_Val;
	_i2s->CLKCTRL[_uc_index].bit.FSINV = 0;
	_i2s->CLKCTRL[_uc_index].bit.FSSEL = I2S_CLKCTRL_FSSEL_SCKDIV_Val;
	_i2s->CLKCTRL[_uc_index].bit.BITDELAY = I2S_CLKCTRL_BITDELAY_I2S_Val;
	_i2s->CLKCTRL[_uc_index].bit.MCKOUTDIV = 0;
	_i2s->CLKCTRL[_uc_index].bit.MCKDIV = 0;
	_i2s->CLKCTRL[_uc_index].bit.NBSLOTS = 1;

	switch (bitsPerSample) {
		case 32:
			_i2s->CLKCTRL[_uc_index].bit.SLOTSIZE = I2S_CLKCTRL_SLOTSIZE_32_Val;
			break;

		case 24:
			_i2s->CLKCTRL[_uc_index].bit.SLOTSIZE = I2S_CLKCTRL_SLOTSIZE_24_Val;
			break;

		case 16:
			_i2s->CLKCTRL[_uc_index].bit.SLOTSIZE = I2S_CLKCTRL_SLOTSIZE_16_Val;
			break;

		case 8:
			_i2s->CLKCTRL[_uc_index].bit.SLOTSIZE = I2S_CLKCTRL_SLOTSIZE_8_Val;
			break;
	}

	_i2s->CLKCTRL[_uc_index].bit.FSWIDTH = I2S_CLKCTRL_FSWIDTH_SLOT_Val;

	pinPeripheral(_uc_sck, PIO_COM);
	pinPeripheral(_uc_fs, PIO_COM);

	_i2s->SERCTRL[_uc_index].bit.RXLOOP = 0;
	_i2s->SERCTRL[_uc_index].bit.DMA = I2S_SERCTRL_DMA_SINGLE_Val;
	_i2s->SERCTRL[_uc_index].bit.MONO = I2S_SERCTRL_MONO_STEREO_Val;
	_i2s->SERCTRL[_uc_index].bit.SLOTDIS0 = 0;
	_i2s->SERCTRL[_uc_index].bit.SLOTDIS1 = 0;
	_i2s->SERCTRL[_uc_index].bit.SLOTDIS2 = 0;
	_i2s->SERCTRL[_uc_index].bit.SLOTDIS3 = 0;
	_i2s->SERCTRL[_uc_index].bit.SLOTDIS4 = 0;
	_i2s->SERCTRL[_uc_index].bit.SLOTDIS5 = 0;
	_i2s->SERCTRL[_uc_index].bit.SLOTDIS6 = 0;
	_i2s->SERCTRL[_uc_index].bit.SLOTDIS7 = 0;
	_i2s->SERCTRL[_uc_index].bit.BITREV = I2S_SERCTRL_BITREV_MSBIT_Val;
	_i2s->SERCTRL[_uc_index].bit.WORDADJ = I2S_SERCTRL_WORDADJ_RIGHT_Val;
	_i2s->SERCTRL[_uc_index].bit.SLOTADJ = I2S_SERCTRL_SLOTADJ_LEFT_Val;
	_i2s->SERCTRL[_uc_index].bit.TXSAME = I2S_SERCTRL_TXSAME_ZERO_Val; // I2S_SERCTRL_TXSAME_SAME_Val
	_i2s->SERCTRL[_uc_index].bit.CLKSEL = (_uc_index == 0) ? I2S_SERCTRL_CLKSEL_CLK0_Val : I2S_SERCTRL_CLKSEL_CLK1_Val;
	_i2s->SERCTRL[_uc_index].bit.SERMODE = I2S_SERCTRL_SERMODE_TX_Val;
	_i2s->SERCTRL[_uc_index].bit.TXDEFAULT = I2S_SERCTRL_TXDEFAULT_ZERO_Val;

	switch (bitsPerSample) {
		case 32:
			_i2s->SERCTRL[_uc_index].bit.DATASIZE = I2S_SERCTRL_DATASIZE_32_Val;
			break;

		case 24:
			_i2s->SERCTRL[_uc_index].bit.DATASIZE = I2S_SERCTRL_DATASIZE_24_Val;
			break;

		case 16:
			_i2s->SERCTRL[_uc_index].bit.DATASIZE = I2S_SERCTRL_DATASIZE_16_Val;
			break;

		case 8:
			_i2s->SERCTRL[_uc_index].bit.DATASIZE = I2S_SERCTRL_DATASIZE_8_Val;
			break;
	}

	_i2s->SERCTRL[_uc_index].bit.EXTEND = I2S_SERCTRL_EXTEND_ZERO_Val;

	pinPeripheral(_uc_sd, PIO_COM);

	while(_i2s->SYNCBUSY.bit.ENABLE);
	_i2s->CTRLA.bit.ENABLE = 1;

	if (_uc_index == 0) {
		while(_i2s->SYNCBUSY.bit.CKEN0);
		_i2s->CTRLA.bit.CKEN0 = 1;

		while(_i2s->SYNCBUSY.bit.SEREN0);
		_i2s->CTRLA.bit.SEREN0 = 1;
	} else {
		while(_i2s->SYNCBUSY.bit.CKEN1);
		_i2s->CTRLA.bit.CKEN1 = 1;

		while(_i2s->SYNCBUSY.bit.SEREN1);
		_i2s->CTRLA.bit.SEREN1 = 1;
	}

	return 0;
}

void I2SClass::end()
{

}

int I2SClass::available()
{
	return 0;
}

int I2SClass::read()
{
	return -1;
}

int I2SClass::peek()
{
	return -1;
}

void I2SClass::flush()
{
}

size_t I2SClass::write(uint8_t data)
{
	return write((int)data);
}

size_t I2SClass::write(const uint8_t *buffer, size_t size)
{
	return 0;
}

size_t I2SClass::availableForWrite()
{
	return 0;
}

int I2SClass::read(int8_t data[], int size)
{
	return 0;
}

int I2SClass::write(short data)
{
	return write((int)data);
}

int I2SClass::write(int data)
{
	if (_uc_index == 0) {
		while (!_i2s->INTFLAG.bit.TXRDY0);
		while (_i2s->SYNCBUSY.bit.DATA0);
	} else {
		while (!_i2s->INTFLAG.bit.TXRDY1);
		while (_i2s->SYNCBUSY.bit.DATA1);
	}

	_i2s->DATA[_uc_index].bit.DATA = data;

	if (_uc_index == 0) {
		_i2s->INTFLAG.bit.TXRDY0 = 1;
	} else {
		_i2s->INTFLAG.bit.TXRDY1 = 1;
	}

	return 1;
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

I2SClass I2S(&sercom2, 0, 9, 1, 0);
