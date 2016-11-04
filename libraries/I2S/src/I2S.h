#ifndef _I2S_H_INCLUDED
#define _I2S_H_INCLUDED

#include <Arduino.h>

typedef enum {
  I2S_PHILIPS_MODE,
  I2S_RIGHT_JUSTIFIED_MODE,
  I2S_LEFT_JUSTIFIED_MODE,
  I2S_DSP_MODE
} i2s_mode_t;

class I2SClass : public Stream
{
public:
  I2SClass(SERCOM *p_sercom, uint8_t uc_index, uint8_t uc_pinSD, uint8_t uc_pinSCK, uint8_t uc_pinFS);

  int begin(int mode, long sampleRate, int bitsPerSample, int driveClock = 1);

  void end();

  virtual int available();
  virtual int read();
  virtual int peek();
  virtual void flush();

  virtual size_t write(uint8_t);
  virtual size_t write(const uint8_t *buffer, size_t size);

  virtual size_t availableForWrite();


  int write(short);
  int write(int);

  int read(int8_t data[], int size);
  int read(int16_t data[], int size);
  int read(int32_t data[], int size);

  int write(const int8_t data[], int size);
  int write(const int16_t data[], int size);
  int write(const int32_t data[], int size);

  void onReceive(void(*)(int));
  void onTransmit(void(*)(void));

private:
  volatile I2s *_i2s = I2S;

  SERCOM *_p_sercom;
  uint8_t _uc_index;
  uint8_t _uc_sd;
  uint8_t _uc_sck;
  uint8_t _uc_fs;
};

#undef I2S

extern I2SClass I2S;

#endif
