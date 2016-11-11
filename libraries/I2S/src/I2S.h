#ifndef _I2S_H_INCLUDED
#define _I2S_H_INCLUDED

#include <Arduino.h>

typedef enum {
  I2S_PHILIPS_MODE,
  I2S_RIGHT_JUSTIFIED_MODE,
  I2S_LEFT_JUSTIFIED_MODE,
  I2S_DSP_MODE
} i2s_mode_t;

#define I2S_BUFFER_SIZE 512

class I2SClass : public Stream
{
public:
  I2SClass(uint8_t uc_index, uint8_t uc_clock_generator, uint8_t uc_pinSD, uint8_t uc_pinSCK, uint8_t uc_pinFS);

  int begin(int mode, long sampleRate, int bitsPerSample, int driveClock = 1);

  void end();

  virtual int available();
  virtual int read();
  virtual int peek();
  virtual void flush();

  virtual size_t write(uint8_t);
  virtual size_t write(const uint8_t *buffer, size_t size);

  virtual size_t availableForWrite();

  size_t write(int);
  size_t write(int32_t);
  size_t write(const void *buffer, size_t size);

  void onReceive(void(*)(int));
  void onTransmit(void(*)(void));

private:
  static void onDmaTransferComplete();
  static void onDmaTransferError();

  void onTransferComplete(void);
  void onTransferError(void);

private:
  volatile I2s *_i2s = I2S;

  uint8_t _uc_index;
  uint8_t _uc_clock_generator;
  uint8_t _uc_sd;
  uint8_t _uc_sck;
  uint8_t _uc_fs;

  int _i_dma_channel;

  volatile bool _b_dma_transfer_in_progress;
  uint8_t* _auc_buffer[2][I2S_BUFFER_SIZE];
  volatile int _i_buffer_length[2];
  volatile int _i_buffer_index;

  void (*_onTransmit)(void);
};

#undef I2S

extern I2SClass I2S;

#endif
