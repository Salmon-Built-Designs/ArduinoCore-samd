#include <Arduino.h>

#include <SD.h>

#define OTA_START    0x2000
#define OTA_SIZE     0x8000

#define SKETCH_START (uint32_t*)(OTA_START + OTA_SIZE)

#ifndef SDCARD_SS_PIN
#define SDCARD_SS_PIN 4
#endif

#define UPDATE_FILE "UPDATE.BIN"

// Initialize C library
extern "C" void __libc_init_array(void);

int main() {
  init();

  __libc_init_array();

  Serial.begin(115200);
  Serial.println("OTA begin");

  if (SD.begin(SDCARD_SS_PIN) && SD.exists(UPDATE_FILE)) {
    File updateFile = SD.open(UPDATE_FILE);
    uint32_t updateSize = updateFile.size();
    bool updateFlashed = false;

    uint32_t PAGE_SIZE = 1 << (3 + NVMCTRL->PARAM.bit.PSZ);
    uint32_t PAGES = NVMCTRL->PARAM.bit.NVMP;
    uint32_t MAX_FLASH = PAGE_SIZE * PAGES;
    uint32_t ROW_SIZE = PAGE_SIZE * 4;

    if (updateSize > OTA_SIZE && updateSize < MAX_FLASH) {
      updateFile.seek(OTA_SIZE);

      updateSize -= OTA_SIZE;

      // erase the pages
      uint32_t flashAddress = (uint32_t)SKETCH_START;

      Serial.println("Erasing sketch flash ...");
      for (uint32_t i = 0; i < updateSize; i += ROW_SIZE) {
        NVMCTRL->ADDR.reg = (flashAddress) / 2;
        NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_ER;
        while (!NVMCTRL->INTFLAG.bit.READY);

        flashAddress += ROW_SIZE;
      }

      // write the pages
      __attribute__ ((aligned (16))) uint8_t buffer[PAGE_SIZE / 4];

      // Disable automatic page write
      NVMCTRL->CTRLB.bit.MANW = 1;

      Serial.println("Writing sketch to flash ...");
      flashAddress = (uint32_t)SKETCH_START;
      for (uint32_t i = 0; i < updateSize; i += sizeof(buffer)) {
        // Execute "PBC" Page Buffer Clear
        NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_PBC;
        while (NVMCTRL->INTFLAG.bit.READY == 0);

        updateFile.read(buffer, sizeof(buffer));

        volatile uint32_t *dst_addr = (volatile uint32_t *)flashAddress;
        const uint32_t *src_addr = (uint32_t *)buffer;

        uint32_t size = sizeof(buffer) / 4;
        while (size) {
          *dst_addr = *src_addr;

          dst_addr++;
          src_addr++;
          size--;
        }

        // Execute "WP" Write Page
        NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_WP;
        while (NVMCTRL->INTFLAG.bit.READY == 0);

        flashAddress += sizeof(buffer);
      }

      updateFlashed = true;
    } else {
      Serial.println("Invalid update file size!");
    }

    updateFile.close();

    if (updateFlashed) {
      SD.remove(UPDATE_FILE);
    }
  } else {
    Serial.println("No SD or update file!");
  }

  Serial.println("Launching sketch");
  Serial.flush();

  __set_MSP(*SKETCH_START);

  //Reset vector table address
  SCB->VTOR = ((uint32_t)(SKETCH_START) & SCB_VTOR_TBLOFF_Msk);

  // address of Reset_Handler is written by the linker at the beginning of the .text section (see linker script)
  uint32_t resetHandlerAddress = (uint32_t) * (SKETCH_START + 1);
  // jump to reset handler
  asm("bx %0"::"r"(resetHandlerAddress));
}

