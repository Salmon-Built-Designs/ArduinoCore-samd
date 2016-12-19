#include <Arduino.h>

#include <SD.h>
#include <FlashStorage.h>

#define OTA_START    0x2000
#define OTA_SIZE     0x8000

#define SKETCH_START (uint32_t*)(OTA_START + OTA_SIZE)

#ifndef SDCARD_SS_PIN
#define SDCARD_SS_PIN 4
#endif

#define UPDATE_FILE "UPDATE.BIN"

FlashClass flash;

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

    if (updateSize > OTA_SIZE) {
      // skip the OTA section
      updateFile.seek(OTA_SIZE);
      updateSize -= OTA_SIZE;


      uint32_t flashAddress = (uint32_t)SKETCH_START;

      // erase the pages
      Serial.println("Erasing sketch flash ...");
      flash.erase((void*)flashAddress, updateSize);

      uint8_t buffer[512];

      // write the pages
      Serial.println("Writing sketch to flash ...");
      for (uint32_t i = 0; i < updateSize; i += sizeof(buffer)) {
        updateFile.read(buffer, sizeof(buffer));

        flash.write((void*)flashAddress, buffer, sizeof(buffer));

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

