#!/bin/sh -x

ARDUINO=arduino
SKETCH_NAME="OTA_SD.ino"
SKETCH="$PWD/$SKETCH_NAME"
BUILD_PATH="$PWD/build"
OUTPUT_PATH="../../src/boot"

if [[ "$OSTYPE" == "darwin"* ]]; then
	ARDUINO="/Applications/Arduino.app/Contents/MacOS/Arduino"
fi

buildOTASketch() {
	BOARD=$1
	DESTINATION=$2

	$ARDUINO --verify --board $BOARD --preserve-temp-files --pref build.path="$BUILD_PATH" $SKETCH
	cat "$BUILD_PATH/$SKETCH_NAME.bin" | xxd -i > $DESTINATION
	rm -rf "$BUILD_PATH"
}

mkdir -p "$OUTPUT_PATH"

buildOTASketch "arduino:samd:arduino_zero_edbg" "$OUTPUT_PATH/zero.h"
buildOTASketch "arduino:samd:mkr1000" "$OUTPUT_PATH/mkr1000.h"
buildOTASketch "arduino:samd:mkrzero" "$OUTPUT_PATH/mkrzero.h"
