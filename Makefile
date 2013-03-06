SOURCES := fhcore.cpp libs/Adafruit_BMP/Adafruit_BMP085.cpp
LIBRARIES := Wire EEPROM Time
BOARD := mega2560
TARGET := fhcore
include ./arduino.mk