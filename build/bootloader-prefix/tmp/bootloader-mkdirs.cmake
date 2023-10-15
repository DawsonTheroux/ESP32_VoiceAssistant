# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "D:/Espressif/frameworks/esp-idf-v5.1.1/components/bootloader/subproject"
  "D:/Projects/ESP32/hello_freertos/build/bootloader"
  "D:/Projects/ESP32/hello_freertos/build/bootloader-prefix"
  "D:/Projects/ESP32/hello_freertos/build/bootloader-prefix/tmp"
  "D:/Projects/ESP32/hello_freertos/build/bootloader-prefix/src/bootloader-stamp"
  "D:/Projects/ESP32/hello_freertos/build/bootloader-prefix/src"
  "D:/Projects/ESP32/hello_freertos/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/Projects/ESP32/hello_freertos/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "D:/Projects/ESP32/hello_freertos/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
