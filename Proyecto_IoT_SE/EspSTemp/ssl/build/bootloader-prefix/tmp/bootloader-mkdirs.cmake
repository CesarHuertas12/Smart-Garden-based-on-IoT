# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/ESP-32/esp-idf/components/bootloader/subproject"
  "C:/Users/david/Documents/Proyecto_IoT_SE/EspSTemp/ssl/build/bootloader"
  "C:/Users/david/Documents/Proyecto_IoT_SE/EspSTemp/ssl/build/bootloader-prefix"
  "C:/Users/david/Documents/Proyecto_IoT_SE/EspSTemp/ssl/build/bootloader-prefix/tmp"
  "C:/Users/david/Documents/Proyecto_IoT_SE/EspSTemp/ssl/build/bootloader-prefix/src/bootloader-stamp"
  "C:/Users/david/Documents/Proyecto_IoT_SE/EspSTemp/ssl/build/bootloader-prefix/src"
  "C:/Users/david/Documents/Proyecto_IoT_SE/EspSTemp/ssl/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Users/david/Documents/Proyecto_IoT_SE/EspSTemp/ssl/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
