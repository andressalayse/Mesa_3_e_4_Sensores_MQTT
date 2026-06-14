# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "F:/Espressif/frameworks/esp-idf-v5.3.1/components/bootloader/subproject"
  "F:/Projetos/workspace/Mesa_3_e_4_Sensores_MQTT/build/bootloader"
  "F:/Projetos/workspace/Mesa_3_e_4_Sensores_MQTT/build/bootloader-prefix"
  "F:/Projetos/workspace/Mesa_3_e_4_Sensores_MQTT/build/bootloader-prefix/tmp"
  "F:/Projetos/workspace/Mesa_3_e_4_Sensores_MQTT/build/bootloader-prefix/src/bootloader-stamp"
  "F:/Projetos/workspace/Mesa_3_e_4_Sensores_MQTT/build/bootloader-prefix/src"
  "F:/Projetos/workspace/Mesa_3_e_4_Sensores_MQTT/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "F:/Projetos/workspace/Mesa_3_e_4_Sensores_MQTT/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "F:/Projetos/workspace/Mesa_3_e_4_Sensores_MQTT/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
