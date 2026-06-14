# CMakeLists.txt (dentro da pasta main)

idf_component_register(
    SRCS
        "main.c"
    INCLUDE_DIRS
        "."
        #"${CMAKE_CURRENT_LIST_DIR}/../managed_components/espressif__esp-zigbee-lib/include"
        #"${CMAKE_CURRENT_LIST_DIR}/../managed_components/espressif__esp-zboss-lib/include"
        "${CMAKE_CURRENT_LIST_DIR}/../components/SensorVL53L0/include"
    REQUIRES
        #espressif__esp-zigbee-lib
        #espressif__esp-zboss-lib
        driver
        esp_timer
        nvs_flash
        esp_event
        log
        esp_wifi
        esp_netif
        mqtt 
        SensorVL53L0
)                       