FILE(GLOB_RECURSE sources *.c)
idf_component_register(
        SRCS ${sources}
        INCLUDE_DIRS . ${CMAKE_SOURCE_DIR}/include
        REQUIRES blink esp_https_ota nvs_flash ring_buf app_update libudp bt ble rest_controller spatial battery esp_wifi
)