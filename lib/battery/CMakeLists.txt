FILE(GLOB_RECURSE sources *.c)
idf_component_register(
        SRCS ${sources}
        INCLUDE_DIRS . ${CMAKE_SOURCE_DIR}/include
        REQUIRES esp_adc_cal
)