FILE(GLOB_RECURSE main_sources ${CMAKE_SOURCE_DIR}/main/*.c)
FILE(GLOB_RECURSE apps_sources ${CMAKE_SOURCE_DIR}/components/apps/*.c)
FILE(GLOB_RECURSE blink_sources ${CMAKE_SOURCE_DIR}/components/blink/*.c)
FILE(GLOB_RECURSE device_sources ${CMAKE_SOURCE_DIR}/components/device/*.c)
FILE(GLOB_RECURSE gy95_sources ${CMAKE_SOURCE_DIR}/components/gy95/*.c)

idf_component_register(SRCS ${main_sources} ${apps_sources} ${blink_sources} ${device_sources} ${gy95_sources}
                       INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/include ${CMAKE_SOURCE_DIR}/components/gy95 ${CMAKE_SOURCE_DIR}/components/device ${CMAKE_SOURCE_DIR}/components/blink)
