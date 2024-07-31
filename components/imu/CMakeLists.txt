idf_component_register(
        INCLUDE_DIRS . ${CMAKE_SOURCE_DIR}/include
        REQUIRES sys $ENV{IMU_MODULE}
)