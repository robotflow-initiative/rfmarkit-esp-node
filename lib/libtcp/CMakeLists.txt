FILE(GLOB_RECURSE sources *.c)
idf_component_register(
    SRCS ${sources}
    INCLUDE_DIRS .
    REQUIRES lwip
)