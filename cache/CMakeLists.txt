project(cache)
add_library(cache SHARED cache.c csim.h)
target_include_directories(cache PRIVATE ../common)
