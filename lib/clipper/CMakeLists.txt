CMAKE_MINIMUM_REQUIRED(VERSION 2.6.12)
PROJECT(polyclipping)

SET(CMAKE_BUILD_TYPE "Release" CACHE STRING "Release type")
SET(BUILD_SHARED_LIBS ON CACHE BOOL
    "Build shared libraries (.dll/.so) instead of static ones (.lib/.a)")
ADD_LIBRARY(polyclipping clipper.cpp)

SET_TARGET_PROPERTIES(polyclipping PROPERTIES VERSION 22.0.0 SOVERSION 22 )
