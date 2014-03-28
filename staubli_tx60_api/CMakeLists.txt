cmake_minimum_required(VERSION 2.4.6)


include_directories(include)

add_library(soapC src/soapC.cpp)
add_library(Server0 src/soapCS8ServerV0Proxy.cpp)
add_library(Server1 src/soapCS8ServerV1Proxy.cpp)
add_library(Server3 src/soapCS8ServerV3Proxy.cpp)
add_library(stdsoap2 src/stdsoap2.cpp)
add_library(tx60l src/TX60L.cpp)