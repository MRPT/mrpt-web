
cmake_minimum_required(VERSION 3.0)
include("cmake/HunterGate.cmake")

HunterGate(
    URL "https://github.com/ruslo/hunter/archive/v0.19.244.tar.gz"
    SHA1 "5ce82630f1943ce11f02a4bbc3bbd352edddd1eb"
    LOCAL
)

PROJECT(mrpt-web)

CMAKE_MINIMUM_REQUIRED(VERSION 3.5)
set (CMAKE_CXX_STANDARD 14)  # Require C++14

find_package(MRPT REQUIRED)

find_package( Threads )

hunter_add_package(libjson-rpc-cpp)
find_package(libjson-rpc-cpp CONFIG REQUIRED)

hunter_add_package(Boost COMPONENTS system)
find_package(Boost CONFIG REQUIRED system)

find_program(JSON_RPC_STUB jsonrpcstub
HINT "${LIBJSON-RPC-CPP_ROOT}/bin/")

# Lib:
add_subdirectory(libs/web)
add_subdirectory(apps/mrpt-ws-rpc)
