
include("cmake/HunterGate.cmake")

HunterGate(
    URL "https://github.com/ruslo/hunter/archive/v0.19.244.tar.gz"
    SHA1 "5ce82630f1943ce11f02a4bbc3bbd352edddd1eb"
    LOCAL
)

PROJECT(mrpt-web)

CMAKE_MINIMUM_REQUIRED(VERSION 3.5)
set (CMAKE_CXX_STANDARD 14)  # Require C++14

find_package(MRPT REQUIRED core poses rtti slam )

find_package( Threads )

hunter_add_package(jsoncpp)
find_package(jsoncpp REQUIRED)

hunter_add_package(OpenSSL)
find_package(OpenSSL REQUIRED)

hunter_add_package(Boost COMPONENTS system)
find_package(Boost CONFIG REQUIRED system)
include_directories(${Boost_INCLUDE_DIRS})

find_program(JSON_RPC_STUB jsonrpcstub
HINT "${LIBJSON-RPC-CPP_ROOT}/bin/")

# Lib:
add_subdirectory(libs/web)
add_subdirectory(apps/mrpt-ws-rpc)
