cmake_minimum_required(VERSION 3.16)

project(ServoDaemon LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Core)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Core Network)

add_executable(ServoDaemon
  main.cpp
  ecatserver.h ecatserver.cpp
  ecatmaster.h ecatmaster.cpp
  ecatmanager.h ecatmanager.cpp
  cia402.h
  slave.h
  servol7nh.h
  servol7nh.cpp
)
target_link_libraries(ServoDaemon PRIVATE
  Qt${QT_VERSION_MAJOR}::Core
  Qt${QT_VERSION_MAJOR}::Network
)

# region soem
set(SOEM_DIR "D:/gneetech_lib/SOEM")

target_include_directories(${PROJECT_NAME} PRIVATE
    ${SOEM_DIR}/soem
    ${SOEM_DIR}/osal
    ${SOEM_DIR}/osal/win32
    ${SOEM_DIR}/oshw/win32
    ${SOEM_DIR}/oshw/win32/wpcap/include
    ${SOEM_DIR}/soem
)

target_link_directories(${PROJECT_NAME} PRIVATE
    ${SOEM_DIR}/build
    ${SOEM_DIR}/oshw/win32/wpcap/Lib/x64
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    soem
    Packet
    wpcap
    winmm
    ws2_32
)

target_compile_definitions(${PROJECT_NAME} PRIVATE
    __STDC_LIMIT_MACROS
    __STDC_CONSTANT_MACROS
    WIN32_LEAN_AND_MEAN
)
# endregion

include(GNUInstallDirs)
install(TARGETS ServoDaemon
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
