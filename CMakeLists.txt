cmake_minimum_required(VERSION 3.16)

project(ServoDaemon LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Core)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Core Network)

include(FetchContent)
FetchContent_Declare(
    spdlog
    GIT_REPOSITORY https://github.com/gabime/spdlog.git
    GIT_TAG        v1.14.1
)
FetchContent_MakeAvailable(spdlog)

add_executable(ServoDaemon
  main.cpp
  ecatserver.h ecatserver.cpp
  ecatmaster.h ecatmaster.cpp
  ecatmanager.h ecatmanager.cpp
  servood.h
  slave.h
  servol7nh.h
  servol7nh.cpp
  servoconfig.h servoconfig.cpp
  configloader.h configloader.cpp
  windowsservice.h windowsservice.cpp
  Logger.h Logger.cpp
)
target_link_libraries(ServoDaemon PRIVATE
  Qt${QT_VERSION_MAJOR}::Core
  Qt${QT_VERSION_MAJOR}::Network
  spdlog::spdlog
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

target_link_libraries(${PROJECT_NAME} PRIVATE
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

if(WIN32)
    target_link_libraries(${PROJECT_NAME} PRIVATE Advapi32)

    # windeployqt를 찾아 빌드 후 자동으로 Qt 라이브러리를 배포하도록 설정
    get_target_property(_qmake_loc Qt${QT_VERSION_MAJOR}::qmake IMPORTED_LOCATION)
    get_filename_component(_qt_bin_dir "${_qmake_loc}" DIRECTORY)
    find_program(WINDEPLOYQT_EXECUTABLE windeployqt HINTS "${_qt_bin_dir}")

    if(WINDEPLOYQT_EXECUTABLE)
        add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
            COMMAND "${WINDEPLOYQT_EXECUTABLE}"
                    --no-compiler-runtime
                    --verbose 0
                    "$<TARGET_FILE:${PROJECT_NAME}>"
            COMMENT "Deploying Qt dependencies for ${PROJECT_NAME} using windeployqt..."
        )
    endif()
endif()

include(GNUInstallDirs)
install(TARGETS ServoDaemon
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
