cmake_minimum_required(VERSION 3.8)

enable_testing()

add_executable (KalmanGainTest KalmanGainTest.cpp)

add_test(NAME KalmanGainTest1 COMMAND KalmanGainTest)

add_library (KalmanFilter KalmanFilter.cpp KalmanGain.cpp)

target_link_libraries (KalmanGainTest KalmanFilter)

target_include_directories (KalmanFilter PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories (KalmanGainTest PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
