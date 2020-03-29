project(mqtt_pid_pwm)

cmake_minimum_required(VERSION 3.13)

set (CMAKE_CXX_STANDARD 14)
set( CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR} /usr/local/lib/cmake/PahoMqttCpp)
FIND_PACKAGE( Boost 1.67 COMPONENTS program_options thread REQUIRED )
FIND_PACKAGE( PahoMqttCpp REQUIRED )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )

ADD_EXECUTABLE( mqtt_sub_example mqtt_sub_example.cpp )
ADD_EXECUTABLE( mqtt_pid_pwm mqtt_pid_pwm.cpp )

TARGET_LINK_LIBRARIES( mqtt_sub_example PahoMqttCpp::paho-mqttpp3 )
TARGET_LINK_LIBRARIES( mqtt_pid_pwm  Boost::program_options PahoMqttCpp::paho-mqttpp3 )
