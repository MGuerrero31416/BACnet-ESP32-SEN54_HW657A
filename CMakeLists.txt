# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

set(EXCLUDE_COMPONENTS TFT_eSPI)

# MS/TP timing experiment compile-time tunables
set(USER_MSTP_PFM_REPLY_PRE_DELAY_US "200" CACHE STRING "Pre-reply silence before Reply-To-PFM transmit (microseconds)" FORCE)
set(USER_RS485_DE_PRE_TX_US "500" CACHE STRING "DE setup delay before UART write for Reply-To-PFM (microseconds)" FORCE)
set(USER_MSTP_ACTIVE_DEBUG_ONLY "1" CACHE STRING "Enable temporary minimal MS/TP active-debug logs")
set(USER_MSTP_TOKEN_PASS_ONLY_DEBUG "1" CACHE STRING "Emergency token-pass-only mode for MS/TP active debugging")
set(USER_MSTP_DEBUG_FORCE_NEXT_STATION "32" CACHE STRING "Forced next station in token-pass-only mode")
set(USER_MSTP_TOKEN_PASS_PRE_DELAY_US "0" CACHE STRING "Pre-delay before token pass TX in token-pass-only debug mode (microseconds)")
add_compile_definitions(
	USER_MSTP_PFM_REPLY_PRE_DELAY_US=${USER_MSTP_PFM_REPLY_PRE_DELAY_US}
	USER_RS485_DE_PRE_TX_US=${USER_RS485_DE_PRE_TX_US}
	USER_MSTP_ACTIVE_DEBUG_ONLY=${USER_MSTP_ACTIVE_DEBUG_ONLY}
	USER_MSTP_TOKEN_PASS_ONLY_DEBUG=${USER_MSTP_TOKEN_PASS_ONLY_DEBUG}
	USER_MSTP_DEBUG_FORCE_NEXT_STATION=${USER_MSTP_DEBUG_FORCE_NEXT_STATION}
	USER_MSTP_TOKEN_PASS_PRE_DELAY_US=${USER_MSTP_TOKEN_PASS_PRE_DELAY_US}
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# "Trim" the build. Include the minimal set of components, main, and anything it depends on.
#idf_build_set_property(MINIMAL_BUILD ON)
project(BACnet-ESP32-SEN54_HW657A)
