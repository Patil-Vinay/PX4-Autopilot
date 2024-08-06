############################################################################
#
#   Custom Driver for sending "Hello World" over CAN
#
############################################################################

px4_add_module(
    MODULE drivers__helloworld_can
    MAIN helloworld_can
    SRCS
        HelloWorldCan.cpp
    DEPENDS
        drivers__device
)
