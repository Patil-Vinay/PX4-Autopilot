############################################################################
# 
#   Custom Driver for sending VehicleAngularVelocity Z component over CAN
#
############################################################################

px4_add_module(
    MODULE drivers__vehicle_angular_velocity_can
    MAIN vehicle_angular_velocity_can
    SRCS
        VehicleAngularVelocityCan.cpp
)

