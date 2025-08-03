#ifdef VEHICLE_DRIVERSET_HWREV1

#include <hwrev1_includes.h>
#define VEHICLE_DRIVER_MOTOR hw_rev_1_MotorDriver
#define VEHICLE_DRIVER_STEERING hw_rev_1_SteeringDriver
#define VEHICLE_DRIVER_TARGET_CONTROL hw_rev_1_TargetControl
#define VEHICLE_GET_CONFIG hwrev1_getConfig()

#endif

#ifdef VEHICLE_DRIVERSET_HWREV2

#include <hwrev2_includes.h>
#define VEHICLE_DRIVER_IMU hw_rev_2_imu
#define VEHICLE_DRIVER_LIDAR hw_rev_2_lidar
#define VEHICLE_DRIVER_SPEED hw_rev_2_VehicleSpeed
#define VEHICLE_DRIVER_MOTOR hw_rev_2_MotorDriver
#define VEHICLE_DRIVER_STEERING hw_rev_2_SteeringDriver
#define VEHICLE_DRIVER_TARGET_CONTROL hw_rev_2_TargetControl
#define VEHICLE_GET_CONFIG hwrev2_getConfig()

#endif