#ifdef VEHICLE_DRIVERSET_HWREV1

#include <hwrev1_includes.h>
#define VEHICLE_DRIVER_MOTOR hw_rev_1_MotorDriver
#define VEHICLE_DRIVER_STEERING hw_rev_1_SteeringDriver
#define VEHICLE_DRIVER_TARGET_CONTROL hw_rev_1_TargetControl
#define VEHICLE_GET_CONFIG hwrev1_getConfig()

#endif