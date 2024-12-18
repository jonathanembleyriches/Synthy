
#pragma once

#include "CoreMinimal.h"
#include <functional>

#include "ROSIntegration/Classes/RI/Topic.h"
#include "ROSIntegration/Classes/ROSIntegrationGameInstance.h"
#include "ROSIntegration/Public/ROSTime.h"
#include "ROSIntegration/Public/geometry_msgs/Point.h"
#include "ROSIntegration/Public/geometry_msgs/Pose.h"
#include "ROSIntegration/Public/sensor_msgs/Image.h"
#include "ROSIntegration/Public/sensor_msgs/Imu.h"
#include "ROSIntegration/Public/sensor_msgs/JointState.h"
#include "ROSIntegration/Public/std_msgs/Float32MultiArray.h"

#include <mujoco/mujoco.h>
class RosManager {

public:
mjModel* model; mjData* data;
    RosManager(mjModel* model, mjData* data);

    void ReadAllSensorDataRos(TSharedPtr<ROSMessages::sensor_msgs::JointState> JointStateMsg,
                              TSharedPtr<ROSMessages::sensor_msgs::Imu> ImuMsg,
                              TSharedPtr<ROSMessages::std_msgs::Float32MultiArray> TouchForceMsg, FROSTime& ROSTime);

}; // namespace RosManager
