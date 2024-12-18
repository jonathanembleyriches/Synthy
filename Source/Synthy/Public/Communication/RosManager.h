
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
    mjModel* model;
    mjData* data;

    UTopic* m_Topic_JointStateSub;

    UTopic* m_Topic_JointStatePub;

    UTopic* m_Topic_ImuPub;

    UTopic* m_Topic_CameraPub;

    UTopic* m_Topic_CameraDepthPub;

    UTopic* m_Topic_StartPosSub;

    UTopic* m_Topic_GoalPosSub;

    UTopic* m_Topic_ContactsPub;
    class UROSIntegrationGameInstance* m_ROSInst;
    RosManager(mjModel* model, mjData* data);

    void SetupPublishers();

    void ReadAllSensorDataRos(TSharedPtr<ROSMessages::sensor_msgs::JointState> JointStateMsg,
                              TSharedPtr<ROSMessages::sensor_msgs::Imu> ImuMsg,
                              TSharedPtr<ROSMessages::std_msgs::Float32MultiArray> TouchForceMsg, FROSTime& ROSTime);

    void SetupRos(UGameInstance* GameInstance);
    void Tick();

}; // namespace RosManager
