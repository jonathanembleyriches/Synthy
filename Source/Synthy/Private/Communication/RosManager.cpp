#include "Communication/RosManager.h"

RosManager::RosManager(mjModel* model, mjData* data, TMap<FString, float>* ActuatorValues) {
    this->model = model;
    this->data = data;
    m_ActuatorValues = ActuatorValues;
}

void RosManager::SubCallback_JointState(TSharedPtr<FROSBaseMsg> Msg) {
    // Cast the message to a JointState message
    auto CastResponse = StaticCastSharedPtr<ROSMessages::sensor_msgs::JointState>(Msg);
    if (!CastResponse) {
        return;
    }

    // Iterate through the joint names in the message
    for (int i = 0; i < CastResponse->name.Num(); i++) {
        // Append "_motor" to the joint name to form the key
        FString JointNameWithMotor = CastResponse->name[i] + "_motor";

        // if (i >= 6)
        //     continue;

        // UE_LOG(LogTemp, Warning, TEXT("joint state %s : %f"), *JointNameWithMotor, CastResponse->position[i]);
        // Check if the key exists in the map
        if (m_ActuatorValues->Contains(JointNameWithMotor)) {

            // m_ActuatorValues[JointNameWithMotor] = CastResponse->position[i];
            (*m_ActuatorValues)[JointNameWithMotor] = CastResponse->position[i];
        }
    }
}

void RosManager::SetupJointStateSub() {
    m_Topic_JointStateSub = NewObject<UTopic>(UTopic::StaticClass());
    UROSIntegrationGameInstance* rosinst = Cast<UROSIntegrationGameInstance>(m_ROSInst);
    m_Topic_JointStateSub->Init(rosinst->ROSIntegrationCore, TEXT("/joint_states"), TEXT("sensor_msgs/JointState"));

    std::function<void(TSharedPtr<FROSBaseMsg>)> callback = [this](TSharedPtr<FROSBaseMsg> msg) {
        this->SubCallback_JointState(msg);
    };

    m_Topic_JointStateSub->Subscribe(callback);
}

void RosManager::PublishJointState(TSharedPtr<ROSMessages::sensor_msgs::JointState> JointStateMsg, FROSTime RosTime) {
    // Publish the JointState message
    if (m_Topic_JointStatePub && m_ROSInst->bIsConnected) {
        JointStateMsg->header.time = RosTime;
        m_Topic_JointStatePub->Publish(JointStateMsg);
    }
}

void RosManager::PublishImu(TSharedPtr<ROSMessages::sensor_msgs::Imu> ImuMsg, FROSTime RosTime) {
    // Publish the Imu message
    if (m_Topic_ImuPub && m_ROSInst->bIsConnected) {

        ImuMsg->header.time = RosTime;
        // UE_LOG(LogTemp, Warning, TEXT("Should be publishign imu %f"), ImuMsg->orientation.x);
        TArray<double> _temp;
        _temp.Init(0.0, 9);
        ImuMsg->orientation_covariance = _temp;
        ImuMsg->angular_velocity_covariance = _temp;
        ImuMsg->linear_acceleration_covariance = _temp;
        m_Topic_ImuPub->Publish(ImuMsg);
    }
}

void RosManager::PublishCamera(const TArray<FColor>& OutBMP, uint32 Width, uint32 Height, FROSTime RosTime) {

    if (!m_Topic_CameraPub)
        return;
    if (!m_ROSInst->bIsConnected)
        return;
    TSharedPtr<ROSMessages::sensor_msgs::Image> OutRosImage(new ROSMessages::sensor_msgs::Image());

    OutRosImage->header.time = RosTime;
    // Set image width and height
    OutRosImage->width = Width;
    OutRosImage->height = Height;

    // Set encoding to RGB8 (since FColor contains R, G, B, A, we only use R, G, B)
    OutRosImage->encoding = "rgb8"; // 8-bit RGB

    // Step size: width * number of channels (3 for RGB)
    OutRosImage->step = Width * 3;

    // is_bigendian: Usually set to 0 for little-endian
    OutRosImage->is_bigendian = 0;

    // Prepare the data array
    int32 ImageSize = Width * Height * 3; // Each pixel is 3 bytes (R, G, B)
    uint8* ImageData = new uint8[ImageSize];

    for (int32 i = 0; i < OutBMP.Num(); i++) {
        // Copy RGB data from FColor (ignore the Alpha channel)
        ImageData[i * 3 + 0] = OutBMP[i].R; // Red
        ImageData[i * 3 + 1] = OutBMP[i].G; // Green
        ImageData[i * 3 + 2] = OutBMP[i].B; // Blue
    }

    // Set the pointer to the data (make sure this memory stays valid)
    OutRosImage->data = ImageData;

    m_Topic_CameraPub->Publish(OutRosImage);
}

void RosManager::PublishDepth(const TArray<FFloat16Color>& DepthData, uint32 Width, uint32 Height, FROSTime RosTime) {

    TSharedPtr<ROSMessages::sensor_msgs::Image> OutRosImage(new ROSMessages::sensor_msgs::Image());
    if (!m_Topic_CameraDepthPub)
        return;

    if (!m_ROSInst->bIsConnected)
        return;
    OutRosImage->header.time = RosTime;
    // Set image width and height
    OutRosImage->width = Width;
    OutRosImage->height = Height;

    // Set encoding to mono8 (grayscale, 8-bit per pixel)
    OutRosImage->encoding = "mono8"; // 8-bit grayscale

    // Step size: width * 1 (1 byte per pixel for grayscale)
    OutRosImage->step = Width;

    // is_bigendian: Usually set to 0 for little-endian
    OutRosImage->is_bigendian = 0;

    // Prepare the data array for grayscale image
    int32 ImageSize = Width * Height; // Each pixel is 1 byte for grayscale
    uint8* ImageData = new uint8[ImageSize];

    // Normalize depth values to [0, 255] range for grayscale
    float MinDepth = FLT_MAX;
    float MaxDepth = -FLT_MAX;

    // Find min and max depth values to normalize depth data
    for (int32 i = 0; i < DepthData.Num(); i++) {
        float DepthValue = DepthData[i].R; // Typically depth is stored in the R channel
        if (DepthValue < MinDepth)
            MinDepth = DepthValue;
        if (DepthValue > MaxDepth)
            MaxDepth = DepthValue;
    }

    // Avoid divide-by-zero if all depth values are the same
    float DepthRange = (MaxDepth - MinDepth) > 0.0f ? (MaxDepth - MinDepth) : 1.0f;

    // Convert depth to 8-bit grayscale
    for (int32 i = 0; i < DepthData.Num(); i++) {
        float DepthValue = DepthData[i].R; // Depth is stored in the R channel
        uint8 GrayscaleValue = static_cast<uint8>(255.0f * (DepthValue - MinDepth) / DepthRange);
        ImageData[i] = GrayscaleValue;
    }

    // Set the pointer to the data (make sure this memory stays valid)
    OutRosImage->data = ImageData;

    m_Topic_CameraDepthPub->Publish(OutRosImage);
}

void RosManager::PublishContacts(TSharedPtr<ROSMessages::std_msgs::Float32MultiArray> Msg, FROSTime RosTime) {
    // Publish the JointState message
    if (m_Topic_ContactsPub && m_ROSInst->bIsConnected) {
        m_Topic_ContactsPub->Publish(Msg);
    }
}

void RosManager::SetupPublishers() {

    if (!m_ROSInst->bIsConnected)
        return;
    m_Topic_ImuPub = NewObject<UTopic>(UTopic::StaticClass());

    m_Topic_ImuPub->Init(m_ROSInst->ROSIntegrationCore, TEXT("/current_imu"), TEXT("sensor_msgs/Imu"));

    m_Topic_ImuPub->Advertise();

    m_Topic_JointStatePub = NewObject<UTopic>(UTopic::StaticClass());
    m_Topic_JointStatePub->Init(m_ROSInst->ROSIntegrationCore, TEXT("/current_joint_state"), TEXT("sensor_msgs/JointState"));
    m_Topic_JointStatePub->Advertise();

    m_Topic_CameraPub = NewObject<UTopic>(UTopic::StaticClass());
    m_Topic_CameraPub->Init(m_ROSInst->ROSIntegrationCore, TEXT("/current_image"), TEXT("sensor_msgs/Image"));
    m_Topic_CameraPub->Advertise();

    m_Topic_CameraDepthPub = NewObject<UTopic>(UTopic::StaticClass());
    m_Topic_CameraDepthPub->Init(m_ROSInst->ROSIntegrationCore, TEXT("/current_depth"), TEXT("sensor_msgs/Image"));
    m_Topic_CameraDepthPub->Advertise();

    m_Topic_ContactsPub = NewObject<UTopic>(UTopic::StaticClass());
    m_Topic_ContactsPub->Init(m_ROSInst->ROSIntegrationCore, TEXT("/current_foot_contacts"), TEXT("std_msgs/Float32MultiArray"));
    m_Topic_ContactsPub->Advertise();
}
void RosManager::SetupRos(UGameInstance* GameInstance) {

    m_ROSInst = Cast<UROSIntegrationGameInstance>(GameInstance);
    UE_LOG(LogTemp, Warning, TEXT("New ros server host is %s"), *m_ROSInst->ROSBridgeServerHost);
    // ROSInst->bConnectToROS = true;
    m_ROSInst->ConnectToRos();

    if (!m_ROSInst->bIsConnected) {

        UE_LOG(LogTemp, Warning, TEXT("ROS NOT CONNECTED"));
        return;
    }

    UE_LOG(LogTemp, Warning, TEXT("ROS should be CONNECTED"));
    SetupPublishers();
    SetupJointStateSub();
    // SetupStartPosSub();
    // SetupGoalPosSub();
}

void RosManager::Publish() {

    TSharedPtr<ROSMessages::sensor_msgs::JointState> JointStateMsg = MakeShareable(new ROSMessages::sensor_msgs::JointState());
    TSharedPtr<ROSMessages::sensor_msgs::Imu> ImuMsg = MakeShareable(new ROSMessages::sensor_msgs::Imu());
    TSharedPtr<ROSMessages::std_msgs::Float32MultiArray> TouchForceMsg = MakeShareable(new ROSMessages::std_msgs::Float32MultiArray());
    FROSTime RosTime;
    ReadAllSensorDataRos(JointStateMsg, ImuMsg, TouchForceMsg, RosTime);
    // PublishImu(ImuMsg, RosTime);
    // PublishJointState(JointStateMsg, RosTime);
    // PublishContacts(TouchForceMsg, RosTime);
    //
    // FTextureRenderTargetResource* RTResourceRGB = RenderTargetRGB->GameThread_GetRenderTargetResource();
    // TArray<FColor> OutBMP;
    // RTResourceRGB->ReadPixels(OutBMP);
    // PublishCamera(OutBMP, 1024, 1024, RosTime);
    //
    // FTextureRenderTargetResource* RTResourceDepth = RenderTargetDepth->GameThread_GetRenderTargetResource();
    // TArray<FFloat16Color> DepthData;
    // RTResourceDepth->ReadFloat16Pixels(DepthData);
    // PublishDepth(DepthData, 1024, 1024, RosTime);
}
void RosManager::ReadAllSensorDataRos(TSharedPtr<ROSMessages::sensor_msgs::JointState> JointStateMsg,
                                      TSharedPtr<ROSMessages::sensor_msgs::Imu> ImuMsg,
                                      TSharedPtr<ROSMessages::std_msgs::Float32MultiArray> TouchForceMsg, FROSTime& ROSTime) {
    // Check if Model and Data are valid
    if (!model || !data)
        return;
    TouchForceMsg->data.Init(0.0f, 4); // Initialize with 4 elements, all set to 0.0f

    // Loop through all sensors
    for (int i = 0; i < model->nsensor; i++) {
        // Get sensor name
        const char* sensor_name = mj_id2name(model, mjOBJ_SENSOR, i);
        if (!sensor_name)
            sensor_name = "Unnamed";

        // Get sensor type
        int sensor_type = model->sensor_type[i];
        int sensor_adr = model->sensor_adr[i];
        int sensor_dim = model->sensor_dim[i];

        // Sort sensor data into relevant categories
        for (int j = 0; j < sensor_dim; j++) {
            float sensor_value = data->sensordata[sensor_adr + j];

            if (sensor_type == mjSENS_ACTUATORPOS) {
                JointStateMsg->name.Add(sensor_name);      // Add joint name
                JointStateMsg->position.Add(sensor_value); // Add joint position
            } else if (sensor_type == mjSENS_ACTUATORVEL) {
                JointStateMsg->velocity.Add(sensor_value); // Add joint velocity
            } else if (sensor_type == mjSENS_ACTUATORFRC) {
                JointStateMsg->effort.Add(sensor_value); // Add joint effort
            } else if (sensor_type == mjSENS_ACCELEROMETER) {

                // UE_LOG(LogTemp, Log, TEXT("Should be adding data to imu msg %f"), data->sensordata[sensor_adr]);
                ImuMsg->linear_acceleration.x = data->sensordata[sensor_adr];
                ImuMsg->linear_acceleration.y = data->sensordata[sensor_adr + 1];
                ImuMsg->linear_acceleration.z = data->sensordata[sensor_adr + 2];
            } else if (sensor_type == mjSENS_GYRO) {
                ImuMsg->angular_velocity.x = data->sensordata[sensor_adr];
                ImuMsg->angular_velocity.y = data->sensordata[sensor_adr + 1];
                ImuMsg->angular_velocity.z = data->sensordata[sensor_adr + 2];
            } else if (sensor_type == mjSENS_FRAMEQUAT) {
                ImuMsg->orientation.x = data->sensordata[sensor_adr];
                ImuMsg->orientation.y = data->sensordata[sensor_adr + 1];
                ImuMsg->orientation.z = data->sensordata[sensor_adr + 2];
                ImuMsg->orientation.w = data->sensordata[sensor_adr + 3];

            } else if (sensor_type == mjSENS_CLOCK) {

                float mujocoTime = data->sensordata[sensor_adr];
                unsigned long seconds = static_cast<unsigned long>(mujocoTime);
                unsigned long nanoseconds = static_cast<unsigned long>((mujocoTime - seconds) * 1e9);
                ROSTime._Sec = seconds;
                ROSTime._NSec = nanoseconds;
            }

            else if (sensor_type == mjSENS_TOUCH || sensor_type == mjSENS_FORCE) {
                // Map specific sensor names to positions in the array
                FString SensorNameString(sensor_name);
                if (SensorNameString == "contact_sensor_FL") {
                    TouchForceMsg->data[0] = sensor_value; // Front Left sensor
                } else if (SensorNameString == "contact_sensor_FR") {
                    TouchForceMsg->data[1] = sensor_value; // Front Right sensor
                } else if (SensorNameString == "contact_sensor_RL") {
                    TouchForceMsg->data[2] = sensor_value; // Rear Left sensor
                } else if (SensorNameString == "contact_sensor_RR") {
                    TouchForceMsg->data[3] = sensor_value; // Rear Right sensor
                }
            }
        }
    }
}
// void RosManager::SubCallback_StartPos(TSharedPtr<FROSBaseMsg> Msg) {
//     // Cast to Pose message
//     auto CastResponse = StaticCastSharedPtr<ROSMessages::geometry_msgs::Pose>(Msg);
//     if (!CastResponse) {
//         return;
//     }
//
//     // Set position from the Pose message
//     m_StartPos.X = CastResponse->position.x;
//     m_StartPos.Y = CastResponse->position.y;
//     m_StartPos.Z = CastResponse->position.z;
//
//     // Set orientation from the Pose message (quaternion)
//     m_Orientation.X = CastResponse->orientation.x;
//     m_Orientation.Y = CastResponse->orientation.y;
//     m_Orientation.Z = CastResponse->orientation.z;
//     m_Orientation.W = CastResponse->orientation.w;
//
//     // Get the body ID of the trunk
//     int body_id = mj_name2id(model, mjOBJ_BODY, "trunk");
//     if (body_id == -1) {
//         // UE_LOG(LogTemp, Warning, TEXT("BODY ID NOT VALID"));
//         return;
//     }
//
//     // Get the joint ID for the free joint associated with the trunk
//     int trunk_joint_id = model->body_jntadr[body_id];
//     if (trunk_joint_id >= 0 && model->jnt_type[trunk_joint_id] == mjJNT_FREE) {
//         // The body has a free joint (6 DOF), so set both the position and orientation
//
//         // Set position (first 3 elements of qpos for the free joint)
//         data->qpos[model->jnt_qposadr[trunk_joint_id] + 0] = m_StartPos.X; // Set x
//         data->qpos[model->jnt_qposadr[trunk_joint_id] + 1] = m_StartPos.Y; // Set y
//         data->qpos[model->jnt_qposadr[trunk_joint_id] + 2] = m_StartPos.Z; // Set z
//
//         // Set orientation (next 4 elements of qpos for quaternion)
//         data->qpos[model->jnt_qposadr[trunk_joint_id] + 3] = m_Orientation.W; // w
//         data->qpos[model->jnt_qposadr[trunk_joint_id] + 4] = m_Orientation.X; // x
//         data->qpos[model->jnt_qposadr[trunk_joint_id] + 5] = m_Orientation.Y; // y
//         data->qpos[model->jnt_qposadr[trunk_joint_id] + 6] = m_Orientation.Z; // z
//
//         // UE_LOG(LogTemp, Warning, TEXT("Set position to (%f, %f, %f) and orientation to (%f, %f, %f, %f)"), m_StartPos.X,
//         // m_StartPos.Y,
//         // m_StartPos.Z, m_Orientation.W, m_Orientation.X, m_Orientation.Y, m_Orientation.Z);
//     }
// }
//
// void AMyPhysicsWorldActor::SetupStartPosSub() {
//     m_Topic_StartPosSub = NewObject<UTopic>(UTopic::StaticClass());
//     UROSIntegrationGameInstance* rosinst = Cast<UROSIntegrationGameInstance>(GetGameInstance());
//     m_Topic_StartPosSub->Init(rosinst->ROSIntegrationCore, TEXT("/start_pos"), TEXT("geometry_msgs/Pose"));
//
//     std::function<void(TSharedPtr<FROSBaseMsg>)> callback = [this](TSharedPtr<FROSBaseMsg> msg) {
//         this->SubCallback_StartPos(msg);
//     };
//
//     m_Topic_StartPosSub->Subscribe(callback);
// }
//
// void AMyPhysicsWorldActor::SubCallback_GoalPos(TSharedPtr<FROSBaseMsg> Msg) {
//     // In here we need to apply the torque to the actuators
//
//     auto CastResponse = StaticCastSharedPtr<ROSMessages::geometry_msgs::Pose>(Msg);
//     if (!CastResponse) {
//         return;
//     }
//
//     // Set position from the Pose message
//     FVector GoalPos{CastResponse->position.x * 100, -CastResponse->position.y * 100, CastResponse->position.z * 100};
//
//     // UE_LOG(LogTemp, Warning, TEXT("Setting goalpos to %s"), *GoalPos.ToString());
//     if (m_GoalActor) {
//
//         // UE_LOG(LogTemp, Warning, TEXT("S actor validetting goalpos to %s"), *GoalPos.ToString());
//         m_GoalActor->SetActorLocation(GoalPos);
//     }
// }
// void AMyPhysicsWorldActor::SetupGoalPosSub() {
//     m_Topic_GoalPosSub = NewObject<UTopic>(UTopic::StaticClass());
//     UROSIntegrationGameInstance* rosinst = Cast<UROSIntegrationGameInstance>(GetGameInstance());
//     m_Topic_GoalPosSub->Init(rosinst->ROSIntegrationCore, TEXT("/goal_pos"), TEXT("geometry_msgs/Pose"));
//
//     std::function<void(TSharedPtr<FROSBaseMsg>)> callback = [this](TSharedPtr<FROSBaseMsg> msg) {
//         this->SubCallback_GoalPos(msg);
//     };
//
//     m_Topic_GoalPosSub->Subscribe(callback);
// }
