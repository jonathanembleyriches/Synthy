#include "Communication/RosManager.h"

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
}
