#include "Utils/MJHelper.h"
FQuat MJHelper::MJQuatToUE(mjtNum* MjQuat) {

    FQuat quat;
    quat.W = MjQuat[0];
    quat.X = -MjQuat[1];
    quat.Y = MjQuat[2];
    quat.Z = -MjQuat[3];
    return quat;
}
FQuat MJHelper::MJQuatToUE(mjtNum* MjQuat, int Offset) {

    FQuat quat;
    quat.W = MjQuat[Offset + 0];
    quat.X = -MjQuat[Offset + 1];
    quat.Y = MjQuat[Offset + 2];
    quat.Z = -MjQuat[Offset + 3];
    return quat;
}


