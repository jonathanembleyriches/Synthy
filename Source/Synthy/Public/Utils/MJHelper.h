
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Kismet/GameplayStatics.h"
#include <functional>
#include <mujoco/mujoco.h>
namespace MJHelper {

struct HeightFieldData {

    bool valid = false;
    int mjXSize;
    int mjYSize;
    int mjHeight;
    int mjXPos;
    int mjYPos;
    int mjZPos;
};
FQuat MJQuatToUE(mjtNum* MjQuat);

FQuat MJQuatToUE(mjtNum* MjQuat, int Offset);
// Function to find all actuators and their most parent body
} // namespace MJHelper
