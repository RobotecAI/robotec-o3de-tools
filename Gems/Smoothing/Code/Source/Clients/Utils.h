
#pragma once
#include <AzFramework/Components/TransformComponent.h>
namespace SmoothingUtils
{

    //! Remove the tilt from the transform.
    //! The tilt is removed by projecting the transform basis of rotation matrix to the ground plane.
    //! @param transform The transform to remove the tilt.
    //! @return The transform without tilt.
    AZ::Transform RemoveTiltFromTransform(AZ::Transform transform);

}