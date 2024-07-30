
#include "Utils.h"
#include <AzCore/Math/Matrix3x3.h>

AZ::Transform SmoothingUtils::RemoveTiltFromTransform(AZ::Transform transform)
{
    const AZ::Vector3 axisX = transform.GetBasisX();
    const AZ::Vector3 axisY = transform.GetBasisY();

    const AZ::Matrix3x3 projectionOnXY {AZ::Matrix3x3::CreateFromColumns(AZ::Vector3::CreateAxisX(), AZ::Vector3::CreateAxisY(), AZ::Vector3::CreateZero())};

    const AZ::Vector3 newAxisZ = AZ::Vector3::CreateAxisZ(); // new axis Z points up

    // project axisX on the XY plane
    const AZ::Vector3 projectedAxisX = (projectionOnXY * axisX);
    const AZ::Vector3 projectedAxisY = (projectionOnXY * axisY);

    AZ::Vector3 newAxisX = AZ::Vector3::CreateZero();
    AZ::Vector3 newAxisY = AZ::Vector3::CreateZero();

    // get 3rd vector of the new basis from the cross product of the projected vectors.
    // Primarily we want to use the projectedAxisX as the newAxisX, but if it is zero-length, we use the projectedAxisY as the newAxisY.
    if (!projectedAxisX.IsZero())
    {
        newAxisX = projectedAxisX.GetNormalized();
        newAxisY = newAxisZ.Cross(newAxisX);
    }
    else
    {
        newAxisY = projectedAxisY.GetNormalized();
        newAxisX = newAxisY.Cross(newAxisZ);
    }
    // apply rotation using created basis
    transform.SetRotation(AZ::Quaternion::CreateFromBasis(newAxisX, newAxisY, newAxisZ));
    return transform;
}