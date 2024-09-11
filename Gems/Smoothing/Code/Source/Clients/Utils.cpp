
#include "Utils.h"
#include <AzCore/Math/Matrix3x3.h>

AZ::Transform SmoothingUtils::RemoveTiltFromTransform(AZ::Transform transform)
{
    const AZ::Vector3 axisX = transform.GetBasisX();
    const AZ::Vector3 axisY = transform.GetBasisY();

    const AZ::Matrix3x3 projectionOnXY{ AZ::Matrix3x3::CreateFromColumns(
        AZ::Vector3::CreateAxisX(), AZ::Vector3::CreateAxisY(), AZ::Vector3::CreateZero()) };

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

AZ::Vector3 SmoothingUtils::AverageVector(const AZStd::deque<AZStd::pair<AZ::Vector3, float>>& buffer)
{
    AZ::Vector3 sum{ 0 };
    float normalization{ 0 };
    for (const auto& p : buffer)
    {
        sum += p.first * p.second;
        normalization += p.second;
    }
    return sum / normalization;
}

AZ::Vector3 SmoothingUtils::SmoothTranslation(const SmoothingUtils::SmoothingCache& cache)
{
    return SmoothingUtils::AverageVector(cache.m_lastTranslationsBuffer);
}

AZ::Quaternion SmoothingUtils::SmoothRotation(const SmoothingUtils::SmoothingCache& cache)
{
    AZ::Quaternion q = cache.m_lastRotationsBuffer.front().first;
    for (int i = 1; i < cache.m_lastRotationsBuffer.size(); i++)
    {
        q = q.Slerp(cache.m_lastRotationsBuffer[i].first, cache.m_lastRotationsBuffer[i].second);
    }
    return q;
}

void SmoothingUtils::CacheTransform(
    SmoothingUtils::SmoothingCache& cache, const AZ::Transform& transform, float deltaTime, int smoothBufferLen)
{
    // update the smoothing buffer
    cache.m_lastTranslationsBuffer.push_back(AZStd::make_pair(transform.GetTranslation(), deltaTime));
    cache.m_lastRotationsBuffer.push_back(AZStd::make_pair(transform.GetRotation(), deltaTime));

    if (cache.m_lastTranslationsBuffer.size() > smoothBufferLen)
    {
        cache.m_lastTranslationsBuffer.pop_front();
    }
    if (cache.m_lastRotationsBuffer.size() > smoothBufferLen)
    {
        cache.m_lastRotationsBuffer.pop_front();
    }
}
