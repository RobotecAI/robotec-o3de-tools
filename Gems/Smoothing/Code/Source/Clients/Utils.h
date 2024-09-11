
#pragma once

#include <AzFramework/Components/TransformComponent.h>

namespace SmoothingUtils
{
    //! Remove the tilt from the transform.
    //! The tilt is removed by projecting the transform basis of rotation matrix to the ground plane.
    //! @param transform The transform to remove the tilt.
    //! @return The transform without tilt.
    AZ::Transform RemoveTiltFromTransform(AZ::Transform transform);

    struct SmoothingCache
    {
        //! The smoothing buffer for translation, the first element is the translation, the second element is the weight.
        AZStd::deque<AZStd::pair<AZ::Vector3, float>> m_lastTranslationsBuffer;
        //! The smoothing buffer for rotation, the first element is the tangential vector, the second element is the weight.
        AZStd::deque<AZStd::pair<AZ::Quaternion, float>> m_lastRotationsBuffer;
    };
    //! Compute weighted average of the vectors in the buffer.
    //! @param buffer The buffer to compute the average.
    //! @return The average vector.
    AZ::Vector3 AverageVector(const AZStd::deque<AZStd::pair<AZ::Vector3, float>>& buffer);

    //! Cache the transform in smoothing buffer.
    //! @param cache The cache to store the transform.
    //! @param transform The transform to cache.
    //! @param deltaTime The time between the last frame and the current frame.
    //! @param smoothBufferLen The length of the buffer to smooth the entity's position.
    void CacheTransform(SmoothingUtils::SmoothingCache& cache, const AZ::Transform& transform, float deltaTime, int smoothBufferLen);

    //! Compute weighted average of translation in the buffer.
    //! @return The average translation.
    AZ::Vector3 SmoothTranslation(const SmoothingCache& cache);

    //! Compute weighted average of rotation in the buffer.
    //! @return The average rotation.
    AZ::Quaternion SmoothRotation(const SmoothingCache& cache);

} // namespace SmoothingUtils
