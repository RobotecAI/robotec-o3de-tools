#pragma once

#include <AzCore/RTTI/TypeInfo.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Math/MathUtils.h>
#include <AzCore/std/containers/vector.h>

namespace SimpleLidarSensor
{

    struct LidarConfiguration
    {

        AZ_TYPE_INFO(LidarConfiguration, "{F7E3B4C1-2D5A-4B8E-9C6F-1A3D7E9F2C4B}");

        int m_numberOfLayers = 30;                    // Number of vertical layers
        float m_minElevationDeg = -15.0f;             // Minimum elevation angle in degrees
        float m_maxElevationDeg = 15.0f;              // Maximum elevation angle in degrees
        int m_azimuthSamples = 8192;                  // Number of azimuth samples (horizontal resolution)
        float m_minRange = 0.1f;                      // Minimum detection range
        float m_maxRange = 100.0f;                    // Maximum detection range
        bool m_publishDebugImages = false;            // Whether to publish debug images showing the LIDAR points
        bool m_publishRGB = true;                     // Whether to include RGB color in the point cloud
        bool m_intensityFromRGB = false;              // Whether to derive intensity from RGB values and add to point cloud
        bool m_publishRing = true;                    // Whether to include ring number in the point cloud
        AZStd::string m_intensity = "intensity";      // Name of the intensity field in the point cloud
        AZStd::string m_ringFieldName = "ring";       // Name of the ring
        AZStd::string m_timeFieldName = "time";       // Name of the time field in the point cloud



        static void Reflect(AZ::ReflectContext* context);

        // Helper methods
        [[nodiscard]] float GetElevationStepDeg() const
        {
            return m_numberOfLayers > 1 ? (m_maxElevationDeg - m_minElevationDeg) / static_cast<float>(m_numberOfLayers - 1) : 0.0f;
        }

        [[nodiscard]] float GetAzimuthStepRad() const
        {
            return static_cast<float>(2.0 * M_PI) / static_cast<float>(m_azimuthSamples);
        }

        [[nodiscard]] AZStd::vector<float> GetLayerElevations() const
        {
            AZStd::vector<float> elevations;
            elevations.reserve(static_cast<size_t>(m_numberOfLayers));

            if (m_numberOfLayers == 1)
            {
                elevations.push_back(AZ::DegToRad((m_minElevationDeg + m_maxElevationDeg) * 0.5f));
            }
            else
            {
                const float stepDeg = GetElevationStepDeg();
                for (int i = 0; i < m_numberOfLayers; ++i)
                {
                    const float elevationDeg = m_minElevationDeg + static_cast<float>(i) * stepDeg;
                    elevations.push_back(AZ::DegToRad(elevationDeg));
                }
            }
            return elevations;
        }
    };
} // namespace SimpleLidarSensor