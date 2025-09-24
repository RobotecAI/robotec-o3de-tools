#include "LidarConfiguration.h"
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Serialization/EditContext.h>

namespace SimpleLidarSensor
{
    void LidarConfiguration::Reflect(AZ::ReflectContext* context)
    {
        if (auto serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<LidarConfiguration>()
                ->Version(3)
                ->Field("NumberOfLayers", &LidarConfiguration::m_numberOfLayers)
                ->Field("MinElevationDeg", &LidarConfiguration::m_minElevationDeg)
                ->Field("MaxElevationDeg", &LidarConfiguration::m_maxElevationDeg)
                ->Field("AzimuthSamples", &LidarConfiguration::m_azimuthSamples)
                ->Field("MinRange", &LidarConfiguration::m_minRange)
                ->Field("MaxRange", &LidarConfiguration::m_maxRange)
                ->Field("PublishDebugImages", &LidarConfiguration::m_publishDebugImages)
                ->Field("PublishRGB", &LidarConfiguration::m_publishRGB)
                ->Field("IntensityFromRGB", &LidarConfiguration::m_intensityFromRGB)
                ->Field("PublishRing", &LidarConfiguration::m_publishRing)
                ->Field("IntensityFieldName", &LidarConfiguration::m_intensity)
                ->Field("RingFieldName", &LidarConfiguration::m_ringFieldName)
                ->Field("TimeFieldName", &LidarConfiguration::m_timeFieldName);
            if (auto ec = serialize->GetEditContext())
            {
                ec->Class<LidarConfiguration>("Lidar Configuration", "Configuration parameters for the LIDAR sensor")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Visibility, AZ::Edit::PropertyVisibility::ShowChildrenOnly)
                    ->DataElement(AZ::Edit::UIHandlers::SpinBox, &LidarConfiguration::m_numberOfLayers,
                                 "Number of Layers", "Number of vertical scanning layers")
                        ->Attribute(AZ::Edit::Attributes::Min, 1)
                        ->Attribute(AZ::Edit::Attributes::Max, 128)
                        ->Attribute(AZ::Edit::Attributes::Step, 1)
                    ->DataElement(AZ::Edit::UIHandlers::SpinBox, &LidarConfiguration::m_minElevationDeg,
                                 "Min Elevation (°)", "Minimum elevation angle in degrees")
                        ->Attribute(AZ::Edit::Attributes::Min, -90.0f)
                        ->Attribute(AZ::Edit::Attributes::Max, 90.0f)
                        ->Attribute(AZ::Edit::Attributes::Step, 0.1f)
                    ->DataElement(AZ::Edit::UIHandlers::SpinBox, &LidarConfiguration::m_maxElevationDeg,
                                 "Max Elevation (°)", "Maximum elevation angle in degrees")
                        ->Attribute(AZ::Edit::Attributes::Min, -90.0f)
                        ->Attribute(AZ::Edit::Attributes::Max, 90.0f)
                        ->Attribute(AZ::Edit::Attributes::Step, 0.1f)
                    ->DataElement(AZ::Edit::UIHandlers::SpinBox, &LidarConfiguration::m_azimuthSamples,
                                 "Azimuth Samples", "Number of horizontal samples (360° resolution)")
                        ->Attribute(AZ::Edit::Attributes::Min, 64)
                        ->Attribute(AZ::Edit::Attributes::Max, 32768)
                        ->Attribute(AZ::Edit::Attributes::Step, 64)
                    ->DataElement(AZ::Edit::UIHandlers::SpinBox, &LidarConfiguration::m_minRange,
                                 "Min Range (m)", "Minimum detection range in meters")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.01f)
                        ->Attribute(AZ::Edit::Attributes::Max, 1000.0f)
                        ->Attribute(AZ::Edit::Attributes::Step, 0.01f)
                    ->DataElement(AZ::Edit::UIHandlers::SpinBox, &LidarConfiguration::m_maxRange,
                                 "Max Range (m)", "Maximum detection range in meters")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.1f)
                        ->Attribute(AZ::Edit::Attributes::Max, 1000.0f)
                        ->Attribute(AZ::Edit::Attributes::Step, 0.1f)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &LidarConfiguration::m_publishDebugImages,
                                 "Publish Debug Images", "Whether to publish debug images showing the LIDAR points")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &LidarConfiguration::m_publishRGB,
                                 "Publish RGB", "Whether to include RGB color in the point cloud")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &LidarConfiguration::m_intensityFromRGB,
                                 "Intensity From RGB", "Whether to derive intensity from RGB values and add to point cloud")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &LidarConfiguration::m_publishRing,
                                 "Publish Ring", "Whether to include ring number in the point cloud")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &LidarConfiguration::m_intensity,
                                 "Intensity Field Name", "Name of the intensity field in the point cloud")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &LidarConfiguration::m_ringFieldName,
                                 "Ring Field Name", "Name of the ring field in the point cloud")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &LidarConfiguration::m_timeFieldName,
                                 "Time Field Name", "Name of the time field in the point cloud");
            }
        }
    }
} // namespace SimpleLidarSensor