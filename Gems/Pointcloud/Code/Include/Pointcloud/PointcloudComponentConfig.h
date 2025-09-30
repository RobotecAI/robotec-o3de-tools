#pragma once

#include "PointcloudAsset.h"
#include "PointcloudFeatureProcessorInterface.h"

namespace Pointcloud
{
    class PointcloudComponentConfig final : public AZ::ComponentConfig
    {
    public:
        AZ_RTTI(PointcloudComponentConfig, "{3ae848a0-3cd0-439e-bafe-db0270caae47}");

        PointcloudComponentConfig() = default;
        ~PointcloudComponentConfig() = default;

        static void Reflect(AZ::ReflectContext* context);

        AZ::EntityId m_editorEntityId;
        float m_pointSize = 1.0f;
        PointcloudFeatureProcessorInterface::PointcloudHandle m_pointcloudHandle =
            PointcloudFeatureProcessorInterface::InvalidPointcloudHandle;
        AZ::Data::Asset<PointcloudAsset> m_pointcloudAsset = {};
    };
} // namespace Pointcloud
