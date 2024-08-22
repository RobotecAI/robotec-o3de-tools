#pragma once

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>
#include <Pointcloud/PointcloudAsset.h>
#include <Pointcloud/PointcloudTypeIds.h>

namespace Pointcloud
{
    class PointcloudConfigurationRequests : public AZ::ComponentBus
    {
    public:
        AZ_RTTI(PointcloudConfigurationRequests, PointCloudRequestsTypeId);
        virtual ~PointcloudConfigurationRequests() = default;
        virtual void SetPointcloudAsset(AZ::Data::Asset<PointcloudAsset> asset) = 0;
        virtual void SetPointSize(float pointSize) = 0;
        virtual void SetVisibility(bool visible) = 0;
    };

    using PointcloudConfigurationBus = AZ::EBus<PointcloudConfigurationRequests>;
} // namespace Pointcloud
