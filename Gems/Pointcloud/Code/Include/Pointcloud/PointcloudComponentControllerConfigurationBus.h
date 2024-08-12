#pragma once

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>
#include <Pointcloud/PointcloudAsset.h>
#include <Pointcloud/PointcloudTypeIds.h>

namespace Pointcloud
{
    class PointcloudControllerConfigurationRequests : public AZ::ComponentBus
    {
    public:
        AZ_RTTI(PointcloudControllerConfigurationRequests, PointCloudEditorComponentRequestsTypeId);
        virtual ~PointcloudControllerConfigurationRequests() = default;
        virtual void SetPointcloudAsset(AZ::Data::Asset<PointcloudAsset> asset) = 0;
        virtual void SetPointSize(float pointSize) = 0;
    };

    using PointcloudComponentControllerConfigurationBus = AZ::EBus<PointcloudControllerConfigurationRequests>;
} // namespace Pointcloud
