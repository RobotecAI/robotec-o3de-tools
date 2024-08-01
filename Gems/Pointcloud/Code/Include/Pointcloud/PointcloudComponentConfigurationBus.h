#pragma once

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>
#include <Pointcloud/PointcloudAsset.h>
#include <Pointcloud/PointcloudTypeIds.h>

namespace Pointcloud
{
    class PointCloudEditorComponentRequests : public AZ::ComponentBus
    {
    public:
        AZ_RTTI(PointCloudEditorComponentRequests, PointCloudEditorComponentRequestsTypeId);
        virtual ~PointCloudEditorComponentRequests() = default;
        virtual void SetPointcloudAsset(AZ::Data::Asset<PointcloudAsset> asset) = 0;
        virtual void SetPointSize(float pointSize) = 0;
    };

    using PointcloudEditorComponentConfigurationBus = AZ::EBus<PointCloudEditorComponentRequests>;
} // namespace Pointcloud