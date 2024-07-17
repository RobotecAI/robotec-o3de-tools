/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Asset/AssetCommon.h>
#include <AzCore/Asset/AssetSerializer.h>
#include <AzFramework/Asset/GenericAssetHandler.h>

namespace Pointcloud
{
    class PointcloudAsset final : public AZ::Data::AssetData
    {
    public:
        struct CloudVertex
        {
            AZStd::array<float, 3> m_position;
            AZStd::array<float, 3> m_normal;
            uint32_t m_color;
        };
        static constexpr inline const char* DisplayName = "PointcloudAsset";
        static constexpr inline const char* Extension = "pointcloud";
        static constexpr inline const char* Group = "Pointcloud";

        AZ_RTTI(PointcloudAsset, "{0190c039-385b-7c8a-9172-31e83c091216}", AZ::Data::AssetData)
        AZ_CLASS_ALLOCATOR(PointcloudAsset, AZ::SystemAllocator);

        AZStd::vector<CloudVertex> m_data;
    };

    class PointcloudAssetHandler final : public AzFramework::GenericAssetHandler<PointcloudAsset>
    {
    public:
        PointcloudAssetHandler();

    private:
        // AZ::Data::AssetHandler overrides...
        AZ::Data::AssetHandler::LoadResult LoadAssetData(
            const AZ::Data::Asset<AZ::Data::AssetData>& asset,
            AZStd::shared_ptr<AZ::Data::AssetDataStream> stream,
            const AZ::Data::AssetFilterCB& assetLoadFilterCB) override;
        bool SaveAssetData(const AZ::Data::Asset<AZ::Data::AssetData>& asset, AZ::IO::GenericStream* stream) override;
    };
} // namespace Pointcloud
