/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include "PointcloudTypeIds.h"
#include <AzCore/Asset/AssetCommon.h>
#include <AzCore/Asset/AssetSerializer.h>
#include <AzFramework/Asset/GenericAssetHandler.h>

namespace Pointcloud
{
    class PointcloudAsset final : public AZ::Data::AssetData
    {
    public:
        //! The vertex data for the pointcloud
        struct CloudVertex
        {
            AZ_TYPE_INFO(CloudVertex, CloudVertexAssetTypeId);
            AZStd::array<float, 3> m_position;
            uint32_t m_color;
            static void Reflect(AZ::ReflectContext* context);
        };

        struct CloudHeader
        {
            uint32_t m_magicNumber{ 0x0 };
            uint32_t m_elementSize{ 0x0 };
            uint32_t m_numPoints{ 0x0 };
        };

        static constexpr uint32_t PointcloudMagicNumber = 0x12345678;

        static constexpr inline const char* DisplayName = "PointcloudAsset";
        static constexpr inline const char* Extension = "pointcloud";
        static constexpr inline const char* Group = "Pointcloud";

        AZ_RTTI(PointcloudAsset, PointcloudAssetTypeId, AZ::Data::AssetData)
        AZ_CLASS_ALLOCATOR(PointcloudAsset, AZ::SystemAllocator);

        static void Reflect(AZ::ReflectContext* context);

        AZStd::vector<CloudVertex> m_data;
    };

    class PointcloudAssetHandler final : public AzFramework::GenericAssetHandler<PointcloudAsset>
    {
    public:
        PointcloudAssetHandler();

        // AZ::Data::AssetHandler overrides...
        AZ::Data::AssetHandler::LoadResult LoadAssetData(
            const AZ::Data::Asset<AZ::Data::AssetData>& asset,
            AZStd::shared_ptr<AZ::Data::AssetDataStream> stream,
            const AZ::Data::AssetFilterCB& assetLoadFilterCB) override;
        bool SaveAssetData(const AZ::Data::Asset<AZ::Data::AssetData>& asset, AZ::IO::GenericStream* stream) override;
    };
} // namespace Pointcloud
