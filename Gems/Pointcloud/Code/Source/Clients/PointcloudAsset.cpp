/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <Pointcloud/PointcloudAsset.h>

namespace Pointcloud
{
    PointcloudAssetHandler::PointcloudAssetHandler()
        : AzFramework::GenericAssetHandler<PointcloudAsset>(
              PointcloudAsset::DisplayName, PointcloudAsset::Group, PointcloudAsset::Extension)
    {
    }

    bool PointcloudAssetHandler::SaveAssetData(const AZ::Data::Asset<AZ::Data::AssetData>& asset, AZ::IO::GenericStream* stream)
    {
        PointcloudAsset* assetData = asset.GetAs<PointcloudAsset>();
        AZ_Assert(assetData, "Asset is not of the expected type.");
        const size_t headerSize = sizeof(PointcloudAsset::CloudHeader);
        const size_t elementSize = sizeof(PointcloudAsset::CloudVertex);
        const size_t numElements = assetData->m_data.size();
        const size_t expectedDataSize = elementSize * numElements;
        const size_t expectedSize = headerSize + expectedDataSize;
        AZStd::vector<uint8_t> headerData(headerSize);
        AZStd::vector<uint8_t> pointData(expectedDataSize);

        PointcloudAsset::CloudHeader header;
        header.m_magicNumber = PointcloudAsset::PointcloudMagicNumber;
        header.m_elementSize = elementSize;
        header.m_numPoints = static_cast<uint32_t>(numElements);

        std::memcpy(headerData.data(), (void*)&header, headerSize);
        auto bytesWritten = stream->Write(headerSize, headerData.data());
        AZ_Assert(bytesWritten == headerSize, "Failed to write header data to stream.");

        std::memcpy(pointData.data(), assetData->m_data.data(), expectedDataSize);
        bytesWritten += stream->Write(expectedDataSize, pointData.data());

        AZ_Assert(bytesWritten == expectedSize, "Failed to write point data to stream.");
        if (bytesWritten == expectedSize)
        {
            return true;
        }
        return false;
    }

    AZ::Data::AssetHandler::LoadResult PointcloudAssetHandler::LoadAssetData(
        const AZ::Data::Asset<AZ::Data::AssetData>& asset,
        AZStd::shared_ptr<AZ::Data::AssetDataStream> stream,
        [[maybe_unused]] const AZ::Data::AssetFilterCB& assetLoadFilterCB)
    {
        PointcloudAsset* assetData = asset.GetAs<PointcloudAsset>();
        AZ_Assert(assetData, "Asset is not of the expected type.");

        if (assetData && stream->GetLength() > 0)
        {
            PointcloudAsset::CloudHeader header;
            stream->Read(sizeof(PointcloudAsset::CloudHeader), (void*)&header);
            AZ_Assert(header.m_magicNumber == PointcloudAsset::PointcloudMagicNumber, "Invalid magic number in pointcloud asset file.");
            AZ_Assert(header.m_elementSize == sizeof(PointcloudAsset::CloudVertex), "Invalid element size in pointcloud asset file.");
            AZ_Printf("Pointcloud", "Loading pointcloud with %d points", header.m_numPoints);
            const auto expectedDataSize = header.m_elementSize * header.m_numPoints;

            AZStd::vector<uint8_t> rawData(expectedDataSize);
            stream->Read(stream->GetLength(), rawData.data());

            const size_t elementSize = sizeof(PointcloudAsset::CloudVertex);
            const size_t numElements = rawData.size() / elementSize;
            assetData->m_data.resize(numElements);
            std::memcpy(assetData->m_data.data(), rawData.data(), rawData.size());
            return AZ::Data::AssetHandler::LoadResult::LoadComplete;
        }

        return AZ::Data::AssetHandler::LoadResult::Error;
    }
} // namespace Pointcloud
