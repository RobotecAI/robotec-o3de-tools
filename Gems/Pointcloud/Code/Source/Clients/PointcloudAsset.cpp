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
       : AzFramework::GenericAssetHandler<PointcloudAsset>(PointcloudAsset::DisplayName, PointcloudAsset::Group, PointcloudAsset::Extension)
   {
   }

   bool PointcloudAssetHandler::SaveAssetData(const AZ::Data::Asset<AZ::Data::AssetData>& asset, AZ::IO::GenericStream* stream)
   {
        PointcloudAsset* assetData = asset.GetAs<PointcloudAsset>();
        AZ_Assert(assetData, "Asset is not of the expected type.");
        const size_t elementSize = sizeof(PointcloudAsset::CloudVertex);
        const size_t numElements = assetData->m_data.size();
        const size_t expectedSize = elementSize * numElements;
        AZStd::vector<uint8_t> rawData(expectedSize);
        std::memcpy(rawData.data(), assetData->m_data.data(), expectedSize);

        const auto bytesWritten = stream->Write(expectedSize, rawData.data());
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

       if(assetData && stream->GetLength() > 0)
       {
           AZStd::vector<uint8_t> rawData(stream->GetLength());
           assetData->m_data.resize(stream->GetLength());
           stream->Read(stream->GetLength(), rawData.data());

           const size_t elementSize = sizeof(PointcloudAsset::CloudVertex);
           const size_t numElements = rawData.size() / elementSize;
           assetData->m_data.resize(numElements);
           std::memcpy(assetData->m_data.data(), rawData.data(), rawData.size());
           return AZ::Data::AssetHandler::LoadResult::LoadComplete;
       }

       return AZ::Data::AssetHandler::LoadResult::Error;
   }
} // AZ::Render
