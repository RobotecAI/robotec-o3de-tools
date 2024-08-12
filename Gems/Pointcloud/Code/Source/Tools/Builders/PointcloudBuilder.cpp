/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root
 * of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "PointcloudBuilder.h"
#include <AssetBuilderSDK/AssetBuilderSDK.h>
#include <AssetBuilderSDK/SerializationDependencies.h>

#include "3rdParty/happly.h"
#include <Atom/RPI.Reflect/Buffer/BufferAsset.h>
#include <Atom/RPI.Reflect/Buffer/BufferAssetCreator.h>
#include <AzCore/Math/Color.h>
#include <Pointcloud/PointcloudAsset.h>
#include <Pointcloud/PointcloudFeatureProcessorInterface.h>
namespace Pointcloud
{

    constexpr const char* PointcloudBuilderJobKey = "PointcloudBuilder";

    PointcloudBuilder::PointcloudBuilder()
    {
        AssetBuilderSDK::AssetBuilderDesc pointcloudAssetBuilderDescriptor;
        pointcloudAssetBuilderDescriptor.m_name = PointcloudBuilderJobKey;
        pointcloudAssetBuilderDescriptor.m_version = 1;
        pointcloudAssetBuilderDescriptor.m_busId = azrtti_typeid<PointcloudBuilder>();
        pointcloudAssetBuilderDescriptor.m_analysisFingerprint = AZStd::string::format(
            "%u,%lu,%lu",
            PointcloudAsset::PointcloudMagicNumber,
            sizeof(PointcloudAsset::CloudHeader),
            sizeof(PointcloudAsset::CloudVertex));
        pointcloudAssetBuilderDescriptor.m_patterns.push_back(
            AssetBuilderSDK::AssetBuilderPattern("*.ply", AssetBuilderSDK::AssetBuilderPattern::PatternType::Wildcard));

        pointcloudAssetBuilderDescriptor.m_createJobFunction = [this](auto&& request, auto&& response)
        {
            return CreateJobs(request, response);
        };

        pointcloudAssetBuilderDescriptor.m_processJobFunction = [this](auto&& request, auto&& response)
        {
            return ProcessJob(request, response);
        };

        BusConnect(pointcloudAssetBuilderDescriptor.m_busId);

        // Register this builder with the AssetBuilderSDK.
        AssetBuilderSDK::AssetBuilderBus::Broadcast(
            &AssetBuilderSDK::AssetBuilderBus::Handler::RegisterBuilderInformation, pointcloudAssetBuilderDescriptor);
    }

    PointcloudBuilder::~PointcloudBuilder()
    {
        BusDisconnect();
    }

    void PointcloudBuilder::CreateJobs(
        const AssetBuilderSDK::CreateJobsRequest& request, AssetBuilderSDK::CreateJobsResponse& response) const
    {
        const auto fullSourcePath = AZ::IO::Path(request.m_watchFolder) / AZ::IO::Path(request.m_sourceFile);

        // Create an output job for each platform
        for (const AssetBuilderSDK::PlatformInfo& platformInfo : request.m_enabledPlatforms)
        {
            AssetBuilderSDK::JobDescriptor jobDescriptor;
            jobDescriptor.m_critical = false;
            jobDescriptor.m_jobKey = "Pointcloud Asset";
            jobDescriptor.SetPlatformIdentifier(platformInfo.m_identifier.c_str());

            response.m_createJobOutputs.push_back(jobDescriptor);
        }

        response.m_result = AssetBuilderSDK::CreateJobsResultCode::Success;
    }

    void PointcloudBuilder::ProcessJob(
        const AssetBuilderSDK::ProcessJobRequest& request, AssetBuilderSDK::ProcessJobResponse& response) const
    {
        response.m_resultCode = AssetBuilderSDK::ProcessJobResult_Failed;
        happly::PLYData plyIn(request.m_fullPath.c_str());
        const auto elementNames = plyIn.getElementNames();
        auto vertices = plyIn.getVertexPositions();
        // find is red green blue are present in the ply file
        bool hasRed = AZStd::find(elementNames.begin(), elementNames.end(), "red") != elementNames.end();
        bool hasGreen = AZStd::find(elementNames.begin(), elementNames.end(), "green") != elementNames.end();
        bool hasBlue = AZStd::find(elementNames.begin(), elementNames.end(), "blue") != elementNames.end();
        std::vector<std::array<unsigned char, 3>> colors;
        if (hasRed && hasGreen && hasBlue)
        {
            colors = plyIn.getVertexColors();
        }
        else
        {
            AZ_TracePrintf(AssetBuilderSDK::WarningWindow, "Warning: The ply file does not contain red, green, and blue color data.\n");
            colors.resize(vertices.size(), { 255, 255, 255 });
        }

        AZ::Data::Asset<PointcloudAsset> pointcloudAsset;
        pointcloudAsset.Create(AZ::Data::AssetId(AZ::Uuid::CreateRandom()));
        pointcloudAsset->m_data.reserve(vertices.size());
        for (int i = 0; i < vertices.size(); i++)
        {
            const auto& vertex = vertices[i];
            AZ::Color color = AZ::Colors::GreenYellow;
            if (i < colors.size())
            {
                const auto& colorData = colors[i];
                color = AZ::Color(
                    static_cast<float>(colorData[0]) / 255.0f,
                    static_cast<float>(colorData[1]) / 255.0f,
                    static_cast<float>(colorData[2]) / 255.0f,
                    1.0f);
            }
            PointcloudAsset::CloudVertex cloudVertex;
            cloudVertex.m_position = { static_cast<float>(vertex[0]), static_cast<float>(vertex[1]), static_cast<float>(vertex[2]) };

            cloudVertex.m_color = color.ToU32();
            pointcloudAsset->m_data.push_back(cloudVertex);
        }

        if (auto assetHandler = AZ::Data::AssetManager::Instance().GetHandler(azrtti_typeid<PointcloudAsset>()))
        {
            auto tempAssetOutputPath = AZ::IO::Path(request.m_tempDirPath) / request.m_sourceFile;
            tempAssetOutputPath.ReplaceExtension(PointcloudAsset::Extension);
            AZ::IO::FileIOStream outStream(
                tempAssetOutputPath.String().c_str(), AZ::IO::OpenMode::ModeWrite | AZ::IO::OpenMode::ModeCreatePath);
            if (!outStream.IsOpen())
            {
                AZ_TracePrintf(
                    AssetBuilderSDK::ErrorWindow, "Error: Failed job %s because file cannot be created.\n", request.m_fullPath.c_str());
                response.m_resultCode = AssetBuilderSDK::ProcessJobResult_Failed;
                return;
            }
            if (assetHandler->SaveAssetData(pointcloudAsset, &outStream))
            {
                // inform the AssetProcessor that we've created a new asset
                AssetBuilderSDK::JobProduct pointcloudJobProduct;
                pointcloudJobProduct.m_productFileName = tempAssetOutputPath.String();
                pointcloudJobProduct.m_productSubID = 0;
                pointcloudJobProduct.m_productAssetType = azrtti_typeid<PointcloudAsset>();
                pointcloudJobProduct.m_dependenciesHandled = true;
                response.m_outputProducts.push_back(AZStd::move(pointcloudJobProduct));
                response.m_resultCode = AssetBuilderSDK::ProcessJobResult_Success;
                return;
            }
            else
            {
                AZ_Error("PrefabInfoAssetBuilder", false, "The asset could not be saved to file: %s", tempAssetOutputPath.c_str());
            }
        }
        response.m_resultCode = AssetBuilderSDK::ProcessJobResult_Failed;
    }

} // namespace Pointcloud
