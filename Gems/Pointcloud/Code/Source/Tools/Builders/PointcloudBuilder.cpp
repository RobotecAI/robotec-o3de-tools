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

#include "3rd/happly.h"
#include <Atom/RPI.Reflect/Buffer/BufferAsset.h>
#include <Atom/RPI.Reflect/Buffer/BufferAssetCreator.h>
#include <Pointcloud/PointcloudFeatureProcessorInterface.h>
#include <Pointcloud//PointcloudAsset.h>
namespace Pointcloud
{

    constexpr const char* PointcloudBuilderJobKey = "PointcloudBuilder";

    PointcloudBuilder::PointcloudBuilder()
    {
        AssetBuilderSDK::AssetBuilderDesc prefabInfoAssetBuilderDescriptor;
        prefabInfoAssetBuilderDescriptor.m_name = PointcloudBuilderJobKey;
        prefabInfoAssetBuilderDescriptor.m_version = 1;
        prefabInfoAssetBuilderDescriptor.m_busId = azrtti_typeid<PointcloudBuilder>();
        prefabInfoAssetBuilderDescriptor.m_analysisFingerprint =
            AZStd::string::format("ElementLength_%lu", sizeof(PointcloudAsset::CloudVertex));
        prefabInfoAssetBuilderDescriptor.m_patterns.push_back(
            AssetBuilderSDK::AssetBuilderPattern("*.ply", AssetBuilderSDK::AssetBuilderPattern::PatternType::Wildcard));

        prefabInfoAssetBuilderDescriptor.m_createJobFunction = [this](auto&& request, auto&& response)
        {
            return CreateJobs(request, response);
        };

        prefabInfoAssetBuilderDescriptor.m_processJobFunction = [this](auto&& request, auto&& response)
        {
            return ProcessJob(request, response);
        };

        BusConnect(prefabInfoAssetBuilderDescriptor.m_busId);

        // Register this builder with the AssetBuilderSDK.
        AssetBuilderSDK::AssetBuilderBus::Broadcast(
            &AssetBuilderSDK::AssetBuilderBus::Handler::RegisterBuilderInformation, prefabInfoAssetBuilderDescriptor);
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
        auto vertices = plyIn.getVertexPositions();
        const auto elementNames = plyIn.getElementNames();

        AZ_Printf("PointcloudBuilder", "Opened pointcloud, elements in PLY: ");
        for (const auto& elementName : elementNames)
        {
            AZ_Printf("PointcloudBuilder", "  -> Element: %s", elementName.c_str());
        }
        AZ_Printf("PointcloudBuilder", " Number of vertices: %d", vertices.size());

        AZ::Data::Asset<PointcloudAsset> pointcloudAsset;
        pointcloudAsset.Create(AZ::Data::AssetId(AZ::Uuid::CreateRandom()));
        pointcloudAsset->m_data.reserve(vertices.size());
        for (const auto& vertex : vertices)
        {
            PointcloudAsset::CloudVertex cloudVertex;
            cloudVertex.m_position = { static_cast<float>(vertex[0]), static_cast<float>(vertex[1]), static_cast<float>(vertex[2]) };
            cloudVertex.m_normal = { 0.0f, 0.0f, 0.0f };
            cloudVertex.m_color = 0xFFFFFFFF;
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
                    AssetBuilderSDK::ErrorWindow, "Error: Failed job %s because file cannot be created.\n",
                    request.m_fullPath.c_str());
                response.m_resultCode = AssetBuilderSDK::ProcessJobResult_Failed;
                return;
            }
            if (assetHandler->SaveAssetData(pointcloudAsset, &outStream))
            {
                AZ_Info("PrefabInfoAssetBuilder", "PrefabInfoAsset created successfully: %s", tempAssetOutputPath.c_str());
                // inform the AssetProcessor that we've created a new asset
                AssetBuilderSDK::JobProduct prefabInfoJobProduct;
                prefabInfoJobProduct.m_productFileName = tempAssetOutputPath.String();
                prefabInfoJobProduct.m_productSubID = 0;
                prefabInfoJobProduct.m_productAssetType = azrtti_typeid<PointcloudAsset>();
                prefabInfoJobProduct.m_dependenciesHandled = true;
                response.m_outputProducts.push_back(AZStd::move(prefabInfoJobProduct));
                response.m_resultCode = AssetBuilderSDK::ProcessJobResult_Success;
                return;
            }
            else
            {
                AZ_Error("PrefabInfoAssetBuilder", false, "The asset could not be saved to file: %s",
                tempAssetOutputPath.c_str());
            }
        }
        response.m_resultCode = AssetBuilderSDK::ProcessJobResult_Failed;
    }

} // namespace Pointcloud
