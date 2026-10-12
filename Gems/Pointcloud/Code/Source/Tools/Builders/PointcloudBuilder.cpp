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
#include <AzCore/Settings/SettingsRegistry.h>
#include <Pointcloud/PointcloudAsset.h>
#include <Pointcloud/PointcloudFeatureProcessorInterface.h>
#include <fstream>
#include <unordered_set>

namespace Pointcloud
{
    namespace
    {
        //! True if the PLY has correct header syntax and all required properties.
        //! Position is always required
        //! If colorsRequired is set to true also "red", "green" and "blue" must be defined
        bool ShouldProcessPointcloud(const char* fullPath, bool colorsRequired)
        {
            std::ifstream file(fullPath, std::ios::binary);
            if (!file.is_open())
            {
                return false;
            }

            // Custom parsing to avoid reading all vertex data which happly always attempts.
            std::unordered_set<std::string> properties;
            std::string line;
            bool inVertexElement = false;
            while (std::getline(file, line))
            {
                if (!line.empty() && line.back() == '\r')
                {
                    line.pop_back();
                }

                if (line.rfind("element ", 0) == 0)
                {
                    inVertexElement = line.rfind("element vertex ", 0) == 0;
                }
                else if (inVertexElement && line.rfind("property ", 0) == 0)
                {
                    const size_t lastSpace = line.find_last_of(' ');
                    if (lastSpace != std::string::npos)
                    {
                        properties.insert(line.substr(lastSpace + 1));
                    }
                }
                else if (line == "end_header")
                {
                    break;
                }
            }

            AZStd::vector<const char*> requiredProperties = { "x", "y", "z" };
            if (colorsRequired)
            {
                requiredProperties.push_back("red");
                requiredProperties.push_back("green");
                requiredProperties.push_back("blue");
            }
            for (const char* required : requiredProperties)
            {
                if (properties.find(required) == properties.end())
                {
                    return false;
                }
            }
            return true;
        }
    } // namespace

    constexpr const char* PointcloudBuilderJobKey = "PointcloudBuilder";

    //! When true, only .ply files whose vertex element has x/y/z/red/green/blue properties are processed. When false
    //! (default), every .ply is processed and points without color data are white.
    constexpr AZStd::string_view PointcloudBuilderSkipPlyWithoutColorKey = "/O3DE/Pointcloud/Builder/SkipPlyWithoutColor";

    PointcloudBuilder::PointcloudBuilder()
    {
        if (auto* registry = AZ::SettingsRegistry::Get())
        {
            registry->Get(m_skipPlyWithoutColor, PointcloudBuilderSkipPlyWithoutColorKey);
        }

        AssetBuilderSDK::AssetBuilderDesc pointcloudAssetBuilderDescriptor;
        pointcloudAssetBuilderDescriptor.m_name = PointcloudBuilderJobKey;
        pointcloudAssetBuilderDescriptor.m_version = 2;
        pointcloudAssetBuilderDescriptor.m_busId = azrtti_typeid<PointcloudBuilder>();
        // Includes the filter setting so toggling it re-runs CreateJobs on every .ply
        pointcloudAssetBuilderDescriptor.m_analysisFingerprint = AZStd::string::format(
            "%u,%lu,%lu,%d",
            PointcloudAsset::PointcloudMagicNumber,
            sizeof(PointcloudAsset::CloudHeader),
            sizeof(PointcloudAsset::CloudVertex),
            m_skipPlyWithoutColor ? 1 : 0);
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

        if (!ShouldProcessPointcloud(fullSourcePath.String().c_str(), m_skipPlyWithoutColor))
        {
            // Not a point-cloud .ply (missing x/y/z/red/green/blue vertex properties)
            response.m_result = AssetBuilderSDK::CreateJobsResultCode::Success;
            return;
        }

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
        if (!ShouldProcessPointcloud(request.m_fullPath.c_str(), m_skipPlyWithoutColor))
        {
            AZ_TracePrintf(
                AssetBuilderSDK::ErrorWindow,
                "Error: %s is not a point-cloud .ply (missing at least one required property x/y/z/red/green/blue).\n",
                request.m_fullPath.c_str());
            return;
        }

        happly::PLYData plyIn(request.m_fullPath.c_str());
        const auto vertices = plyIn.getVertexPositions();
        auto& vertexElement = plyIn.getElement("vertex");
        std::vector<std::array<unsigned char, 3>> colors;
        if (vertexElement.hasProperty("red") && vertexElement.hasProperty("green") && vertexElement.hasProperty("blue"))
        {
            colors = plyIn.getVertexColors();
        }
        else
        {
            AZ_TracePrintf(
                AssetBuilderSDK::WarningWindow,
                "Warning: %s does not contain red, green and blue color data, points will be white.\n",
                request.m_fullPath.c_str());
        }

        AZ::Data::Asset<PointcloudAsset> pointcloudAsset;
        pointcloudAsset.Create(AZ::Data::AssetId(AZ::Uuid::CreateRandom()));
        pointcloudAsset->m_data.reserve(vertices.size());
        for (int i = 0; i < vertices.size(); i++)
        {
            const auto& vertex = vertices[i];
            AZ::Color color = AZ::Colors::White;
            if (i < colors.size())
            {
                const auto& colorData = colors[i];
                color = AZ::Color(
                    static_cast<float>(colorData[0]) / 255.0f,
                    static_cast<float>(colorData[1]) / 255.0f,
                    static_cast<float>(colorData[2]) / 255.0f,
                    1.0f);
            }
            PointcloudAsset::CloudVertex cloudVertex{};
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
