
#pragma once

#include <AssetBuilderSDK/AssetBuilderBusses.h>
#include <AssetBuilderSDK/AssetBuilderSDK.h>
#include <AzCore/std/string/regex.h>

namespace Pointcloud
{
    [[maybe_unused]] constexpr const char* PointcloudBuilderName = "PointcloudBuilder";

    class PointcloudBuilder : public AssetBuilderSDK::AssetBuilderCommandBus::Handler
    {
    public:
        AZ_RTTI(PointcloudBuilder, "{0190bb45-d105-7336-8b3e-d039c647115f}");

        PointcloudBuilder();
        ~PointcloudBuilder();

        // AssetBuilderSDK::AssetBuilderCommandBus overrides...
        void CreateJobs(const AssetBuilderSDK::CreateJobsRequest& request, AssetBuilderSDK::CreateJobsResponse& response) const;
        void ProcessJob(const AssetBuilderSDK::ProcessJobRequest& request, AssetBuilderSDK::ProcessJobResponse& response) const;
        void ShutDown() override
        {
        }
    };

} // namespace Pointcloud
