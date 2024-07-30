#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/smart_ptr/unique_ptr.h>
#include <Pointcloud/PointcloudAsset.h>
namespace AZ
{
    class ReflectContext;
}
namespace AZ::Data
{
    class AssetHandler;
}

namespace Pointcloud
{
    class PointcloudBuilder;

    class PointcloudAssetBuilderSystemComponent : public AZ::Component
    {
    public:
        AZ_COMPONENT(PointcloudAssetBuilderSystemComponent, "{0190bb4b-5e26-7898-b3c9-e9f3a50f1177}");
        static void Reflect(AZ::ReflectContext* context);

        PointcloudAssetBuilderSystemComponent();
        ~PointcloudAssetBuilderSystemComponent();

    private:
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        // Component overrides ...
        void Activate() override;
        void Deactivate() override;

        // Asset builder instance
        AZStd::unique_ptr<PointcloudBuilder> m_pointcloudBuilder;

    private:
        PointcloudAssetHandler* m_pointcloudAssetHandler;
    };
} // namespace Pointcloud
