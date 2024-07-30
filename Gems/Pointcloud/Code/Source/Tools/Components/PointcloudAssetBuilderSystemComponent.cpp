#include "PointcloudAssetBuilderSystemComponent.h"
#include "../Builders/PointcloudBuilder.h"
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/smart_ptr/unique_ptr.h>

namespace Pointcloud
{

    PointcloudAssetBuilderSystemComponent::PointcloudAssetBuilderSystemComponent() = default;
    PointcloudAssetBuilderSystemComponent::~PointcloudAssetBuilderSystemComponent() = default;

    void PointcloudAssetBuilderSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<PointcloudAssetBuilderSystemComponent, AZ::Component>()->Version(0)->Attribute(
                AZ::Edit::Attributes::SystemComponentTags, AssetBuilderSDK::ComponentTags::AssetBuilder);
        }
    }

    void PointcloudAssetBuilderSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("PointcloudAssetBuilderService"));
    }

    void PointcloudAssetBuilderSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("PointcloudAssetBuilderService"));
    }

    void PointcloudAssetBuilderSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        // This doesn't require any services to exist before startup.
    }

    void PointcloudAssetBuilderSystemComponent::GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        // If the asset services exist at all, they should be started first so that
        // the Sdf builder can register with them correctly.
        dependent.push_back(AZ_CRC_CE("AssetDatabaseService"));
        dependent.push_back(AZ_CRC_CE("AssetCatalogService"));
    }

    void PointcloudAssetBuilderSystemComponent::Activate()
    {
        m_pointcloudAssetHandler = aznew PointcloudAssetHandler();
        m_pointcloudAssetHandler->Register();
        m_pointcloudBuilder = AZStd::make_unique<PointcloudBuilder>();
    }

    void PointcloudAssetBuilderSystemComponent::Deactivate()
    {
        m_pointcloudAssetHandler->Unregister();
        delete m_pointcloudAssetHandler;
        m_pointcloudBuilder.reset();
    }

} // namespace Pointcloud