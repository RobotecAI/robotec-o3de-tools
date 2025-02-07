
#include <AzCore/Serialization/SerializeContext.h>

#include <AzToolsFramework/API/ViewPaneOptions.h>

#include "TerrainShaperWidget.h"
#include "TerrainShaperEditorSystemComponent.h"

#include <TerrainShaper/TerrainShaperTypeIds.h>

namespace TerrainShaper
{
    AZ_COMPONENT_IMPL(TerrainShaperEditorSystemComponent, "TerrainShaperEditorSystemComponent",
        TerrainShaperEditorSystemComponentTypeId, BaseSystemComponent);

    void TerrainShaperEditorSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<TerrainShaperEditorSystemComponent, TerrainShaperSystemComponent>()
                ->Version(0);
        }
    }

    TerrainShaperEditorSystemComponent::TerrainShaperEditorSystemComponent() = default;

    TerrainShaperEditorSystemComponent::~TerrainShaperEditorSystemComponent() = default;

    void TerrainShaperEditorSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        BaseSystemComponent::GetProvidedServices(provided);
        provided.push_back(AZ_CRC_CE("TerrainShaperEditorService"));
    }

    void TerrainShaperEditorSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        BaseSystemComponent::GetIncompatibleServices(incompatible);
        incompatible.push_back(AZ_CRC_CE("TerrainShaperEditorService"));
    }

    void TerrainShaperEditorSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        BaseSystemComponent::GetRequiredServices(required);
    }

    void TerrainShaperEditorSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        BaseSystemComponent::GetDependentServices(dependent);
    }

    void TerrainShaperEditorSystemComponent::Activate()
    {
        TerrainShaperSystemComponent::Activate();
        AzToolsFramework::EditorEvents::Bus::Handler::BusConnect();
    }

    void TerrainShaperEditorSystemComponent::Deactivate()
    {
        AzToolsFramework::EditorEvents::Bus::Handler::BusDisconnect();
        TerrainShaperSystemComponent::Deactivate();
    }

    void TerrainShaperEditorSystemComponent::NotifyRegisterViews()
    {
        AzToolsFramework::ViewPaneOptions options;
        options.paneRect = QRect(100, 100, 500, 400);
        options.showOnToolsToolbar = true;
        options.toolbarIcon = ":/TerrainShaper/toolbar_icon.svg";

        // Register our custom widget as a dockable tool with the Editor under an Examples sub-menu
        AzToolsFramework::RegisterViewPane<TerrainShaperWidget>("TerrainShaper", "Examples", options);
    }

} // namespace TerrainShaper
