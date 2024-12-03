
#include "GeoJSONSpawner/GeoJSONSpawnerEditorComponent.h"
#include "GeoJSONSpawner/ROS2Interface/GeoJSONSpawnerROS2InterfaceEditorComponent.h"
#include "GeoJSONSpawnerEditorSystemComponent.h"
#include <GeoJSONSpawner/GeoJSONSpawnerTypeIds.h>
#include <GeoJSONSpawnerModuleInterface.h>

namespace GeoJSONSpawner
{
    class GeoJSONSpawnerEditorModule : public GeoJSONSpawnerModuleInterface
    {
    public:
        AZ_RTTI(GeoJSONSpawnerEditorModule, GeoJSONSpawnerEditorModuleTypeId, GeoJSONSpawnerModuleInterface);
        AZ_CLASS_ALLOCATOR(GeoJSONSpawnerEditorModule, AZ::SystemAllocator);

        GeoJSONSpawnerEditorModule()
        {
            // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
            // Add ALL components descriptors associated with this gem to m_descriptors.
            // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and
            // EditContext. This happens through the [MyComponent]::Reflect() function.
            m_descriptors.insert(
                m_descriptors.end(),
                { GeoJSONSpawnerEditorSystemComponent::CreateDescriptor(),
                  GeoJSONSpawnerEditorComponent::CreateDescriptor(),
                  ROS2Interface::GeoJSONSpawnerROS2InterfaceEditorComponent::CreateDescriptor() });
        }

        /**
         * Add required SystemComponents to the SystemEntity.
         * Non-SystemComponents should not be added here
         */
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList{
                azrtti_typeid<GeoJSONSpawnerEditorSystemComponent>(),
            };
        }
    };
} // namespace GeoJSONSpawner

AZ_DECLARE_MODULE_CLASS(Gem_GeoJSONSpawner, GeoJSONSpawner::GeoJSONSpawnerEditorModule)
