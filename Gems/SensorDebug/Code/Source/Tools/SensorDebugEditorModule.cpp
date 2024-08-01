
#include "SensorDebugEditorSystemComponent.h"
#include <SensorDebug/SensorDebugTypeIds.h>
#include <SensorDebugModuleInterface.h>

namespace SensorDebug
{
    class SensorDebugEditorModule : public SensorDebugModuleInterface
    {
    public:
        AZ_RTTI(SensorDebugEditorModule, SensorDebugEditorModuleTypeId, SensorDebugModuleInterface);
        AZ_CLASS_ALLOCATOR(SensorDebugEditorModule, AZ::SystemAllocator);

        SensorDebugEditorModule()
        {
            // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
            // Add ALL components descriptors associated with this gem to m_descriptors.
            // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and
            // EditContext. This happens through the [MyComponent]::Reflect() function.
            m_descriptors.insert(
                m_descriptors.end(),
                {
                    SensorDebugEditorSystemComponent::CreateDescriptor(),
                });
        }

        /**
         * Add required SystemComponents to the SystemEntity.
         * Non-SystemComponents should not be added here
         */
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList{
                azrtti_typeid<SensorDebugEditorSystemComponent>(),
            };
        }
    };
} // namespace SensorDebug

AZ_DECLARE_MODULE_CLASS(Gem_SensorDebug, SensorDebug::SensorDebugEditorModule)
