
#include "CsvSpawnerEditorSystemComponent.h"
#include <CsvSpawner/CsvSpawnerEditorComponent.h>
#include <CsvSpawner/CsvSpawnerTypeIds.h>
#include <CsvSpawnerModuleInterface.h>

namespace CsvSpawner
{
    class CsvSpawnerEditorModule : public CsvSpawnerModuleInterface
    {
    public:
        AZ_RTTI(CsvSpawnerEditorModule, CsvSpawnerEditorModuleTypeId, CsvSpawnerModuleInterface);
        AZ_CLASS_ALLOCATOR(CsvSpawnerEditorModule, AZ::SystemAllocator);

        CsvSpawnerEditorModule()
        {
            // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
            // Add ALL components descriptors associated with this gem to m_descriptors.
            // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and
            // EditContext. This happens through the [MyComponent]::Reflect() function.
            m_descriptors.insert(
                m_descriptors.end(),
                {
                    CsvSpawnerEditorSystemComponent::CreateDescriptor(),
                    CsvSpawnerEditorComponent::CreateDescriptor(),
                });
        }

        /**
         * Add required SystemComponents to the SystemEntity.
         * Non-SystemComponents should not be added here
         */
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList{
                azrtti_typeid<CsvSpawnerEditorSystemComponent>(),
            };
        }
    };
} // namespace CsvSpawner

AZ_DECLARE_MODULE_CLASS(Gem_CsvSpawner, CsvSpawner::CsvSpawnerEditorModule)
