
#include <TerrainShaper/TerrainShaperTypeIds.h>
#include <TerrainShaperModuleInterface.h>
#include "TerrainShaperEditorSystemComponent.h"

void InitTerrainShaperResources()
{
    // We must register our Qt resources (.qrc file) since this is being loaded from a separate module (gem)
    Q_INIT_RESOURCE(TerrainShaper);
}

namespace TerrainShaper
{
    class TerrainShaperEditorModule
        : public TerrainShaperModuleInterface
    {
    public:
        AZ_RTTI(TerrainShaperEditorModule, TerrainShaperEditorModuleTypeId, TerrainShaperModuleInterface);
        AZ_CLASS_ALLOCATOR(TerrainShaperEditorModule, AZ::SystemAllocator);

        TerrainShaperEditorModule()
        {
            InitTerrainShaperResources();

            // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
            // Add ALL components descriptors associated with this gem to m_descriptors.
            // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and EditContext.
            // This happens through the [MyComponent]::Reflect() function.
            m_descriptors.insert(m_descriptors.end(), {
                TerrainShaperEditorSystemComponent::CreateDescriptor(),
            });
        }

        /**
         * Add required SystemComponents to the SystemEntity.
         * Non-SystemComponents should not be added here
         */
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList {
                azrtti_typeid<TerrainShaperEditorSystemComponent>(),
            };
        }
    };
}// namespace TerrainShaper

#if defined(O3DE_GEM_NAME)
AZ_DECLARE_MODULE_CLASS(AZ_JOIN(Gem_, O3DE_GEM_NAME, _Editor), TerrainShaper::TerrainShaperEditorModule)
#else
AZ_DECLARE_MODULE_CLASS(Gem_TerrainShaper_Editor, TerrainShaper::TerrainShaperEditorModule)
#endif
