
#include <RandomizeUtils/RandomizeUtilsTypeIds.h>
#include <RandomizeUtilsModuleInterface.h>

namespace RandomizeUtils
{
    class RandomizeUtilsEditorModule
        : public RandomizeUtilsModuleInterface
    {
    public:
        AZ_RTTI(RandomizeUtilsEditorModule, RandomizeUtilsEditorModuleTypeId, RandomizeUtilsModuleInterface);
        AZ_CLASS_ALLOCATOR(RandomizeUtilsEditorModule, AZ::SystemAllocator);

        RandomizeUtilsEditorModule()
        {
            // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
            // Add ALL components descriptors associated with this gem to m_descriptors.
            // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and EditContext.
            // This happens through the [MyComponent]::Reflect() function.
            m_descriptors.insert(m_descriptors.end(), {
            });
        }

        /**
         * Add required SystemComponents to the SystemEntity.
         * Non-SystemComponents should not be added here
         */
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList {
            };
        }
    };
}// namespace RandomizeUtils

#if defined(O3DE_GEM_NAME)
AZ_DECLARE_MODULE_CLASS(AZ_JOIN(Gem_, O3DE_GEM_NAME, _Editor), RandomizeUtils::RandomizeUtilsEditorModule)
#else
AZ_DECLARE_MODULE_CLASS(Gem_RandomizeUtils_Editor, RandomizeUtils::RandomizeUtilsEditorModule)
#endif
