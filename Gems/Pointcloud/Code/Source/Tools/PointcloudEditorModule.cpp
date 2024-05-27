
#include <Pointcloud/PointcloudTypeIds.h>
#include <PointcloudModuleInterface.h>
#include "PointcloudEditorSystemComponent.h"
#include "Components/EditorPointcloudComponent.h"

namespace Pointcloud
{
    class PointcloudEditorModule
        : public PointcloudModuleInterface
    {
    public:
        AZ_RTTI(PointcloudEditorModule, PointcloudEditorModuleTypeId, PointcloudModuleInterface);
        AZ_CLASS_ALLOCATOR(PointcloudEditorModule, AZ::SystemAllocator);

        PointcloudEditorModule()
        {
            // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
            // Add ALL components descriptors associated with this gem to m_descriptors.
            // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and EditContext.
            // This happens through the [MyComponent]::Reflect() function.
            m_descriptors.insert(m_descriptors.end(), {
                PointcloudEditorSystemComponent::CreateDescriptor(),
                PointcloudComponent::CreateDescriptor(),
                EditorPointcloudComponent::CreateDescriptor(),
            });
        }

        /**
         * Add required SystemComponents to the SystemEntity.
         * Non-SystemComponents should not be added here
         */
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList {
                azrtti_typeid<PointcloudEditorSystemComponent>(),
            };
        }
    };
}// namespace Pointcloud

#if defined(O3DE_GEM_NAME)
AZ_DECLARE_MODULE_CLASS(AZ_JOIN(Gem_, O3DE_GEM_NAME, _Editor), Pointcloud::PointcloudEditorModule)
#else
AZ_DECLARE_MODULE_CLASS(Gem_Pointcloud_Editor, Pointcloud::PointcloudEditorModule)
#endif
