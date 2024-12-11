
#include <Clients/ViewportStreamerComponent.h>
#include <ViewportStreamer/ViewportStreamerTypeIds.h>
#include <ViewportStreamerModuleInterface.h>

namespace ViewportStreamer
{
    class ViewportStreamerEditorModule : public ViewportStreamerModuleInterface
    {
    public:
        AZ_RTTI(ViewportStreamerEditorModule, ViewportStreamerEditorModuleTypeId, ViewportStreamerModuleInterface);
        AZ_CLASS_ALLOCATOR(ViewportStreamerEditorModule, AZ::SystemAllocator);

        ViewportStreamerEditorModule()
        {
            // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
            // Add ALL components descriptors associated with this gem to m_descriptors.
            // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and
            // EditContext. This happens through the [MyComponent]::Reflect() function.
            m_descriptors.insert(
                m_descriptors.end(),
                {
                    ViewportStreamerComponent::CreateDescriptor(),
                });
        }

        /**
         * Add required SystemComponents to the SystemEntity.
         * Non-SystemComponents should not be added here
         */
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList{};
        }
    };
} // namespace ViewportStreamer

#if defined(O3DE_GEM_NAME)
AZ_DECLARE_MODULE_CLASS(AZ_JOIN(Gem_, O3DE_GEM_NAME, _Editor), ViewportStreamer::ViewportStreamerEditorModule)
#else
AZ_DECLARE_MODULE_CLASS(Gem_ViewportStreamer_Editor, ViewportStreamer::ViewportStreamerEditorModule)
#endif
