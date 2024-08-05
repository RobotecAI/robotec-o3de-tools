
#include "RobotecSpectatorCameraEditorSystemComponent.h"
#include <RobotecSpectatorCamera/RobotecSpectatorCameraTypeIds.h>
#include <RobotecSpectatorCameraModuleInterface.h>
#include <SpectatorCamera/SpectatorCameraEditorComponent.h>

namespace RobotecSpectatorCamera
{
    class RobotecSpectatorCameraEditorModule : public RobotecSpectatorCameraModuleInterface
    {
    public:
        AZ_RTTI(RobotecSpectatorCameraEditorModule, RobotecSpectatorCameraEditorModuleTypeId, RobotecSpectatorCameraModuleInterface);
        AZ_CLASS_ALLOCATOR(RobotecSpectatorCameraEditorModule, AZ::SystemAllocator);

        RobotecSpectatorCameraEditorModule()
        {
            // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
            // Add ALL components descriptors associated with this gem to m_descriptors.
            // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and
            // EditContext. This happens through the [MyComponent]::Reflect() function.
            m_descriptors.insert(
                m_descriptors.end(),
                {
                    RobotecSpectatorCameraEditorSystemComponent::CreateDescriptor(),
                    SpectatorCameraEditorComponent::CreateDescriptor(),
                });
        }

        /**
         * Add required SystemComponents to the SystemEntity.
         * Non-SystemComponents should not be added here
         */
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList{
                azrtti_typeid<RobotecSpectatorCameraEditorSystemComponent>(),
            };
        }
    };
} // namespace RobotecSpectatorCamera

AZ_DECLARE_MODULE_CLASS(Gem_RobotecSpectatorCamera, RobotecSpectatorCamera::RobotecSpectatorCameraEditorModule)
