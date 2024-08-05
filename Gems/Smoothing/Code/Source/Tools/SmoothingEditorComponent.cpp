#include "SmoothingEditorComponent.h"

SmoothingComponentEditorComponent::SmoothingComponentEditorComponent(const SmoothingConfig& configuration)
    : SmoothingComponentEditorBase(configuration)
{
}

void SmoothingComponentEditorComponent::Reflect(AZ::ReflectContext* context)
{
    SmoothingComponentEditorBase::Reflect(context);
    AZ::SerializeContext* serializeContext = azrtti_cast<AZ::SerializeContext*>(context);
    if (serializeContext)
    {
        serializeContext->Class<SmoothingComponentEditorComponent, SmoothingComponentEditorBase>()->Version(1);

        AZ::EditContext* editContext = serializeContext->GetEditContext();
        if (editContext)
        {
            editContext->Class<SmoothingComponentEditorComponent>("SmoothingComponentEditorComponent", "SmoothingComponentEditorComponent")
                ->ClassElement(AZ::Edit::ClassElements::EditorData, "Component allows to provide georeference level for the level")
                ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                ->Attribute(AZ::Edit::Attributes::Visibility, AZ::Edit::PropertyVisibility::ShowChildrenOnly);
        }
    }
}

void SmoothingComponentEditorComponent::Activate()
{
    SmoothingComponentEditorBase::Activate();
}

void SmoothingComponentEditorComponent::Deactivate()
{
    SmoothingComponentEditorBase::Deactivate();
}

bool SmoothingComponentEditorComponent::ShouldActivateController() const
{
    return true;
};
