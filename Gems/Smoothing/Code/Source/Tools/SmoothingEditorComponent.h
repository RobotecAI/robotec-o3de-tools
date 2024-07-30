#pragma once

#include <Clients/SmoothingComponent.h>
#include <AzCore/Component/Component.h>
#include <AzToolsFramework/ToolsComponents/EditorComponentAdapter.h>
#include <AzToolsFramework/ToolsComponents/EditorComponentBase.h>

using SmoothingComponentEditorBase =
    AzToolsFramework::Components::EditorComponentAdapter<SmoothingComponentController, SmoothingComponent, SmoothingConfig>;

class SmoothingComponentEditorComponent : public SmoothingComponentEditorBase
{
public:
    SmoothingComponentEditorComponent() = default;
    explicit SmoothingComponentEditorComponent(const SmoothingConfig& configuration);

    AZ_EDITOR_COMPONENT(SmoothingComponentEditorComponent, "{01910451-5821-799e-b17a-908d53847ff3}", SmoothingComponentEditorBase);
    static void Reflect(AZ::ReflectContext* context);

    // SmoothingComponentEditorBase interface overrides...
    void Activate() override;
    void Deactivate() override;
    bool ShouldActivateController() const override;


};