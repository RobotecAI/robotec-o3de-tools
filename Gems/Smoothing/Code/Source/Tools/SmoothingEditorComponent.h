#pragma once

#include <AzCore/Component/Component.h>
#include <AzToolsFramework/ToolsComponents/EditorComponentAdapter.h>
#include <AzToolsFramework/ToolsComponents/EditorComponentBase.h>
#include <Clients/SmoothingComponent.h>

namespace Smoothing
{
    using SmoothingComponentEditorBase =
        AzToolsFramework::Components::EditorComponentAdapter<SmoothingComponentController, SmoothingComponent, SmoothingConfig>;

    class SmoothingComponentEditorComponent : public SmoothingComponentEditorBase
    {
    public:
        SmoothingComponentEditorComponent() = default;
        explicit SmoothingComponentEditorComponent(const SmoothingConfig& configuration);

        AZ_EDITOR_COMPONENT(SmoothingComponentEditorComponent, SmoothingComponentEditorComponentTypeId, SmoothingComponentEditorBase);
        static void Reflect(AZ::ReflectContext* context);

        // SmoothingComponentEditorBase interface overrides...
        void Activate() override;
        void Deactivate() override;
        bool ShouldActivateController() const override;

    private:
        bool m_activeInEditor = false;
        void OnActiveInEditorChanged();
    };
} // namespace Smoothing
