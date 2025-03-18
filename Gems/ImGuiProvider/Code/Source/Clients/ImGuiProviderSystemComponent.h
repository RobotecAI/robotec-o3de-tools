
#pragma once

#include "Utils/MenuTreeUtilities.h"
#include <AzCore/Component/Component.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/base.h>
#include <AzCore/std/containers/unordered_set.h>
#include <AzCore/std/optional.h>
#include <AzCore/std/string/string.h>
#include <ImGuiBus.h>
#include <ImGuiProvider/ImGuiProviderBus.h>

namespace ImGuiProvider
{

    struct FeatureWithState
    {
        ImGuiFeaturePath m_featurePath;
        ImGui::DisplayState m_state = ImGui::DisplayState::Hidden;
    };

    class ImGuiProviderSystemComponent
        : public AZ::Component
        , protected ImGuiProviderRequestBus::Handler
        , public AZ::TickBus::Handler
    {
    public:
        AZ_COMPONENT_DECL(ImGuiProviderSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        ImGuiProviderSystemComponent();
        ~ImGuiProviderSystemComponent();

    protected:
        ////////////////////////////////////////////////////////////////////////
        // ImGuiProviderRequestBus interface implementation
        AZStd::optional<ImGuiFeaturePath> GetActiveGuiId() override;
        void SetActiveGUI(const ImGuiFeaturePath& guiIdToActivate) override;
        ////////////////////////////////////////////////////////////////////////

        ////////////////////////////////////////////////////////////////////////
        // AZ::Component interface implementation
        void Init() override;
        void Activate() override;
        void Deactivate() override;
        ////////////////////////////////////////////////////////////////////////

        ////////////////////////////////////////////////////////////////////////
        // AZTickBus interface implementation
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;
        ////////////////////////////////////////////////////////////////////////
        void SetEditorPlayModeState(bool state)
        {
            m_isEditorPlayModeActive = state;
        }

    private:
        void RenderMenuRecursive(const GUIMenuNode& node);
        void NotifyGUIChange(const ImGuiFeaturePath& featureToNotify);
        void RestoreStateAfterDebug();
        [[nodiscard]] bool IsDebugGUIDeactivated();

        void MonitorHandlersCount();
        void UnregisterFeatureByPath(const ImGuiFeaturePath& path);
        void RegisterFeatureInGUI(const ImGuiFeaturePath& featurePath);

        //! notifies viewport buses if imgui is active and commands to shift position of viewport icons
        void MoveViewportIconsDown();

        ImGuiContext* m_currentImGuiContext = nullptr;
        ImGuiContext* m_previousImGuiContext = nullptr;
        FeatureWithState m_currentGUIOwner;
        AZStd::unordered_set<ImGuiFeaturePath> m_registeredFeatures;
        GUIMenuNode m_menuRoot;
        bool m_guiHideOrderSent = false;
        bool m_isEditorPlayModeActive = false;
        bool m_appInEditor = false;
        bool m_showFeature = false;
    };

} // namespace ImGuiProvider
