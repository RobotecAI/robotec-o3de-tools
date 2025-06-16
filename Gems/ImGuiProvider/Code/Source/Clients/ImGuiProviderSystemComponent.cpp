
#include "ImGuiProviderSystemComponent.h"
#include <Atom/Feature/ImGui/SystemBus.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/Console/IConsole.h>
#include <AzCore/Debug/Trace.h>
#include <AzCore/base.h>
#include <AzCore/std/algorithm.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/numeric.h>
#include <AzCore/std/smart_ptr/make_shared.h>
#include <AzCore/std/string/string.h>
#include <AzCore/std/utility/pair.h>
#include <AzFramework/Viewport/ViewportBus.h>
#include <ImGui/ImGuiPass.h>

#include <ImGuiBus.h>
#include <ImGuiProvider/ImGuiProviderBus.h>
#include <ImGuiProvider/ImGuiProviderTypeIds.h>
#include <imgui/imgui.h>

#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Serialization/SerializeContext.h>

namespace ImGuiProvider
{

    AZ_CVAR(bool, cl_hide_menu_bar, false, nullptr, AZ::ConsoleFunctorFlags::Null, "Hide Menu Bar if the flag is enabled");

    AZ_COMPONENT_IMPL(ImGuiProviderSystemComponent, "ImGuiProviderSystemComponent", ImGuiProviderSystemComponentTypeId);

    void ImGuiProviderSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ImGuiProviderSystemComponent, AZ::Component>()->Version(0);
        }
    }

    void ImGuiProviderSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("ImGuiProviderService"));
    }

    void ImGuiProviderSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("ImGuiProviderService"));
    }

    void ImGuiProviderSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }

    void ImGuiProviderSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    ImGuiProviderSystemComponent::ImGuiProviderSystemComponent()
    {
        if (ImGuiProviderInterface::Get() == nullptr)
        {
            ImGuiProviderInterface::Register(this);
        }
    }

    ImGuiProviderSystemComponent::~ImGuiProviderSystemComponent()
    {
        if (ImGuiProviderInterface::Get() == this)
        {
            ImGuiProviderInterface::Unregister(this);
        }
    }

    void ImGuiProviderSystemComponent::Init()
    {
    }

    void ImGuiProviderSystemComponent::Activate()
    {
        AZ::ApplicationTypeQuery appType;
        AZ::ComponentApplicationBus::Broadcast(&AZ::ComponentApplicationBus::Events::QueryApplicationType, appType);
        m_appInEditor = appType.IsEditor();

        if (!cl_hide_menu_bar)
        {
            ImGuiProviderRequestBus::Handler::BusConnect();
            AZ::TickBus::Handler::BusConnect();
        }
    }

    void ImGuiProviderSystemComponent::Deactivate()
    {
        if (AZ::TickBus::Handler::BusIsConnected())
        {
            AZ::TickBus::Handler::BusDisconnect();
        }
        if (ImGuiProviderRequestBus::Handler::BusIsConnected())
        {
            ImGuiProviderRequestBus::Handler::BusDisconnect();
        }
        AZStd::vector<ImGuiFeaturePath> features;
        for (auto& feature : m_registeredFeatures)
        {
            features.emplace_back(feature);
        }
        for (auto& feature : features)
        {
            UnregisterFeatureByPath(feature);
        }
    }

    void ImGuiProviderSystemComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        MonitorHandlersCount();
        // handling with m_appInEditor and m_isEditorPlayMode instead of activating system component in relevant event in editor system
        // component was choosen to handle potential system components which what to register their features in their activation. Using this
        // approach, ImguiProvider is able to register all request because it is active all the time

        // imgui notification bus is called called only if debug menu is shown, so it cannot be reliably used for imgui displaying

        if (m_registeredFeatures.empty())
        {
            return;
        }

        if (IsDebugGUIDeactivated())
        {
            AZ::Render::ImGuiSystemRequestBus::BroadcastResult(m_currentImGuiContext, &AZ::Render::ImGuiSystemRequests::GetActiveContext);
            AZ_Warning("ImGuiProviderSystemComponent::OnTick", m_currentImGuiContext, "ImGui context is not available.");
            // if current context is not available through Imgui system bus, get context from pass and push is to imgui system bus
            if (!m_currentImGuiContext)
            {
                AZ::Render::ImGuiPass* pass;
                AZ::Render::ImGuiSystemRequestBus::BroadcastResult(pass, &AZ::Render::ImGuiSystemRequests::GetDefaultImGuiPass);
                if (pass)
                {
                    auto context = pass->GetContext();
                    if (context)
                    {
                        AZ_Info("ImGuiProviderSystemComponent", "Gathering pass context and pushing as active");
                        m_currentImGuiContext = context;
                        m_previousImGuiContext = context;
                        ImGui::SetCurrentContext(context);
                    }
                }
            }
            // if needed move viewport icons
            MoveViewportIconsDown();
            RestoreStateAfterDebug();

            bool m_imguiAvailable = false;
            AZ::Render::ImGuiSystemRequestBus::BroadcastResult(
                m_imguiAvailable, &AZ::Render::ImGuiSystemRequests::PushActiveContextFromDefaultPass);
            AZ_Assert(
                m_imguiAvailable,
                "ImGuiProviderSystemComponent requires ImGui to be available. Make "
                "sure ImGuiSystemComponent is active.");
            m_previousImGuiContext = ImGui::GetCurrentContext();
            ImGui::SetCurrentContext(m_currentImGuiContext);
            if (m_imguiAvailable)
            {
                // Render the menu dynamically
                if (ImGui::BeginMainMenuBar())
                {
                    RenderMenuRecursive(m_menuRoot);
                    ImGui::EndMainMenuBar();
                }
                if (m_showFeature)
                {
                    AzFramework::ViewportImGuiNotificationBus::Broadcast(
                        &AzFramework::ViewportImGuiNotificationBus::Events::OnImGuiDropDownShown);
                    ImGuiProviderNotificationBus::Event(
                        m_currentGUIOwner.m_featurePath, &ImGuiProviderNotificationBus::Events::OnImGuiUpdate);
                }
                else
                {
                    AzFramework::ViewportImGuiNotificationBus::Broadcast(
                        &AzFramework::ViewportImGuiNotificationBus::Events::OnImGuiDropDownHidden);
                }
            }
            ImGui::SetCurrentContext(m_previousImGuiContext);
        }
        else if (!m_guiHideOrderSent)
        {
            m_showFeature = false;
            m_guiHideOrderSent = true;
            ImGuiProviderNotificationBus::Event(m_currentGUIOwner.m_featurePath, &ImGuiProviderNotifications::OnImGuiUnselected);
        }
    }

    void ImGuiProviderSystemComponent::MoveViewportIconsDown()
    {
        if (m_appInEditor && !m_isEditorPlayModeActive && !m_registeredFeatures.empty())
        {
            AzFramework::ViewportImGuiNotificationBus::Broadcast(&AzFramework::ViewportImGuiNotificationBus::Events::OnImGuiActivated);
        }
        else
        {
            AzFramework::ViewportImGuiNotificationBus::Broadcast(&AzFramework::ViewportImGuiNotificationBus::Events::OnImGuiDeactivated);
        }
    }

    // Recursively render the menu tree
    void ImGuiProviderSystemComponent::RenderMenuRecursive(const GUIMenuNode& node)
    {
        for (const auto& [name, child] : node.GetAllChildren())
        {
            if (child.HasChildren())
            { // If this node has children, create a submenu
                if (ImGui::BeginMenu(name.c_str()))
                {
                    RenderMenuRecursive(child);
                    ImGui::EndMenu();
                }
            }
            else
            { // If it's a leaf, create a menu item
                if (ImGui::MenuItem(name.c_str()))
                {
                    NotifyGUIChange(child.GetNodeLabel());
                }
            }
        }
    }

    void ImGuiProviderSystemComponent::NotifyGUIChange(const ImGuiFeaturePath& featureToNotify)
    {
        // get current state, before any changes
        bool sameWidgetButHidden =
            m_currentGUIOwner.m_featurePath == featureToNotify && m_currentGUIOwner.m_state == ImGui::DisplayState::Hidden;

        bool differentWidget = m_currentGUIOwner.m_featurePath != featureToNotify;

        // iif previous menu is valid and visible, hide it
        if (m_currentGUIOwner.m_state == ImGui::DisplayState::Visible)
        {
            m_showFeature = false;
            m_currentGUIOwner.m_state = ImGui::DisplayState::Hidden;
        }

        if (differentWidget || sameWidgetButHidden) // if the same widget was clicked it should stay disabled
        {
            ImGuiProviderNotificationBus::Event(m_currentGUIOwner.m_featurePath, &ImGuiProviderNotifications::OnImGuiUnselected);
            m_showFeature = true;
            m_currentGUIOwner.m_featurePath = featureToNotify;
            m_currentGUIOwner.m_state = ImGui::DisplayState::Visible;
            ImGuiProviderNotificationBus::Event(featureToNotify, &ImGuiProviderNotifications::OnImGuiSelected);
        }
        else // if the same was clicked, mark it hidden
        {
            m_currentGUIOwner.m_featurePath = featureToNotify;
            m_currentGUIOwner.m_state = ImGui::DisplayState::Hidden;
            ImGuiProviderNotificationBus::Event(featureToNotify, &ImGuiProviderNotifications::OnImGuiUnselected);
        }
    }

    bool ImGuiProviderSystemComponent::IsDebugGUIDeactivated()
    {
        // imguiManagerBus handles displaying debug imgui. its hidden state indicates that debug gui is inactive and regular gui can be
        // displayed. ImGui notificationBus or updateTick are active only when debug mode is enabled.
        ImGui::DisplayState debugImGuiState;
        ImGui::ImGuiManagerBus::BroadcastResult(debugImGuiState, &ImGui::ImGuiManagerBus::Events::GetDisplayState);
        return debugImGuiState == ImGui::DisplayState::Hidden;
    }

    void ImGuiProviderSystemComponent::RestoreStateAfterDebug()
    {
        if (m_guiHideOrderSent)
        {
            // reset state
            m_guiHideOrderSent = false;
            m_showFeature = m_currentGUIOwner.m_state == ImGui::DisplayState::Visible;
            ImGuiProviderNotificationBus::Event(m_currentGUIOwner.m_featurePath, &ImGuiProviderNotifications::OnImGuiSelected);
        }
    }

    void ImGuiProviderSystemComponent::MonitorHandlersCount()
    {
        if (m_registeredFeatures.size() != ImGuiProviderNotificationBus::GetTotalNumOfEventHandlers())
        {
            AZStd::vector<ImGuiFeaturePath> handlersToErase;
            for (auto feature : m_registeredFeatures)
            {
                handlersToErase.push_back(feature);
            }
            AZStd::vector<ImGuiFeaturePath> handleresToRegister;
            ImGuiProviderNotificationBus::EnumerateHandlers(
                [&handlersToErase, &handleresToRegister](ImGuiProviderNotifications* handler) -> bool
                {
                    ImGuiFeaturePath id = *(ImGuiProviderNotificationBus::GetCurrentBusId());
                    auto it = AZStd::find(handlersToErase.begin(), handlersToErase.end(), id);
                    if (it != handlersToErase.end())
                    {
                        handlersToErase.erase(it);
                    }
                    else
                    {
                        handleresToRegister.push_back(id);
                    }
                    return true;
                });
            // if no handlers were removed, this vector is empty
            for (auto path : handlersToErase)
            {
                UnregisterFeatureByPath(path);
            }
            // if not handlers were added, this vector is empty
            for (auto path : handleresToRegister)
            {
                RegisterFeatureInGUI(path);
            }
        }
    }

    // API implementation

    void ImGuiProviderSystemComponent::RegisterFeatureInGUI(const ImGuiFeaturePath& featurePath)
    {
        if (featurePath.empty())
        {
            AZ_Warning("ImGuiProviderSystemComponent", false, "Invalid path passed to ImGui registration");
            return;
        }
        AZ_Assert(
            !m_registeredFeatures.contains(featurePath),
            "Duplicated paths for ImGuiFeature are not supported. Duplicated path %s",
            featurePath.c_str());
        m_registeredFeatures.insert(featurePath);
        // insert into tree which represent structure of gui menus
        InsertIntoTree(m_menuRoot, featurePath);
    }

    AZStd::optional<ImGuiFeaturePath> ImGuiProviderSystemComponent::GetActiveGuiId()
    {
        if (m_currentGUIOwner.m_state == ImGui::DisplayState::Visible && IsDebugGUIDeactivated())
            return m_currentGUIOwner.m_featurePath;
        else
            return AZStd::nullopt;
    }

    void ImGuiProviderSystemComponent::SetActiveGUI(const ImGuiFeaturePath& guiIdToActivate)
    {
        NotifyGUIChange(guiIdToActivate);
    }

    void ImGuiProviderSystemComponent::UnregisterFeatureByPath(const ImGuiFeaturePath& path)
    {
        if (path.empty())
        {
            AZ_Warning("ImGuiProviderSystemComponent", false, "Failed to erase feature with empty path");
            return;
        }
        auto it = AZStd::find(m_registeredFeatures.begin(), m_registeredFeatures.end(), path);

        if (it == m_registeredFeatures.end())
        {
            AZ_Warning("ImGuiProviderSystemComponent", false, "Failed to erase feature with path: %s", path.String().c_str());
            return;
        }
        else
        {
            RemoveFromTree(m_menuRoot, *it);
            m_registeredFeatures.erase(*it);
        }
    }

} // namespace ImGuiProvider
