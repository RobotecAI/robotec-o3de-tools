
#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/Console/IConsole.h>
#include <ImGuizmo/ImGuizmoBus.h>

namespace ImGuizmo
{
    namespace ImGuizmoConsoleCommands
    {
        static void imguizmo_show(const AZ::ConsoleCommandContainer& arguments)
        {
            AZ_UNUSED(arguments);
            ImGuizmoRequestBus::Broadcast(&ImGuizmoRequestBus::Events::SetGizmoVisible, true);
        }

        static void imguizmo_hide(const AZ::ConsoleCommandContainer& arguments)
        {
            AZ_UNUSED(arguments);
            ImGuizmoRequestBus::Broadcast(&ImGuizmoRequestBus::Events::SetGizmoVisible, false);
        }

        static void imguizmo_local(const AZ::ConsoleCommandContainer& arguments)
        {
            AZ_UNUSED(arguments);
            ImGuizmoRequestBus::Broadcast(&ImGuizmoRequestBus::Events::SetGizmoMode, ImGuizmo::MODE::LOCAL);
        }

        static void imguizmo_world(const AZ::ConsoleCommandContainer& arguments)
        {
            AZ_UNUSED(arguments);
            ImGuizmoRequestBus::Broadcast(&ImGuizmoRequestBus::Events::SetGizmoMode, ImGuizmo::MODE::WORLD);
        }

        AZ_CONSOLEFREEFUNC(imguizmo_show, AZ::ConsoleFunctorFlags::DontReplicate, "Show imguizmo gizmo");
        AZ_CONSOLEFREEFUNC(imguizmo_hide, AZ::ConsoleFunctorFlags::DontReplicate, "Hide imguizmo gizmo");
        AZ_CONSOLEFREEFUNC(imguizmo_local, AZ::ConsoleFunctorFlags::DontReplicate, "Set imguizmo gizmo to local mode");
        AZ_CONSOLEFREEFUNC(imguizmo_world, AZ::ConsoleFunctorFlags::DontReplicate, "Set imguizmo gizmo to world mode");

    } // namespace ImGuizmoConsoleCommands

    class ImGuizmoSystemComponent
        : public AZ::Component
        , protected ImGuizmoRequestBus::Handler
        , public AZ::TickBus::Handler
    {
    public:
        AZ_COMPONENT_DECL(ImGuizmoSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        ImGuizmoSystemComponent();
        ~ImGuizmoSystemComponent();

    protected:
        // ImGuizmoRequestBus overrides ...
        AZ::Transform GetGizmoTransform() override;
        void SetGizmoTransform(const AZ::Transform& transform) override;
        void SetGizmoVisible(bool visible) override;
        void SetGizmoOperation(ImGuizmo::OPERATION operation) override;
        void SetGizmoMode(ImGuizmo::MODE mode) override;

        // AZ::Component overrides ...
        void Init() override;
        void Activate() override;
        void Deactivate() override;

        // AZ::TickBus::Handler overrides ...
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

        void ImGuiRender(float deltaTime);

        bool m_imguiAvailable = false;
        bool m_gizmoVisible = false;
        float m_gizmoMatrix[16] = { 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f };
        ImGuizmo::OPERATION m_gizmoOperation = ImGuizmo::OPERATION::ROTATE | ImGuizmo::OPERATION::TRANSLATE | ImGuizmo::OPERATION::SCALE;
        ImGuizmo::MODE m_gizmoMode = ImGuizmo::MODE::LOCAL;
    };

} // namespace ImGuizmo
