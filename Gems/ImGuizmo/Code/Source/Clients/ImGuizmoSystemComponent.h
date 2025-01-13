
#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/Console/IConsole.h>
#include <ImGuizmo/ImGuizmoBus.h>

namespace ImGuizmo
{
    namespace ImGuizmoConsoleCommands
    {
        static ImGuizmoRequests::GizmoHandle ArgumentsToHandle(const AZ::ConsoleCommandContainer& arguments)
        {
            if (arguments.size() < 1)
            {
                AZ_Printf("ImGuizmo", "No handle provided\n");
                return ImGuizmoRequests::InvalidGizmoHandle;
            }
            return AZStd::stoi(AZStd::string(arguments[0].data(), arguments[0].size()));
        };

        static void imguizmo_acquire(const AZ::ConsoleCommandContainer& arguments)
        {
            AZ_UNUSED(arguments);
            // Acquire a gizmo handle
            ImGuizmoRequests::GizmoHandle handle = ImGuizmoRequests::InvalidGizmoHandle;
            ImGuizmoRequestBus::BroadcastResult(
                handle, &ImGuizmoRequestBus::Events::AcquireHandle, AZ::Transform::CreateIdentity(), "TestGizmo");
            AZ_Printf("ImGuizmo", "Gizmo handle: %d\n", handle);
        }

        static void imguizmo_show(const AZ::ConsoleCommandContainer& arguments)
        {
            auto handle = ArgumentsToHandle(arguments);
            ImGuizmoRequestBus::Broadcast(&ImGuizmoRequestBus::Events::SetGizmoVisible, handle, true);
        }

        static void imguizmo_hide(const AZ::ConsoleCommandContainer& arguments)
        {
            auto handle = ArgumentsToHandle(arguments);
            ImGuizmoRequestBus::Broadcast(&ImGuizmoRequestBus::Events::SetGizmoVisible, handle, false);
        }

        static void imguizmo_local(const AZ::ConsoleCommandContainer& arguments)
        {
            auto handle = ArgumentsToHandle(arguments);
            ImGuizmoRequestBus::Broadcast(&ImGuizmoRequestBus::Events::SetGizmoMode, handle, ImGuizmo::MODE::LOCAL);
        }

        static void imguizmo_world(const AZ::ConsoleCommandContainer& arguments)
        {
            auto handle = ArgumentsToHandle(arguments);
            ImGuizmoRequestBus::Broadcast(&ImGuizmoRequestBus::Events::SetGizmoMode, handle, ImGuizmo::MODE::WORLD);
        }

        AZ_CONSOLEFREEFUNC(imguizmo_acquire, AZ::ConsoleFunctorFlags::DontReplicate, "Show imguizmo gizmo");
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
        //! GizmoData struct to store gizmo data
        struct GizmoData
        {
            AZStd::string m_name;
            OPERATION m_operation;
            MODE m_mode;
            bool m_gizmoVisible = false;
            bool m_manipulated = false;
            float m_gizmoMatrix[16];
        };

        AZStd::unordered_map<GizmoHandle, GizmoData> m_gizmoData;

        // ImGuizmoRequestBus overrides ...
        ImGuizmoRequests::GizmoHandle AcquireHandle(const AZ::Transform& transform, const AZStd::string& name) override;
        void ReleaseHandle(ImGuizmoRequests::GizmoHandle handle) override;
        AZ::Transform GetGizmoTransform(ImGuizmoRequests::GizmoHandle handle) override;
        void SetGizmoTransform(ImGuizmoRequests::GizmoHandle handle, const AZ::Transform& transform) override;
        void SetGizmoVisible(ImGuizmoRequests::GizmoHandle handle, bool visible) override;
        bool GetGizmoVisible(ImGuizmoRequests::GizmoHandle handle) override;
        void SetGizmoOperation(ImGuizmoRequests::GizmoHandle handle, OPERATION operation) override;
        OPERATION GetGizmoOperation(ImGuizmoRequests::GizmoHandle handle) override;
        void SetGizmoLabel(ImGuizmoRequests::GizmoHandle handle, const AZStd::string& name) override;
        AZStd::string GetGizmoLabel(ImGuizmoRequests::GizmoHandle handle) override;
        bool GetIfManipulated(ImGuizmoRequests::GizmoHandle handle) override;
        void SetGizmoMode(ImGuizmoRequests::GizmoHandle handle, MODE mode) override;

        // AZ::Component overrides ...
        void Init() override;
        void Activate() override;
        void Deactivate() override;

        // AZ::TickBus::Handler overrides ...
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

        void ImGuiRender(float deltaTime);

        bool m_imguiAvailable = false;
        ImGuizmoRequests::GizmoHandle m_nextHandle{ 0 };
    };

} // namespace ImGuizmo
