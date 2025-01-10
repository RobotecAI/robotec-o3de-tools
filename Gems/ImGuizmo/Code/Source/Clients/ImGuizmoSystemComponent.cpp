
#include "ImGuizmoSystemComponent.h"

#include <ImGuizmo/ImGuizmoTypeIds.h>

#include <Atom/Feature/ImGui/SystemBus.h>
#include <Atom/RPI.Public/ViewportContext.h>
#include <Atom/RPI.Public/ViewportContextBus.h>
#include <AzCore/Math/Matrix3x3.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <ImGuizmo/ImGuizmo.h>

namespace ImGuizmo
{
    namespace conversions
    {

        AZ::Transform Float16ToAZTransform(const float* matrix)
        {
            AZ::Matrix4x4 mat = AZ::Matrix4x4::CreateFromColumnMajorFloat16(matrix);
            return AZ::Transform::CreateFromMatrix3x3AndTranslation(AZ::Matrix3x3::CreateFromMatrix4x4(mat), mat.GetTranslation());
        }

        void AZTransformToFloat16(const AZ::Transform& transform, float* matrix)
        {
            AZ::Matrix4x4 mat = AZ::Matrix4x4::CreateFromTransform(transform);
            mat.StoreToColumnMajorFloat16(matrix);
        }

    } // namespace conversions

    AZ_COMPONENT_IMPL(ImGuizmoSystemComponent, "ImGuizmoSystemComponent", ImGuizmoSystemComponentTypeId);

    void ImGuizmoSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ImGuizmoSystemComponent, AZ::Component>()->Version(0);
        }
        if (auto behaviorContext = azrtti_cast<AZ::BehaviorContext*>(context))
        {
            behaviorContext->EBus<ImGuizmoRequestBus>("ImGuizmoRequestBus")
                ->Attribute(AZ::Edit::Attributes::Category, "ImGuizmo")
                ->Event("SetGizmoTransform", &ImGuizmoRequestBus::Events::SetGizmoTransform)
                ->Event("GetGizmoTransform", &ImGuizmoRequestBus::Events::GetGizmoTransform)
                ->Event("SetGizmoVisible", &ImGuizmoRequestBus::Events::SetGizmoVisible)
                ->Event("SetGizmoModeLocal", &ImGuizmoRequestBus::Events::SetGizmoModeLocal)
                ->Event("SetGizmoModeWorld", &ImGuizmoRequestBus::Events::SetGizmoModeWorld);
        }
    }

    void ImGuizmoSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("ImGuizmoService"));
    }

    void ImGuizmoSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("ImGuizmoService"));
    }

    void ImGuizmoSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }

    void ImGuizmoSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    ImGuizmoSystemComponent::ImGuizmoSystemComponent()
    {
        if (ImGuizmoInterface::Get() == nullptr)
        {
            ImGuizmoInterface::Register(this);
        }
    }

    ImGuizmoSystemComponent::~ImGuizmoSystemComponent()
    {
        if (ImGuizmoInterface::Get() == this)
        {
            ImGuizmoInterface::Unregister(this);
        }
    }

    void ImGuizmoSystemComponent::Init()
    {
    }

    void ImGuizmoSystemComponent::Activate()
    {
        m_imguiAvailable = false;
        ImGuizmoRequestBus::Handler::BusConnect();
        AZ::TickBus::Handler::BusConnect();
    }

    void ImGuizmoSystemComponent::Deactivate()
    {
        m_imguiAvailable = false;
        AZ::TickBus::Handler::BusDisconnect();
        ImGuizmoRequestBus::Handler::BusDisconnect();
    }

    void ImGuizmoSystemComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        if (m_imguiAvailable)
        {
            ImGuiRender(deltaTime);
            return;
        }
        // As I understand, this is a way to poke the ImGuiSystemComponent to make sure that ImGuiContext is created.
        AZ::Render::ImGuiSystemRequestBus::BroadcastResult(
            m_imguiAvailable, &AZ::Render::ImGuiSystemRequests::PushActiveContextFromDefaultPass);
        AZ_Assert(m_imguiAvailable, "ImGuizmoComponent requires ImGui to be available. Make sure ImGuiSystemComponent is active.");
    }

    void ImGuizmoSystemComponent::ImGuiRender([[maybe_unused]] float deltaTime)
    {
        if (m_gizmoVisible)
        {
            ImGuiContext* imGuiContext = nullptr;
            AZ::Render::ImGuiSystemRequestBus::BroadcastResult(imGuiContext, &AZ::Render::ImGuiSystemRequests::GetActiveContext);
            AZ_Assert(imGuiContext, "ImGui context is not available.");
            if (!imGuiContext)
            {
                return;
            }
            ImGui::SetCurrentContext(imGuiContext);

            auto viewportContext = AZ::RPI::ViewportContextRequests::Get()->GetDefaultViewportContext();
            AZ_Assert(viewportContext, "Viewport context is not available.");
            if (!viewportContext)
            {
                return;
            }

            ImGuizmo::BeginFrame();
            ImGuizmo::Enable(true);
            ImGuizmo::SetRect(0, 0, viewportContext->GetViewportSize().m_width, viewportContext->GetViewportSize().m_height);

            float view[16];
            float projection[16];

            viewportContext->GetCameraProjectionMatrix().StoreToColumnMajorFloat16(projection);
            viewportContext->GetCameraViewMatrix().StoreToColumnMajorFloat16(view);
            ImGuizmo::AllowAxisFlip(false);
            ImGuizmo::Manipulate(view, projection, m_gizmoOperation, m_gizmoMode, m_gizmoMatrix);
        }
    }

    AZ::Transform ImGuizmoSystemComponent::GetGizmoTransform()
    {
        return conversions::Float16ToAZTransform(m_gizmoMatrix);
    }

    void ImGuizmoSystemComponent::SetGizmoTransform(const AZ::Transform& transform)
    {
        conversions::AZTransformToFloat16(transform, m_gizmoMatrix);
    }

    void ImGuizmoSystemComponent::SetGizmoVisible(bool visible)
    {
        m_gizmoVisible = visible;
    }

    void ImGuizmoSystemComponent::SetGizmoOperation(ImGuizmo::OPERATION operation)
    {
        m_gizmoOperation = operation;
    }

    void ImGuizmoSystemComponent::SetGizmoMode(ImGuizmo::MODE mode)
    {
        m_gizmoMode = mode;
    }

} // namespace ImGuizmo
