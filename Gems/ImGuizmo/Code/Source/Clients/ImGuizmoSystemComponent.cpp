
#include "ImGuizmoSystemComponent.h"

#include <ImGuizmo/ImGuizmoTypeIds.h>

#include <Atom/Feature/ImGui/SystemBus.h>
#include <Atom/RPI.Public/ViewportContext.h>
#include <Atom/RPI.Public/ViewportContextBus.h>
#include <AzCore/Math/Matrix3x3.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzFramework/Viewport/ViewportScreen.h>
#include <ImGuizmo/ImGuizmo.h>

namespace ImGuizmo
{
    namespace Conversions
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

        AZ::Vector4 Float16ToTranslation4D(const float* matrix)
        {
            return AZ::Vector4(matrix[12], matrix[13], matrix[14], matrix[15]);
        }

    } // namespace Conversions

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
                ->Event("AcquireHandle", &ImGuizmoRequestBus::Events::AcquireHandle)
                ->Event("ReleaseHandle", &ImGuizmoRequestBus::Events::ReleaseHandle)
                ->Event("GetGizmoTransform", &ImGuizmoRequestBus::Events::GetGizmoTransform)
                ->Event("SetGizmoTransform", &ImGuizmoRequestBus::Events::SetGizmoTransform)
                ->Event("SetGizmoVisible", &ImGuizmoRequestBus::Events::SetGizmoVisible)
                ->Event("GetGizmoVisible", &ImGuizmoRequestBus::Events::GetGizmoVisible)
                ->Event("SetGizmoLabel", &ImGuizmoRequestBus::Events::SetGizmoLabel)
                ->Event("GetGizmoLabel", &ImGuizmoRequestBus::Events::GetGizmoLabel)
                ->Event("GetIfManipulated", &ImGuizmoRequestBus::Events::GetIfManipulated)
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
        m_gizmoData.clear();
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
            ImGuiRender();
            return;
        }
        // As I understand, this is a way to poke the ImGuiSystemComponent to make sure that ImGuiContext is created.
        AZ::Render::ImGuiSystemRequestBus::BroadcastResult(
            m_imguiAvailable, &AZ::Render::ImGuiSystemRequests::PushActiveContextFromDefaultPass);
        AZ_Assert(m_imguiAvailable, "ImGuizmoComponent requires ImGui to be available. Make sure ImGuiSystemComponent is active.");
    }

    void ImGuizmoSystemComponent::ImGuiRender()
    {
        ImGuiContext* imGuiContext = nullptr;
        AZ::Render::ImGuiSystemRequestBus::BroadcastResult(imGuiContext, &AZ::Render::ImGuiSystemRequests::GetActiveContext);
        AZ_Assert(imGuiContext, "ImGui context is not available.");
        if (!imGuiContext)
        {
            return;
        }

        auto viewportContext = AZ::RPI::ViewportContextRequests::Get()->GetDefaultViewportContext();
        AZ_Assert(viewportContext, "Viewport context is not available.");
        if (!viewportContext)
        {
            return;
        }

        ImGuiContext* prevContext = ImGui::GetCurrentContext();
        ImGui::SetCurrentContext(imGuiContext);

        ImGuizmoRequests::GizmoHandle handleToDraw = ImGuizmoRequests::InvalidGizmoHandle;

        // if we are currently manipulating a gizmo, we want to draw that one
        // otherwise we want to draw the closest gizmo to the cursor
        if (m_currentlyManipulatedHandle != ImGuizmoRequests::InvalidGizmoHandle)
        {
            handleToDraw = m_currentlyManipulatedHandle;
        }
        else
        {
            AZStd::pair<float, GizmoHandle> closestGizmo{ std::numeric_limits<float>::max(), ImGuizmoRequests::InvalidGizmoHandle };
            for (auto& [handle, gizmoData] : m_gizmoData)
            {
                if (!gizmoData.m_gizmoVisible)
                {
                    continue;
                }
                const auto gizmoTranslation = Conversions::Float16ToTranslation4D(gizmoData.m_gizmoMatrix);
                const AZ::Vector4 localPosition = viewportContext->GetCameraViewMatrix() * gizmoTranslation;
                // Skip gizmos that are behind the camera
                if (localPosition.GetZ() > 0.0f)
                {
                    continue;
                }
                const AzFramework::ScreenSize windowSize{ static_cast<int>(viewportContext->GetViewportSize().m_width),
                                                          static_cast<int>(viewportContext->GetViewportSize().m_height) };

                AzFramework::ScreenPoint renderScreenpoint = AzFramework::WorldToScreen(
                    gizmoTranslation.GetAsVector3(),
                    viewportContext->GetCameraViewMatrixAsMatrix3x4(),
                    viewportContext->GetCameraProjectionMatrix(),
                    windowSize);

                gizmoData.m_screenLocation = AZ::Vector2(renderScreenpoint.m_x, renderScreenpoint.m_y);
                const ImVec2 mousePos = ImGui::GetMousePos();

                // compute distance to cursor
                AZ::Vector2 distanceToCursor = AZ::Vector2(mousePos.x - renderScreenpoint.m_x, mousePos.y - renderScreenpoint.m_y);
                float distance = distanceToCursor.GetLengthSq();
                if (distance < closestGizmo.first)
                {
                    closestGizmo = { distance, handle };
                }

                const auto& screen_x = gizmoData.m_screenLocation.GetX();
                const auto& screen_y = gizmoData.m_screenLocation.GetY();

                ImGui::SetNextWindowPos(ImVec2(screen_x, screen_y + 2.0f * ImGui::GetFrameHeight())); // Offset text slightly

                AZStd::string gizmoLabel = AZStd::string::format("%s##%d", gizmoData.m_name.c_str(), handle);
                ImGui::Begin(
                    gizmoLabel.c_str(),
                    nullptr,
                    ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove);
                ImGui::Text("%s", gizmoData.m_name.c_str());
                ImGui::End();
            }
            if (closestGizmo.second != InvalidGizmoHandle)
            {
                handleToDraw = closestGizmo.second;
            }
        }

        if (handleToDraw != ImGuizmoRequests::InvalidGizmoHandle)
        {
            AZ_Assert(m_gizmoData.contains(handleToDraw), "Gizmo handle not found");
            auto& gizmoData = m_gizmoData[handleToDraw];
            ImGuizmo::BeginFrame();
            ImGuizmo::Enable(true);
            ImGuizmo::SetRect(0, 0, viewportContext->GetViewportSize().m_width, viewportContext->GetViewportSize().m_height);

            float view[16];
            float projection[16];

            viewportContext->GetCameraProjectionMatrix().StoreToColumnMajorFloat16(projection);
            viewportContext->GetCameraViewMatrix().StoreToColumnMajorFloat16(view);
            ImGuizmo::AllowAxisFlip(false);
            ImGuizmo::Manipulate(view, projection, gizmoData.m_operation, gizmoData.m_mode, gizmoData.m_gizmoMatrix);
            gizmoData.m_manipulated = ImGuizmo::IsUsing();
            m_currentlyManipulatedHandle = ImGuizmo::IsUsing() ? handleToDraw : ImGuizmoRequests::InvalidGizmoHandle;
        }
        ImGui::SetCurrentContext(prevContext);
    }

    ImGuizmoRequests::GizmoHandle ImGuizmoSystemComponent::AcquireHandle(const AZ::Transform& transform, const AZStd::string& name)
    {
        ImGuizmoRequests::GizmoHandle handle = m_nextHandle++;
        GizmoData data;
        data.m_name = name;
        data.m_operation = ImGuizmo::OPERATION::ROTATE | ImGuizmo::OPERATION::TRANSLATE | ImGuizmo::OPERATION::SCALE;
        data.m_mode = ImGuizmo::MODE::LOCAL;
        Conversions::AZTransformToFloat16(transform, data.m_gizmoMatrix);
        m_gizmoData[handle] = data;
        return handle;
    }

    void ImGuizmoSystemComponent::ReleaseHandle(ImGuizmoRequests::GizmoHandle handle)
    {
        m_gizmoData.erase(handle);
    }

    AZ::Transform ImGuizmoSystemComponent::GetGizmoTransform(ImGuizmoRequests::GizmoHandle handle)
    {
        const auto gizmoDataIt = m_gizmoData.find(handle);
        AZ_Assert(gizmoDataIt != m_gizmoData.end(), "Gizmo handle not found");
        if (gizmoDataIt == m_gizmoData.end())
        {
            return AZ::Transform::CreateIdentity();
        }
        return Conversions::Float16ToAZTransform(gizmoDataIt->second.m_gizmoMatrix);
    }

    void ImGuizmoSystemComponent::SetGizmoTransform(GizmoHandle handle, const AZ::Transform& transform)
    {
        if (m_gizmoData.find(handle) == m_gizmoData.end())
        {
            AZ_Assert(false, "Gizmo handle not found");
            return;
        }

        Conversions::AZTransformToFloat16(transform, m_gizmoData[handle].m_gizmoMatrix);
    }

    void ImGuizmoSystemComponent::SetGizmoVisible(ImGuizmoRequests::GizmoHandle handle, bool visible)
    {
        if (m_gizmoData.find(handle) == m_gizmoData.end())
        {
            AZ_Assert(false, "Gizmo handle not found");
            return;
        }
        m_gizmoData[handle].m_gizmoVisible = visible;
    }

    bool ImGuizmoSystemComponent::GetGizmoVisible(ImGuizmoRequests::GizmoHandle handle)
    {
        if (m_gizmoData.find(handle) == m_gizmoData.end())
        {
            AZ_Assert(false, "Gizmo handle not found");
            return false;
        }
        return m_gizmoData[handle].m_gizmoVisible;
    }

    void ImGuizmoSystemComponent::SetGizmoOperation(ImGuizmoRequests::GizmoHandle handle, OPERATION operation)
    {
        if (m_gizmoData.find(handle) == m_gizmoData.end())
        {
            AZ_Assert(false, "Gizmo handle not found");
            return;
        }
        m_gizmoData[handle].m_operation = operation;
    }

    OPERATION ImGuizmoSystemComponent::GetGizmoOperation(ImGuizmoRequests::GizmoHandle handle)
    {
        if (m_gizmoData.find(handle) == m_gizmoData.end())
        {
            AZ_Assert(false, "Gizmo handle not found");
            return ImGuizmo::OPERATION::UNIVERSAL;
        }
        return m_gizmoData[handle].m_operation;
    }

    void ImGuizmoSystemComponent::SetGizmoLabel(ImGuizmoRequests::GizmoHandle handle, const AZStd::string& name)
    {
        if (m_gizmoData.find(handle) == m_gizmoData.end())
        {
            AZ_Assert(false, "Gizmo handle not found");
            return;
        }
        m_gizmoData[handle].m_name = name;
    }

    AZStd::string ImGuizmoSystemComponent::GetGizmoLabel(ImGuizmoRequests::GizmoHandle handle)
    {
        if (m_gizmoData.find(handle) == m_gizmoData.end())
        {
            AZ_Assert(false, "Gizmo handle not found");
            return "";
        }
        return m_gizmoData[handle].m_name;
    }

    bool ImGuizmoSystemComponent::GetIfManipulated(ImGuizmoRequests::GizmoHandle handle)
    {
        if (m_gizmoData.find(handle) == m_gizmoData.end())
        {
            AZ_Assert(false, "Gizmo handle not found");
            return false;
        }
        return m_gizmoData[handle].m_manipulated;
    }

    void ImGuizmoSystemComponent::SetGizmoMode(ImGuizmoRequests::GizmoHandle handle, MODE mode)
    {
        if (m_gizmoData.find(handle) == m_gizmoData.end())
        {
            AZ_Assert(false, "Gizmo handle not found");
            return;
        }
        m_gizmoData[handle].m_mode = mode;
    }
} // namespace ImGuizmo
