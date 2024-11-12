#include "VisualizeSplineComponent.h"
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <Atom/RPI.Public/AuxGeom/AuxGeomDraw.h>
#include <Atom/RPI.Public/AuxGeom/AuxGeomFeatureProcessorInterface.h>
#include <Atom/RPI.Public/Scene.h>
#include <AzCore/Component/TransformBus.h>
#include <LmbrCentral/Shape/SplineComponentBus.h>
namespace SplineTools
{
    void VisualizeSplineComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<VisualizeSplineComponent, AZ::Component>()
                ->Version(0)
                ->Field("resolution", &VisualizeSplineComponent::m_resolution)
                ->Field("nodeRadius", &VisualizeSplineComponent::m_nodeRadius);
            if (auto* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<VisualizeSplineComponent>("VisualizeSplineComponent", "SplineToolsEditorComponent")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "VisualizeSplineComponent")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "RobotecTools")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &VisualizeSplineComponent::m_resolution,
                        "Resolution",
                        "Resolution of the spline visualization")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &VisualizeSplineComponent::m_nodeRadius,
                        "Node Radius",
                        "Radius of the nodes in the spline visualization");
            }
        }
    }
    void VisualizeSplineComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("TransformService"));
        required.push_back(AZ_CRC_CE("SplineService"));
    }

    void VisualizeSplineComponent::Activate()
    {
        auto* entityScene = AZ::RPI::Scene::GetSceneForEntityId(m_entity->GetId());
        m_drawQueue = AZ::RPI::AuxGeomFeatureProcessorInterface::GetDrawQueueForScene(entityScene);
        AZ_Assert(m_drawQueue, "AuxGeomFeatureProcessorInterface not available for scene");
        AZ::TickBus::Handler::BusConnect();
    }

    void VisualizeSplineComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        m_drawQueue = nullptr;
    }

    void VisualizeSplineComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        if (m_drawQueue)
        {
            AZ::Transform worldTm = AZ::Transform::CreateIdentity();
            AZ::TransformBus::EventResult(worldTm, GetEntityId(), &AZ::TransformBus::Events::GetWorldTM);

            AZ::SplinePtr splinePtr;
            LmbrCentral::SplineComponentRequestBus::EventResult(
                splinePtr, GetEntityId(), &LmbrCentral::SplineComponentRequestBus::Events::GetSpline);

            AZ_Assert(splinePtr, "SplineComponentRequestBus::Events::GetSpline failed");

            for (size_t i = 0; i < splinePtr->GetVertexCount(); i++)
            {
                AZ::Vector3 point = splinePtr->GetVertex(i);
                point = worldTm.TransformPoint(point);

                m_drawQueue->DrawSphere(point, m_nodeRadius, AZ::Colors::Red);
            }
            std::vector<AZ::Vector3> splineVertices;
            splineVertices.reserve(m_resolution);
            for (unsigned int i = 0; i < m_resolution; i++)
            {
                float t = static_cast<float>(i) / static_cast<float>(m_resolution);

                auto address = splinePtr->GetAddressByFraction(t);

                AZ::Vector3 point = splinePtr->GetPosition(address);
                splineVertices.push_back(worldTm.TransformPoint(point));
            }

            AZ::RPI::AuxGeomDraw::AuxGeomDynamicDrawArguments drawArgs;
            drawArgs.m_verts = splineVertices.data();
            drawArgs.m_vertCount = splineVertices.size();
            drawArgs.m_colors = &AZ::Colors::White;
            drawArgs.m_colorCount = 1u;
            drawArgs.m_opacityType = AZ::RPI::AuxGeomDraw::OpacityType::Opaque;
            drawArgs.m_size = 1u;
            m_drawQueue->DrawLines(drawArgs);
        }
    }

} // namespace SplineTools
