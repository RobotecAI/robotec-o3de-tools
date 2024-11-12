#pragma once

#include <Atom/RPI.Public/AuxGeom/AuxGeomDraw.h>

#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <SplineTools/SplineToolsTypeIds.h>
namespace SplineTools
{
    class VisualizeSplineComponent
        : public AZ::Component
        , public AZ::TickBus::Handler
    {
    public:
        AZ_COMPONENT(VisualizeSplineComponent, VisualizeSplineComponentTypeId, AZ::Component);

        static void Reflect(AZ::ReflectContext* context);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        // AZ::Component overrides ...
        void Activate() override;
        void Deactivate() override;

        // AZ::TickBus::Handler overrides ...
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

    private:
        AZ::RPI::AuxGeomDrawPtr m_drawQueue;
        unsigned int m_resolution = 100;
        float m_nodeRadius = 0.1f;
    };
} // namespace SplineTools
