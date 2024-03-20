
#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>

namespace SplineTools
{
    class SplineToolsSystemComponent : public AZ::Component
    {
    public:
        AZ_COMPONENT_DECL(SplineToolsSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);

        SplineToolsSystemComponent() = default;
        ~SplineToolsSystemComponent() = default;

    protected:
        // AZ::Component interface implementation
        void Activate() override;
        void Deactivate() override;
    };

} // namespace SplineTools
