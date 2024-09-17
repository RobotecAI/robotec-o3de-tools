
#pragma once

#include <AzCore/Component/Component.h>

namespace CsvSpawner
{
    class CsvSpawnerSystemComponent : public AZ::Component
    {
    public:
        AZ_COMPONENT_DECL(CsvSpawnerSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        CsvSpawnerSystemComponent() = default;
        ~CsvSpawnerSystemComponent() = default;

    protected:
        ////////////////////////////////////////////////////////////////////////
        // AZ::Component interface implementation
        void Init() override;
        void Activate() override;
        void Deactivate() override;
        ////////////////////////////////////////////////////////////////////////
    };

} // namespace CsvSpawner
