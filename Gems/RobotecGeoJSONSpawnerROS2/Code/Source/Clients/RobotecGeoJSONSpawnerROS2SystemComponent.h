/*
 * Copyright (c) Robotec.ai sp. z o.o.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>

namespace RobotecGeoJSONSpawnerROS2
{
    class RobotecGeoJSONSpawnerROS2SystemComponent : public AZ::Component
    {
    public:
        AZ_COMPONENT_DECL(RobotecGeoJSONSpawnerROS2SystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        RobotecGeoJSONSpawnerROS2SystemComponent() = default;
        ~RobotecGeoJSONSpawnerROS2SystemComponent() = default;

    protected:
        ////////////////////////////////////////////////////////////////////////
        // AZ::Component interface implementation
        void Init() override;
        void Activate() override;
        void Deactivate() override;
        ////////////////////////////////////////////////////////////////////////
    };

} // namespace RobotecGeoJSONSpawnerROS2
