/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <Pointcloud/PointcloudBus.h>
#include <Clients/PointcloudAsset.h>
namespace Pointcloud
{
    class PointcloudSystemComponent
        : public AZ::Component
        , protected PointcloudRequestBus::Handler
    {
    public:
        AZ_COMPONENT_DECL(PointcloudSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        PointcloudSystemComponent();
        ~PointcloudSystemComponent();

    protected:
        ////////////////////////////////////////////////////////////////////////
        // PointcloudRequestBus interface implementation

        ////////////////////////////////////////////////////////////////////////

        ////////////////////////////////////////////////////////////////////////
        // AZ::Component interface implementation
        void Init() override;
        void Activate() override;
        void Deactivate() override;
        ////////////////////////////////////////////////////////////////////////
    private:
//        PointcloudAssetHandler* m_pointcloudAssetHandler;
    };

} // namespace Pointcloud
