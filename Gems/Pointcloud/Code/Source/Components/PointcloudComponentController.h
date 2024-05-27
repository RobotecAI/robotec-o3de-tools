/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Asset/AssetCommon.h>
#include <AzCore/Component/Component.h>
#include <AzCore/Component/TransformBus.h>

#include <Pointcloud/PointcloudFeatureProcessorInterface.h>

namespace Pointcloud
{
    class PointcloudComponentConfig final
        : public AZ::ComponentConfig
    {
    public:
        AZ_RTTI(PointcloudComponentConfig, "{8E9D09CB-1A73-41D1-A55F-FAD302BD49D0}", ComponentConfig);
        AZ_CLASS_ALLOCATOR(PointcloudComponentConfig, AZ::SystemAllocator);
        static void Reflect(AZ::ReflectContext* context);
        float m_pointSize = 1.0f;
        AZStd::string m_plyFilePath;
        PointcloudComponentConfig() = default;

        AZ::u64 m_entityId{ AZ::EntityId::InvalidEntityId };
    };

    class PointcloudComponentController final
        : public AZ::Data::AssetBus::MultiHandler
        , private AZ::TransformNotificationBus::Handler
    {
    public:
        friend class EditorPointcloudComponent;

        AZ_RTTI(PointcloudComponentController, "{37F67BFB-D71C-483C-A6E5-12E1AF3CE123}");
        AZ_CLASS_ALLOCATOR(PointcloudComponentController, AZ::SystemAllocator);

        static void Reflect(AZ::ReflectContext* context);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        PointcloudComponentController() = default;
        PointcloudComponentController(const PointcloudComponentConfig& config);

        void Activate(AZ::EntityId entityId);
        void Deactivate();
        void SetConfiguration(const PointcloudComponentConfig& config);
        const PointcloudComponentConfig& GetConfiguration() const;

    private:
        AZ::Crc32 OnConfigurationChanged();
        AZ::Crc32 OnLoadButton();
        AZ_DISABLE_COPY(PointcloudComponentController);

        // TransformNotificationBus overrides
        void OnTransformChanged(const AZ::Transform& local, const AZ::Transform& world) override;

        // handle for this probe in the feature processor
        PointcloudHandle m_handle;

        PointcloudFeatureProcessorInterface* m_featureProcessor = nullptr;
        AZ::TransformInterface* m_transformInterface = nullptr;
        AZ::EntityId m_entityId;
        
        PointcloudComponentConfig m_configuration;
        AZ::RPI::Scene* m_scene;

    };
}
