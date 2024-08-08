/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "GrassComponent.h"
#include <Atom/RPI.Public/Scene.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <Grass/GrassTypeIds.h>
#include <Render/GrassFeatureProcessor.h>
#include "../Tools/Components/ShaderParameter.h"

namespace Grass
{

    GrassComponent::GrassComponent(uint32_t totalVertices, AZStd::vector<ShaderParameter> shaderParameters)
        : m_shaderParameters(shaderParameters)
        , m_totalVertices(totalVertices)
    {
    }

    void GrassComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<GrassComponent, AZ::Component>()
                ->Version(0)
                ->Field("ShaderParameters", &GrassComponent::m_shaderParameters)
                ->Field("TotalVertices", &GrassComponent::m_totalVertices);
        }
    }

    void GrassComponent::Activate()
    {
        AZ::SystemTickBus::QueueFunction(
            [this]()
            {
                m_scene = AZ::RPI::Scene::GetSceneForEntityId(GetEntityId());
                if (m_scene) {
                    m_featureProcessor = m_scene->EnableFeatureProcessor<GrassFeatureProcessor>();
                    AZ_Assert(m_featureProcessor, "Failed to enable GrassFeatureProcessorInterface.");
                    m_featureProcessor->ForceUpdate(m_totalVertices);
                    PushNewParameters();
                    printf("Gamemode grass activated\n");
                }

            });
        AZ::TransformNotificationBus::Handler::BusConnect(GetEntityId());
    }

    void GrassComponent::Deactivate()
    {
        AZ::TransformNotificationBus::Handler::BusDisconnect();
        //m_featureProcessor->ReleaseGrass(m_grassHandle);
    }

    AZStd::vector<ShaderParameterUnion> GrassComponent::GetConstants() const {
        AZStd::vector<ShaderParameterUnion> shaderParameterUnions;
        for (const auto &param : m_shaderParameters) {
            shaderParameterUnions.push_back(param.ToShaderParameterUnion());
        }
        return shaderParameterUnions;
    }

    void GrassComponent::PushNewParameters() {
        m_featureProcessor->SetParameters(GetConstants());
    }
    

    void GrassComponent::OnTransformChanged(const AZ::Transform& local, const AZ::Transform& world)
    {
        // AZ_UNUSED(local);
        // if (m_grassHandle != GrassFeatureProcessorInterface::InvalidGrassHandle)
        // {
        //     m_featureProcessor->SetTransform(m_grassHandle, world);
        // }
    }

} // namespace Grass
