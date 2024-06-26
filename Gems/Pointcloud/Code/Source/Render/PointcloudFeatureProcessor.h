/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <Pointcloud/PointcloudFeatureProcessorInterface.h>
#include <Atom/RPI.Public/ViewportContextBus.h>
#include <Atom/RPI.Public/PipelineState.h>
#include <AzCore/Math/Transform.h>
#include <Atom/RPI.Reflect/Image/StreamingImageAsset.h>
#include <Atom/RPI.Public/Image/StreamingImage.h>
namespace Pointcloud
{
    class Scene;
    class Shader;

    class PointcloudFeatureProcessor
        : public PointcloudFeatureProcessorInterface
        , protected AZ::RPI::ViewportContextIdNotificationBus::Handler
        , protected AZ::Data::AssetBus::Handler
    {
    public:
        AZ_RTTI(PointcloudFeatureProcessor, "{B6EF8776-F7F9-432B-8BD9-D43869FFFC3D}", PointcloudFeatureProcessorInterface);
        AZ_CLASS_ALLOCATOR(PointcloudFeatureProcessor, AZ::SystemAllocator)

        static void Reflect(AZ::ReflectContext* context);

        PointcloudFeatureProcessor() = default;
        virtual ~PointcloudFeatureProcessor() = default;

        // PointcloudFeatureProcessorInterface
        // void SetTransform(const AZ::Transform& transform) override;
        // void SetPointSize(float pointSize) override;
        void SetParameters(const AZStd::vector<ShaderParameterUnion> &shaderParameters) override;
        void ForceUpdate(uint32_t totalVertices) override;
        AZStd::vector<ShaderParameterUnion> GetParameters() override;
        static AZ::Outcome<ParameterType, AZStd::string> ExtractParameterType(const AZ::Name &parameterName);

    protected:
        // RPI::SceneNotificationBus overrides
        void OnRenderPipelineChanged(AZ::RPI::RenderPipeline* pipeline, AZ::RPI::SceneNotification::RenderPipelineChangeType changeType) override;

        // Data::AssetBus overrides
        void OnAssetReloaded(AZ::Data::Asset<AZ::Data::AssetData> asset) override;

    private:
        void UpdateBackgroundClearColor();

        // FeatureProcessor overrides
        void Activate() override;
        void Deactivate() override;
        void Simulate(const FeatureProcessor::SimulatePacket& packet) override;
        void Render(const FeatureProcessor::RenderPacket& packet) override;

        void UpdateDrawPacket();
        void UpdateShaderConstants();
        AZStd::vector<float> ConvertToBuffer(const AZStd::vector<CloudVertex> &cloudVertexData);
        //! build a draw packet to draw the point cloud
        AZ::RHI::ConstPtr<AZ::RHI::DrawPacket> BuildDrawPacket(
                const AZ::Data::Instance<AZ::RPI::ShaderResourceGroup> &drawSrg,
                const AZ::Data::Instance<AZ::RPI::ShaderResourceGroup> &objectSrg,
                const AZ::RPI::Ptr<AZ::RPI::PipelineStateForDraw>& pipelineState,
                const AZ::RHI::DrawListTag& drawListTag,
                const AZStd::span<const AZ::RHI::StreamBufferView>& streamBufferViews,
                uint32_t vertexCount);

        AZ::RPI::Ptr<AZ::RPI::PipelineStateForDraw> m_meshPipelineState;
        AZ::RHI::DrawListTag m_drawListTag;
        AZ::RHI::ConstPtr<AZ::RHI::DrawPacket> m_drawPacket;
        AZ::Data::Instance<AZ::RPI::Shader> m_shader = nullptr;
        AZ::Data::Instance<AZ::RPI::ShaderResourceGroup> m_drawSrg = nullptr;
        AZ::Data::Instance<AZ::RPI::ShaderResourceGroup> m_objectSrg = nullptr;

        AZStd::array<AZ::RHI::StreamBufferView,1> m_meshStreamBufferViews;
        
        uint32_t m_totalVertices = 3;

        AzFramework::WindowSize m_viewportSize{0,0};
        [[maybe_unused]] bool m_updateShaderConstants = false;
        AZ::Transform m_transform = AZ::Transform::CreateIdentity();

        AZ::RHI::ShaderInputNameIndex m_modelMatrixIndex = "m_modelMatrix";
        AZ::RHI::ShaderInputNameIndex m_mBuffery = "m_f_positionBuffer";
        AZ::Data::Asset<AZ::RPI::StreamingImageAsset> texAsset;
        //bool m_isTextureValid = false;
        AZ::Data::Instance< AZ::RPI::StreamingImage> texture;
        AZ::RHI::ShaderInputNameIndex m_inputTextureImageIndex = "m_t2_inputTexture";
        // time index
        AZ::RHI::ShaderInputNameIndex m_timeIndex = "m_time";
        float m_time = 0.0f;
        // start time
        AZStd::chrono::system_clock::time_point m_startTime;
        AZStd::vector<ShaderParameterUnion> m_shaderParameters;
        uint32_t m_vertexCountPerMesh = 0;
    };
}
