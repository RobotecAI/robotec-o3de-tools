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
namespace Pointcloud
{
    class Scene;
    class Shader;

    class PointcloudFeatureProcessor
        : public PointcloudFeatureProcessorInterface
        , protected AZ::RPI::ViewportContextIdNotificationBus::Handler
        , protected AZ::Data::AssetBus::Handler
    {
        struct PointcloudHandleData
        {
            AZ::Data::Asset<PointcloudAsset> m_asset; //!< The asset that this handle is associated with
            AZStd::vector<PointcloudAsset::CloudVertex> m_vertices; //!< The vertices of the point cloud
            AZ::Transform m_transform = AZ::Transform::CreateIdentity();
            AZStd::array<AZ::RHI::StreamBufferView,1> m_meshStreamBufferViews;
            float m_pointSize = 1.0f;
            bool m_updateShaderConstants = false;
            AZ::Data::Instance<AZ::RPI::Buffer> m_cloudVertexBuffer = nullptr;
            AZ::RHI::ConstPtr<AZ::RHI::DrawPacket> m_drawPacket;

            AZ::Data::Instance<AZ::RPI::ShaderResourceGroup> m_drawSrg = nullptr;
        };
    public:
        AZ_RTTI(PointcloudFeatureProcessor, "{B6EF8776-F7F9-432B-8BD9-D43869FFFC3D}", PointcloudFeatureProcessorInterface);
        AZ_CLASS_ALLOCATOR(PointcloudFeatureProcessor, AZ::SystemAllocator)

        static void Reflect(AZ::ReflectContext* context);

        PointcloudFeatureProcessor() = default;
        virtual ~PointcloudFeatureProcessor() = default;

        // PointcloudFeatureProcessorInterface
        void SetTransform(const PointcloudHandle& handle, const AZ::Transform &transform) override;
        void SetPointSize(const PointcloudHandle& handle, float pointSize) override;
        PointcloudHandle AquirePointcloud(const AZ::Data::Asset<PointcloudAsset> &asset ) override;

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

        void UpdateDrawPacket(PointcloudHandleData &handleData);
        void UpdateShaderConstants(PointcloudHandleData &handleData);

        //! build a draw packet to draw the point cloud
        AZ::RHI::ConstPtr<AZ::RHI::DrawPacket> BuildDrawPacket(
                const AZ::Data::Instance<AZ::RPI::ShaderResourceGroup>& srg,
                const AZ::RPI::Ptr<AZ::RPI::PipelineStateForDraw>& pipelineState,
                const AZ::RHI::DrawListTag& drawListTag,
                const AZStd::span<const AZ::RHI::StreamBufferView>& streamBufferViews,
                uint32_t vertexCount);

        AZ::RPI::Ptr<AZ::RPI::PipelineStateForDraw> m_meshPipelineState;
        AZ::Data::Instance<AZ::RPI::Shader> m_shader = nullptr;
        AZ::RHI::DrawListTag m_drawListTag;


        AzFramework::WindowSize m_viewportSize{0,0};
        AZ::RHI::ShaderInputNameIndex m_pointSizeIndex = "m_pointSize";
        AZ::RHI::ShaderInputNameIndex m_modelMatrixIndex = "m_modelMatrix";
        AZStd::unordered_map<PointcloudHandle, PointcloudHandleData> m_pointcloudHandles;

        PointcloudHandle  m_nextHandle = 1;

    };
}
