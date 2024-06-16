/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "PointcloudFeatureProcessor.h"
#include <Atom/RHI/DrawPacketBuilder.h>
#include <Atom/RPI.Public/RPIUtils.h>
#include <Atom/RHI.Reflect/InputStreamLayoutBuilder.h>
#include <Atom/RPI.Public/ViewportContext.h>
#include <Atom/RPI.Public/ViewportContextBus.h>
#include <fstream>
#include <Atom/RPI.Public/Pass/PassFilter.h>
#include <Atom/RPI.Reflect/Shader/ShaderAsset.h>
#include <Atom/RPI.Reflect/Asset/AssetUtils.h>
#include <AzCore/Name/NameDictionary.h>
namespace Pointcloud {
    void PointcloudFeatureProcessor::Reflect(AZ::ReflectContext *context) {
        if (auto *serializeContext = azrtti_cast<AZ::SerializeContext *>(context)) {
            serializeContext
                    ->Class<PointcloudFeatureProcessor, FeatureProcessor>();
        }
    }

    void PointcloudFeatureProcessor::Activate() {
        AZ_Printf("PointcloudFeatureProcessor", "PointcloudFeatureProcessor Activated");
        const char *shaderFilePath = "shaders/billboard2.azshader";
        m_shader = AZ::RPI::LoadCriticalShader(shaderFilePath);
        m_startTime = AZStd::chrono::system_clock::now();
        if (!m_shader) {
            printf("Failed to load required stars shader.\n");
            AZ_Error("PointcloudFeatureProcessor", false, "Failed to load required stars shader.");
            return;
        }
        AZ::Data::AssetBus::Handler::BusConnect(m_shader->GetAssetId());
        
        printf(" m_shader->GetAsset() %s Lol", m_shader->GetAsset()->GetName().GetCStr());
        //auto drawSrgLayout = m_shader->GetAsset()->GetDrawSrgLayout(m_shader->GetSupervariantIndex());
        auto drawSrgLayout = m_shader->GetAsset()->FindShaderResourceGroupLayout(AZ::Name("PerDrawSrg"));
        AZ_Error("PointcloudFeatureProcessor", drawSrgLayout,
                 "Failed to get the draw shader resource group layout for the pointcloud shader.");

        if (drawSrgLayout) {

            m_drawSrg = AZ::RPI::ShaderResourceGroup::Create(m_shader->GetAsset(), m_shader->GetSupervariantIndex(),
                                                             drawSrgLayout->GetName());
            m_pointSizeIndex.Reset();
            m_modelMatrixIndex.Reset();
            printf("Created SRG\n");

        }   else {
            printf("Failed to create SRG\n");
        }

        m_drawListTag = m_shader->GetDrawListTag();


        auto viewportContextInterface = AZ::Interface<AZ::RPI::ViewportContextRequestsInterface>::Get();
        AZ_Assert(viewportContextInterface,
                  "PointcloudFeatureProcessor requires the ViewportContextRequestsInterface.");
        auto viewportContext = viewportContextInterface->GetViewportContextByScene(GetParentScene());
        AZ_Assert(viewportContext, "PointcloudFeatureProcessor requires a valid ViewportContext.");
        m_viewportSize = viewportContext->GetViewportSize();

        EnableSceneNotification();

        AZ::RPI::ViewportContextIdNotificationBus::Handler::BusConnect(viewportContext->GetId());
        texAsset = AZ::RPI::AssetUtils::GetAssetByProductPath<AZ::RPI::StreamingImageAsset>("assets/untitled1.png.streamingimage");
        texAsset.QueueLoad();


    }

    AZStd::vector<float> PointcloudFeatureProcessor::ConvertToBuffer(const AZStd::vector<CloudVertex> &cloudVertexData){
        AZStd::vector<float> random_data(4 * 4 * cloudVertexData.size());
        auto m_worldMatrix = AZ::Matrix4x4::CreateFromTransform(m_transform);
        AZ::Matrix3x4 m_worldMatrix3x4;
        m_worldMatrix3x4.SetRows(m_worldMatrix.GetRow(0), m_worldMatrix.GetRow(1), m_worldMatrix.GetRow(2));
        for (int k = 0; k < cloudVertexData.size(); k++) {
            AZ::Transform transform = AZ::Transform::CreateFromMatrix3x4(m_worldMatrix3x4);
            AZ::Vector3 position = {cloudVertexData[k].m_position[0], cloudVertexData[k].m_position[1], cloudVertexData[k].m_position[2]};
            transform.SetTranslation(position);
            AZ::Matrix4x4 tmp_mat = AZ::Matrix4x4::CreateFromTransform(transform);
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    random_data[16 * k + i * 4 + j] = tmp_mat.GetElement(i, j);
                }
            }
        }
        return random_data;
    }

    void PointcloudFeatureProcessor::SetCloud(const AZStd::vector<CloudVertex> &cloudVertexData) {
        const uint32_t elementCount = static_cast<uint32_t>(cloudVertexData.size());
        const uint32_t elementSize = sizeof(float);

        AZStd::vector<float> cloudVertexDataBuffer = ConvertToBuffer(cloudVertexData);
        m_starsMeshData = cloudVertexDataBuffer;
        const uint32_t bufferSize = 4 * 4 * cloudVertexDataBuffer.size() * elementSize; // bytecount
        const uint32_t strideSize = 4 * 4 * elementSize;
        m_numStarsVertices = elementCount;


        if (!m_cloudVertexBuffer) {
            AZ::RPI::CommonBufferDescriptor desc;
            desc.m_poolType = AZ::RPI::CommonBufferPoolType::StaticInputAssembly;
            desc.m_bufferName = AZStd::string(m_mBuffery.GetNameForDebug().GetCStr());
            desc.m_byteCount = 4 * 4 * cloudVertexDataBuffer.size() * sizeof(float);
            desc.m_elementSize = elementSize;
            desc.m_bufferData = m_starsMeshData.data();
            desc.m_elementFormat = AZ::RHI::Format::R32_FLOAT;
            m_cloudVertexBuffer = AZ::RPI::BufferSystemInterface::Get()->CreateBufferFromCommonPool(desc);
            m_cloudVertexBuffer->WaitForUpload();

        } else {
            if (m_cloudVertexBuffer->GetBufferSize() != bufferSize) {
                m_cloudVertexBuffer->Resize(bufferSize);
            }

            m_cloudVertexBuffer->UpdateData(m_starsMeshData.data(), bufferSize);
            m_cloudVertexBuffer->WaitForUpload();
        }

        m_meshStreamBufferViews.front() = AZ::RHI::StreamBufferView(*m_cloudVertexBuffer->GetRHIBuffer(), 0, bufferSize,strideSize);
        //m_meshStreamBufferViews.front() = AZ::RHI::StreamBufferView(*m_cloudVertexBuffer->GetRHIBuffer(), 0, bufferSize,elementSize);
        UpdateDrawPacket();
        UpdateBackgroundClearColor();
        m_updateShaderConstants = true;
    }

    void PointcloudFeatureProcessor::UpdateDrawPacket() {
        // print if  things
        printf("PointcloudFeatureProcessor UpdateDrawPacket\n");
        // variables
        printf("m_meshPipelineState %s\n", m_meshPipelineState ? "true" : "false");
        printf("m_drawSrg %s\n", m_drawSrg ? "true" : "false");
        printf("m_cloudVertexBuffer %s\n", m_cloudVertexBuffer ? "true" : "false");

        if (m_meshPipelineState && m_drawSrg && m_cloudVertexBuffer) {
            printf ("PointcloudFeatureProcessor UpdateDrawPacket\n");
            m_drawPacket = BuildDrawPacket(m_drawSrg, m_meshPipelineState, m_drawListTag, m_meshStreamBufferViews,
                                           m_numStarsVertices);
        }
    }

    void PointcloudFeatureProcessor::OnAssetReloaded(AZ::Data::Asset<AZ::Data::AssetData> asset) {
        AZ_Printf("PointcloudFeatureProcessor", "PointcloudFeatureProcessor OnAssetReloaded");
        UpdateDrawPacket();
        UpdateBackgroundClearColor();
    }

    void PointcloudFeatureProcessor::Deactivate() {
        AZ_Printf("PointcloudFeatureProcessor", "PointcloudFeatureProcessor Deactivated");
        AZ::Data::AssetBus::Handler::BusDisconnect(m_shader->GetAssetId());
        AZ::RPI::ViewportContextIdNotificationBus::Handler::BusDisconnect();
    }

    void PointcloudFeatureProcessor::Simulate([[maybe_unused]] const FeatureProcessor::SimulatePacket &packet) {
        UpdateShaderConstants();
//        if (m_updateShaderConstants) {
//            m_updateShaderConstants = false;
//            UpdateShaderConstants();
//        }

    }

    void PointcloudFeatureProcessor::Render([[maybe_unused]] const FeatureProcessor::RenderPacket &packet) {
        auto time = AZStd::chrono::system_clock::now();
        // time since beginning

        // loop every
        float loop_length = 5.0f; // 5s
        double time_diff = AZStd::chrono::duration_cast<AZStd::chrono::microseconds>(time - m_startTime).count() / 1e6;
        m_time = fmod(time_diff, loop_length) / loop_length;


        AZ_PROFILE_FUNCTION(AzRender);
        if (m_drawPacket) {
            for (auto &view: packet.m_views) {
                if (!view->HasDrawListTag(m_drawListTag)) {
                    continue;
                }
                constexpr float depth = 0.f;
                view->AddDrawPacket(m_drawPacket.get(), depth);
            }
        }


    }
    void PointcloudFeatureProcessor::UpdateBackgroundClearColor()
    {
        // This function is only necessary for now because the default clear value
        // color is not black, and is set in various .pass files in places a user
        // is unlikely to find.  Unfortunately, the viewport will revert to the
        // grey color when resizing momentarily.
        const AZ::RHI::ClearValue blackClearValue = AZ::RHI::ClearValue::CreateVector4Float(0.f, 0.f, 0.f, 0.f);
        AZ::RPI::PassFilter passFilter;
        AZStd::string slot;

        auto setClearValue = [&](AZ::RPI::Pass* pass)->AZ:: RPI::PassFilterExecutionFlow
        {
            AZ::Name slotName = AZ::Name::FromStringLiteral(slot, AZ::Interface<AZ::NameDictionary>::Get());
            if (auto binding = pass->FindAttachmentBinding(slotName))
            {
                binding->m_unifiedScopeDesc.m_loadStoreAction.m_clearValue = blackClearValue;
            }
            return AZ::RPI::PassFilterExecutionFlow::ContinueVisitingPasses;
        };

        slot = "SpecularOutput";
        passFilter= AZ::RPI::PassFilter::CreateWithTemplateName(AZ::Name("ForwardPassTemplate"), GetParentScene());
        AZ::RPI::PassSystemInterface::Get()->ForEachPass(passFilter, setClearValue);
        passFilter = AZ::RPI::PassFilter::CreateWithTemplateName(AZ::Name("ForwardMSAAPassTemplate"), GetParentScene());
        AZ::RPI::PassSystemInterface::Get()->ForEachPass(passFilter, setClearValue);

        slot = "ReflectionOutput";
        passFilter =AZ::RPI::PassFilter::CreateWithTemplateName(AZ::Name("ReflectionGlobalFullscreenPassTemplate"), GetParentScene());
        AZ::RPI::PassSystemInterface::Get()->ForEachPass(passFilter, setClearValue);
    }

    AZ::RHI::ConstPtr<AZ::RHI::DrawPacket> PointcloudFeatureProcessor::BuildDrawPacket(
            const AZ::Data::Instance<AZ::RPI::ShaderResourceGroup> &srg,
            const AZ::RPI::Ptr<AZ::RPI::PipelineStateForDraw> &pipelineState,
            const AZ::RHI::DrawListTag &drawListTag,
            const AZStd::span<const AZ::RHI::StreamBufferView> &streamBufferViews,
            uint32_t vertexCount) {
        AZ::RHI::DrawLinear drawLinear;
        drawLinear.m_vertexCount = vertexCount*21;
        // drawLinear.m_vertexOffset = 0;
        // drawLinear.m_instanceCount = 1;
        // drawLinear.m_instanceOffset = 0;

        AZ::RHI::DrawPacketBuilder drawPacketBuilder;
        drawPacketBuilder.Begin(nullptr);
        drawPacketBuilder.SetDrawArguments(drawLinear);
        drawPacketBuilder.AddShaderResourceGroup(srg->GetRHIShaderResourceGroup());

        AZ::Data::AssetData::AssetStatus status = texAsset.BlockUntilLoadComplete();
        if (!m_isTextureValid) {
            if(status == AZ::Data::AssetData::AssetStatus::Ready ) {
                m_isTextureValid = true;
                texture = AZ::RPI::StreamingImage::FindOrCreate(texAsset);
                srg->SetImage(m_inputTextureImageIndex, texture);
            }else {
                AZ_Printf("Blibloard", "Texture not ready\n");
            }
        }

        srg->SetBufferView(m_mBuffery, m_cloudVertexBuffer->GetBufferView());
        AZ::RHI::DrawPacketBuilder::DrawRequest drawRequest;
        drawRequest.m_listTag = drawListTag;
        drawRequest.m_pipelineState = pipelineState->GetRHIPipelineState();
        drawRequest.m_streamBufferViews = streamBufferViews;
        drawPacketBuilder.AddDrawItem(drawRequest);
        return drawPacketBuilder.End();
    }

    void PointcloudFeatureProcessor::OnRenderPipelineChanged([[maybe_unused]] AZ::RPI::RenderPipeline *renderPipeline,
                                                             AZ::RPI::SceneNotification::RenderPipelineChangeType changeType) {
        if (changeType == AZ::RPI::SceneNotification::RenderPipelineChangeType::Added) {
            if (!m_meshPipelineState) {
                m_meshPipelineState = aznew AZ::RPI::PipelineStateForDraw;
                m_meshPipelineState->Init(m_shader);


                // AZ::RHI::InputStreamLayoutBuilder layoutBuilder;
                // layoutBuilder.AddBuffer()
                //         ->Channel("POSITION", AZ::RHI::Format::R32G32B32_FLOAT)
                //         ->Channel("VERTEXID", AZ::RHI::Format::Count);
                // layoutBuilder.SetTopology(AZ::RHI::PrimitiveTopology::TriangleList);
                // auto inputStreamLayout = layoutBuilder.End();

                AZ::RHI::InputStreamLayout inputStreamLayout;
                inputStreamLayout.SetTopology(AZ::RHI::PrimitiveTopology::TriangleList);
                inputStreamLayout.Finalize();
                m_meshPipelineState->SetInputStreamLayout(inputStreamLayout);
                m_meshPipelineState->SetOutputFromScene(GetParentScene());
                m_meshPipelineState->Finalize();

                UpdateDrawPacket();
                UpdateBackgroundClearColor();
            }
        } else if (changeType == AZ::RPI::SceneNotification::RenderPipelineChangeType::PassChanged) {
            if (m_meshPipelineState) {
                m_meshPipelineState->SetOutputFromScene(GetParentScene());
                m_meshPipelineState->Finalize();
                UpdateDrawPacket();
                UpdateBackgroundClearColor();
            }
        }
    }

    void PointcloudFeatureProcessor::UpdateShaderConstants() {
        if (m_drawSrg) {
            AZ_Printf("PointcloudFeatureProcessor", "PointcloudFeatureProcessor SetPointSize");
            m_drawSrg->SetConstant(m_timeIndex, m_time);
            //AZ::Matrix4x4 orientation = AZ::Matrix4x4::CreateFromTransform(m_transform);
//            m_drawSrg->SetConstant(m_modelMatrixIndex, orientation);
//            m_drawSrg->SetConstant(m_pointSizeIndex, m_pointSize);
//            bool m_alwaysFaceCamera;
//            row_major float4x4 m_modelToWorld;
//
//            Buffer<float>  m_positionBuffer;
//            Texture2D<float4> m_inputTexture; // A texture generated by a RenderJoy pipeline.



            m_drawSrg->Compile();
        }
    }

    void PointcloudFeatureProcessor::SetTransform(const AZ::Transform &transform) {
        m_updateShaderConstants = true;
        m_transform = transform;
    }

    void PointcloudFeatureProcessor::SetPointSize(float pointSize) {
        m_updateShaderConstants = true;
        m_pointSize = pointSize;

    }


}
