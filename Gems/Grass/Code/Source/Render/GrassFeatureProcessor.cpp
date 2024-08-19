/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "GrassFeatureProcessor.h"
#include <Atom/RHI.Reflect/InputStreamLayoutBuilder.h>
#include <Atom/RHI/DrawPacketBuilder.h>
#include <Atom/RPI.Public/Pass/PassFilter.h>
#include <Atom/RPI.Public/RPIUtils.h>
#include <Atom/RPI.Public/ViewportContext.h>
#include <Atom/RPI.Public/ViewportContextBus.h>
#include <Atom/RPI.Reflect/Asset/AssetUtils.h>
#include <Atom/RPI.Reflect/Shader/ShaderAsset.h>
#include <AzCore/Name/NameDictionary.h>
#include <fstream>
namespace Grass
{
    void GrassFeatureProcessor::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<GrassFeatureProcessor, FeatureProcessor>();
        }
    }

    void GrassFeatureProcessor::Activate()
    {
        AZ_Printf("GrassFeatureProcessor", "GrassFeatureProcessor Activated");
        m_startTime = AZStd::chrono::system_clock::now();

        LoadShader("shaders/grassdepth.azshader");
        LoadShader("shaders/grassforward.azshader");
        LoadShader("shaders/grassshadow.azshader");

        auto viewportContextInterface = AZ::Interface<AZ::RPI::ViewportContextRequestsInterface>::Get();
        AZ_Assert(viewportContextInterface, "GrassFeatureProcessor requires the ViewportContextRequestsInterface.");
        auto viewportContext = viewportContextInterface->GetViewportContextByScene(GetParentScene());
        AZ_Assert(viewportContext, "GrassFeatureProcessor requires a valid ViewportContext.");
        m_viewportSize = viewportContext->GetViewportSize();

        EnableSceneNotification();

        AZ::RPI::ViewportContextIdNotificationBus::Handler::BusConnect(viewportContext->GetId());
        // texAsset = AZ::RPI::AssetUtils::GetAssetByProductPath<AZ::RPI::StreamingImageAsset>("assets/untitled1.png.streamingimage");
        // texAsset.QueueLoad();
    }

    // const char *shaderFilePath = "shaders/billboard2.azshader";

    void GrassFeatureProcessor::LoadShader(AZStd::string shaderFilePath)
    {
        AZ_Printf("GrassFeatureProcessor", "Loading Shader");
        ShaderInstance shaderInstance;
        if (shaderInstance.m_shader)
        {
            AZ::Data::AssetBus::MultiHandler::BusDisconnect(shaderInstance.m_shader->GetAssetId());
        }
        shaderInstance.m_shader = AZ::RPI::LoadCriticalShader(shaderFilePath);
        if (!shaderInstance.m_shader)
        {
            printf("Failed to load required stars shader.\n");
            AZ_Error("GrassFeatureProcessor", false, "Failed to load required stars shader.");
            return;
        }
        AZ::Data::AssetBus::MultiHandler::BusConnect(shaderInstance.m_shader->GetAssetId());
        AZ::RPI::ShaderOptionGroup shaderOptions = shaderInstance.m_shader->CreateShaderOptionGroup();
        m_drawListTag = shaderInstance.m_shader->GetDrawListTag();

        if (true)
        {
            static const AZ::Name valueTrue = AZ::Name("true");
            static const AZ::Name valueFalse = AZ::Name("false");
            shaderOptions.SetValue(AZ::Name("o_enablePunctualLights"), valueTrue);
            shaderOptions.SetValue(AZ::Name("o_enableAreaLights"), valueTrue);
            shaderOptions.SetValue(AZ::Name("o_enableDirectionalLights"), valueTrue);
            shaderOptions.SetValue(AZ::Name("o_enableIBL"), valueTrue);
            shaderOptions.SetValue(AZ::Name("o_enableShadows"), valueTrue);
        }

        auto shaderVariantId = shaderOptions.GetShaderVariantId();
        printf(" shaderInstance.m_shader->GetAsset() %s Lol", shaderInstance.m_shader->GetAsset()->GetName().GetCStr());
        auto drawSrgLayout = shaderInstance.m_shader->GetAsset()->FindShaderResourceGroupLayout(AZ::Name("PerDrawSrg"));
        auto objectSrgLayout = shaderInstance.m_shader->GetAsset()->FindShaderResourceGroupLayout(AZ::Name("ObjectSrg"));
        AZ_Error("GrassFeatureProcessor", drawSrgLayout, "Failed to get the draw shader resource group layout for the grass shader.");
        auto shader_variant = shaderInstance.m_shader->GetVariant(shaderVariantId);

        if (drawSrgLayout && objectSrgLayout)
        {
            shaderInstance.m_drawSrg = shaderInstance.m_shader->CreateDrawSrgForShaderVariant(shaderOptions, false);
            shaderInstance.m_objectSrg = AZ::RPI::ShaderResourceGroup::Create(
                shaderInstance.m_shader->GetAsset(), shaderInstance.m_shader->GetSupervariantIndex(), objectSrgLayout->GetName());
            m_modelMatrixIndex.Reset();
            printf("Created SRG\n");
            auto input = objectSrgLayout->GetConstantsLayout();
            auto inputs = input->GetShaderInputList();
            m_shaderParameters.clear();
            for (auto input : inputs)
            {
                printf(
                    "Name: %s, ByteOffset: %d, ByteCount: %d, RegisterId: %d, SpaceId: %d\n",
                    input.m_name.GetCStr(),
                    input.m_constantByteOffset,
                    input.m_constantByteCount,
                    input.m_registerId,
                    input.m_spaceId);
                auto extractOutcome = ExtractParameterType(input.m_name);
                if (extractOutcome.IsSuccess())
                {
                    m_shaderParameters.push_back(ShaderParameterUnion(input.m_name, extractOutcome.GetValue()));
                }
                else
                {
                    AZ_Error("GrassFeatureProcessor", false, extractOutcome.GetError().c_str());
                }
            }
        }
        else
        {
            printf("Failed to create SRG\n");
        }

        auto tag = shaderInstance.m_shader->GetDrawListTag();
        printf("DrawListTag %d\n", tag.GetIndex());
        m_shaderInstances[shaderInstance.m_shader->GetDrawListTag()] = shaderInstance;
    }

    AZStd::vector<float> GrassFeatureProcessor::ConvertToBuffer(const AZStd::vector<CloudVertex>& cloudVertexData)
    {
        AZStd::vector<float> random_data(4 * 4 * cloudVertexData.size());
        auto m_worldMatrix = AZ::Matrix4x4::CreateFromTransform(m_transform);
        AZ::Matrix3x4 m_worldMatrix3x4;
        m_worldMatrix3x4.SetRows(m_worldMatrix.GetRow(0), m_worldMatrix.GetRow(1), m_worldMatrix.GetRow(2));
        for (int k = 0; k < cloudVertexData.size(); k++)
        {
            AZ::Transform transform = AZ::Transform::CreateFromMatrix3x4(m_worldMatrix3x4);
            AZ::Vector3 position = { cloudVertexData[k].m_position[0], cloudVertexData[k].m_position[1], cloudVertexData[k].m_position[2] };
            transform.SetTranslation(position);
            AZ::Matrix4x4 tmp_mat = AZ::Matrix4x4::CreateFromTransform(transform);
            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    random_data[16 * k + i * 4 + j] = tmp_mat.GetElement(i, j);
                }
            }
        }
        return random_data;
    }

    void GrassFeatureProcessor::ForceUpdate(uint32_t totalVertices)
    {
        LoadShader("shaders/grassdepth.azshader");
        LoadShader("shaders/grassforward.azshader");
        LoadShader("shaders/grassshadow.azshader");
        m_totalVertices = totalVertices;
        UpdateDrawPacket();
        UpdateBackgroundClearColor();
        m_updateShaderConstants = true;
    }

    AZStd::vector<ShaderParameterUnion> GrassFeatureProcessor::GetParameters()
    {
        return m_shaderParameters;
    }

    AZ::Outcome<ParameterType, AZStd::string> GrassFeatureProcessor::ExtractParameterType(const AZ::Name& parameterName)
    {
        // all look for u_ , f_ , f2_ , f3_
        // but frist there will be m_
        if (parameterName.GetStringView().starts_with("m_"))
        {
            if (parameterName.GetStringView().starts_with("m_f3_"))
            {
                return ParameterType::Float3;
            }
            else if (parameterName.GetStringView().starts_with("m_f2_"))
            {
                return ParameterType::Float2;
            }
            else if (parameterName.GetStringView().starts_with("m_f_"))
            {
                return ParameterType::Float;
            }
            else if (parameterName.GetStringView().starts_with("m_u_"))
            {
                return ParameterType::uint;
            }
            else if (parameterName.GetStringView().starts_with("m_t2_"))
            {
                return ParameterType::Texture2D;
                AZStd::string error = "Parameter name '" + AZStd::string(parameterName.GetCStr()) +
                    "' does not start with any of m_f_, m_f2_, m_f3_, or m_u_";
                return AZ::Failure(error);
            }
        }
        AZStd::string error = "Parameter name '" + AZStd::string(parameterName.GetCStr()) + "' does not start with m_";
        return AZ::Failure(error);
    }

    void GrassFeatureProcessor::UpdateDrawPacket()
    {
        // print if  things
        printf("GrassFeatureProcessor UpdateDrawPacket\n");
        // variables

        for (auto& [drawListTag, shaderInstance] : m_shaderInstances)
        {
            printf("m_meshPipelineState %s\n", shaderInstance.m_meshPipelineState ? "true" : "false");
            printf("shaderInstance.m_shader %s\n", shaderInstance.m_shader ? "true" : "false");
            printf("shaderInstance.m_drawSrg %s\n", shaderInstance.m_drawSrg ? "true" : "false");
            printf("shaderInstance.m_objectSrg %s\n", shaderInstance.m_objectSrg ? "true" : "false");
            if (!shaderInstance.m_shader || !shaderInstance.m_drawSrg || !shaderInstance.m_objectSrg || !shaderInstance.m_meshPipelineState)
            {
                printf("ShaderInstance not ready\n");
                return;
            }
        }

        printf("GrassFeatureProcessor UpdateDrawPacket\n");
        m_drawPacket = BuildDrawPacket(m_shaderInstances, m_meshStreamBufferViews, m_totalVertices);
    }

    void GrassFeatureProcessor::OnAssetReloaded(AZ::Data::Asset<AZ::Data::AssetData> asset)
    {
        AZ_Printf("GrassFeatureProcessor", "GrassFeatureProcessor OnAssetReloaded");
        UpdateDrawPacket();
        UpdateBackgroundClearColor();
    }

    void GrassFeatureProcessor::Deactivate()
    {
        AZ_Printf("GrassFeatureProcessor", "GrassFeatureProcessor Deactivated");
        for (auto& [drawListTag, shaderInstance] : m_shaderInstances)
        {
            AZ::Data::AssetBus::MultiHandler::BusDisconnect(shaderInstance.m_shader->GetAssetId());
        }
        AZ::RPI::ViewportContextIdNotificationBus::Handler::BusDisconnect();
    }

    void GrassFeatureProcessor::Simulate([[maybe_unused]] const FeatureProcessor::SimulatePacket& packet)
    {
        UpdateShaderConstants();
        //        if (m_updateShaderConstants) {
        //            m_updateShaderConstants = false;
        //            UpdateShaderConstants();
        //        }
    }

    void GrassFeatureProcessor::Render([[maybe_unused]] const FeatureProcessor::RenderPacket& packet)
    {
        auto time = AZStd::chrono::system_clock::now();

        // loop every
        float loop_length = 30.0f; // 5s
        double time_diff = AZStd::chrono::duration_cast<AZStd::chrono::microseconds>(time - m_startTime).count() / 1e6;
        double phase = fmod(time_diff, loop_length) / loop_length; // phase goes from 0 to 1

        // m_time goes from 0 to 1 to 0
        if (phase < 0.5)
        {
            m_time = 2.0 * phase; // first half of the cycle, m_time goes from 0 to 1
        }
        else
        {
            m_time = 2.0 * (1.0 - phase); // second half of the cycle, m_time goes from 1 to 0
        }

        AZ_PROFILE_FUNCTION(AzRender);
        if (m_drawPacket)
        {
            for (auto& view : packet.m_views)
            {
                if (!view->HasDrawListTag(m_drawListTag))
                {
                    continue;
                }
                constexpr float depth = 0.f;
                view->AddDrawPacket(m_drawPacket.get(), depth);
            }
        }
    }

    void GrassFeatureProcessor::UpdateBackgroundClearColor()
    {
        // This function is only necessary for now because the default clear value
        // color is not black, and is set in various .pass files in places a user
        // is unlikely to find.  Unfortunately, the viewport will revert to the
        // grey color when resizing momentarily.
        const AZ::RHI::ClearValue blackClearValue = AZ::RHI::ClearValue::CreateVector4Float(0.f, 0.f, 0.f, 0.f);
        AZ::RPI::PassFilter passFilter;
        AZStd::string slot;

        auto setClearValue = [&](AZ::RPI::Pass* pass) -> AZ::RPI::PassFilterExecutionFlow
        {
            AZ::Name slotName = AZ::Name::FromStringLiteral(slot, AZ::Interface<AZ::NameDictionary>::Get());
            if (auto binding = pass->FindAttachmentBinding(slotName))
            {
                binding->m_unifiedScopeDesc.m_loadStoreAction.m_clearValue = blackClearValue;
            }
            return AZ::RPI::PassFilterExecutionFlow::ContinueVisitingPasses;
        };

        slot = "SpecularOutput";
        passFilter = AZ::RPI::PassFilter::CreateWithTemplateName(AZ::Name("ForwardPassTemplate"), GetParentScene());
        AZ::RPI::PassSystemInterface::Get()->ForEachPass(passFilter, setClearValue);
        passFilter = AZ::RPI::PassFilter::CreateWithTemplateName(AZ::Name("ForwardMSAAPassTemplate"), GetParentScene());
        AZ::RPI::PassSystemInterface::Get()->ForEachPass(passFilter, setClearValue);

        slot = "ReflectionOutput";
        passFilter = AZ::RPI::PassFilter::CreateWithTemplateName(AZ::Name("ReflectionGlobalFullscreenPassTemplate"), GetParentScene());
        AZ::RPI::PassSystemInterface::Get()->ForEachPass(passFilter, setClearValue);
    }

    // AZ::RHI::ConstPtr<AZ::RHI::DrawPacket> GrassFeatureProcessor::BuildDrawPacket(
    //     const AZ::Data::Instance<AZ::RPI::ShaderResourceGroup>& drawSrg,
    //     const AZ::Data::Instance<AZ::RPI::ShaderResourceGroup>& objectSrg,
    //     const AZ::RPI::Ptr<AZ::RPI::PipelineStateForDraw>& pipelineState,
    //     const AZ::RHI::DrawListTag& drawListTag,
    //     const AZStd::span<const AZ::RHI::StreamBufferView>& streamBufferViews,
    //     uint32_t vertexCount)
    // {
    //     AZ::RHI::DrawLinear drawLinear;
    //     drawLinear.m_vertexCount = vertexCount;
    //     printf("Vertex Count %d\n", vertexCount);
    //     AZ::RHI::DrawPacketBuilder drawPacketBuilder;
    //     drawPacketBuilder.Begin(nullptr);
    //     drawPacketBuilder.SetDrawArguments(drawLinear);
    //     drawPacketBuilder.AddShaderResourceGroup(drawSrg->GetRHIShaderResourceGroup());
    //     drawPacketBuilder.AddShaderResourceGroup(objectSrg->GetRHIShaderResourceGroup());
    //
    //     // AZ::Data::AssetData::AssetStatus status = texAsset.BlockUntilLoadComplete();
    //     // if (!m_isTextureValid) {
    //     //     if(status == AZ::Data::AssetData::AssetStatus::Ready ) {
    //     //         m_isTextureValid = true;
    //     //         texture = AZ::RPI::StreamingImage::FindOrCreate(texAsset);
    //     //         objectSrg->SetImage(m_inputTextureImageIndex, texture);
    //     //     }else {
    //     //         AZ_Printf("Blibloard", "Texture not ready\n");
    //     //     }
    //     // }
    //
    //     AZ::RHI::DrawPacketBuilder::DrawRequest drawRequest;
    //     drawRequest.m_listTag = drawListTag;
    //     drawRequest.m_pipelineState = pipelineState->GetRHIPipelineState();
    //     drawRequest.m_streamBufferViews = streamBufferViews;
    //     drawPacketBuilder.AddDrawItem(drawRequest);
    //     return drawPacketBuilder.End();
    // }

    AZ::RHI::ConstPtr<AZ::RHI::DrawPacket> GrassFeatureProcessor::BuildDrawPacket(
        AZStd::unordered_map<AZ::RHI::DrawListTag, ShaderInstance> shaderInstances,
        const AZStd::span<const AZ::RHI::StreamBufferView>& streamBufferViews,
        uint32_t vertexCount)
    {
        AZ::RHI::DrawLinear drawLinear;
        drawLinear.m_vertexCount = vertexCount;
        printf("Vertex Count %d\n", vertexCount);
        AZ::RHI::DrawPacketBuilder drawPacketBuilder;
        drawPacketBuilder.Begin(nullptr);
        drawPacketBuilder.SetDrawArguments(drawLinear);
        // drawPacketBuilder.AddShaderResourceGroup(shaderInstances[].m_drawSrg->GetRHIShaderResourceGroup());
        // drawPacketBuilder.AddShaderResourceGroup(shaderInstances[drawListTag].m_drawSrg->GetRHIShaderResourceGroup());
        for (auto& [drawListTag, shaderInstance] : shaderInstances)
        {
            drawPacketBuilder.AddShaderResourceGroup(shaderInstances[drawListTag].m_objectSrg->GetRHIShaderResourceGroup());
            // drawPacketBuilder.AddShaderResourceGroup(shaderInstances[drawListTag].m_drawSrg->GetRHIShaderResourceGroup());
        }
        // AZ::Data::AssetData::AssetStatus status = texAsset.BlockUntilLoadComplete();
        // if (!m_isTextureValid) {
        //     if(status == AZ::Data::AssetData::AssetStatus::Ready ) {
        //         m_isTextureValid = true;
        //         texture = AZ::RPI::StreamingImage::FindOrCreate(texAsset);
        //         objectSrg->SetImage(m_inputTextureImageIndex, texture);
        //     }else {
        //         AZ_Printf("Blibloard", "Texture not ready\n");
        //     }
        // }

        for (auto& [drawListTag, shaderInstance] : shaderInstances)
        {
            AZ::RHI::DrawPacketBuilder::DrawRequest drawRequest;
            drawRequest.m_listTag = drawListTag;
            drawRequest.m_uniqueShaderResourceGroup = shaderInstance.m_drawSrg->GetRHIShaderResourceGroup();
            drawRequest.m_pipelineState = shaderInstance.m_meshPipelineState->GetRHIPipelineState();
            drawRequest.m_streamBufferViews = streamBufferViews;
            drawPacketBuilder.AddDrawItem(drawRequest);
        }
        return drawPacketBuilder.End();
    }

    void GrassFeatureProcessor::OnRenderPipelineChanged(
        [[maybe_unused]] AZ::RPI::RenderPipeline* renderPipeline, AZ::RPI::SceneNotification::RenderPipelineChangeType changeType)
    {
        for (auto& [drawListTag, shaderInstance] : m_shaderInstances)
        {
            auto& meshPipelineState = shaderInstance.m_meshPipelineState;
            auto& shader = shaderInstance.m_shader;

            if (changeType == AZ::RPI::SceneNotification::RenderPipelineChangeType::Added)
            {
                if (!meshPipelineState)
                {
                    meshPipelineState = aznew AZ::RPI::PipelineStateForDraw;
                    meshPipelineState->Init(shader);

                    // AZ::RHI::InputStreamLayoutBuilder layoutBuilder;
                    // layoutBuilder.AddBuffer()
                    //         ->Channel("POSITION", AZ::RHI::Format::R32G32B32_FLOAT)
                    //         ->Channel("VERTEXID", AZ::RHI::Format::Count);
                    // layoutBuilder.SetTopology(AZ::RHI::PrimitiveTopology::TriangleList);
                    // auto inputStreamLayout = layoutBuilder.End();

                    AZ::RHI::InputStreamLayout inputStreamLayout;
                    inputStreamLayout.SetTopology(AZ::RHI::PrimitiveTopology::TriangleList);
                    inputStreamLayout.Finalize();
                    meshPipelineState->SetInputStreamLayout(inputStreamLayout);
                    meshPipelineState->SetOutputFromScene(GetParentScene());
                    meshPipelineState->Finalize();
                }
            }
            else if (changeType == AZ::RPI::SceneNotification::RenderPipelineChangeType::PassChanged)
            {
                if (meshPipelineState)
                {
                    meshPipelineState->SetOutputFromScene(GetParentScene());
                    meshPipelineState->Finalize();
                }
            }
        }
        if (changeType == AZ::RPI::SceneNotification::RenderPipelineChangeType::PassChanged ||
            changeType == AZ::RPI::SceneNotification::RenderPipelineChangeType::Added)
        {
            UpdateDrawPacket();
            UpdateBackgroundClearColor();
        }
    }

    void GrassFeatureProcessor::UpdateShaderConstants()
    {
        for (auto& [drawListTag, shaderInstance] : m_shaderInstances)
        {
            auto& m_drawSrg = shaderInstance.m_drawSrg;
            auto& m_objectSrg = shaderInstance.m_objectSrg;

            if (m_drawSrg)
            {
                m_drawSrg->SetConstant(m_timeIndex, m_time);
                m_drawSrg->Compile();
            }

            if (m_objectSrg)
            {
                for (const auto& param : m_shaderParameters)
                {
                    AZ::RHI::ShaderInputNameIndex index(param.m_parameterName);
                    switch (param.m_parameterType)
                    {
                    case ParameterType::uint:
                        m_objectSrg->SetConstant(index, param.m_value.m_uintInput);
                        break;
                    case ParameterType::Float:
                        m_objectSrg->SetConstant(index, param.m_value.m_floatInput);
                        break;
                    case ParameterType::Float2:
                        m_objectSrg->SetConstant(index, param.m_value.m_float2Input);
                        break;
                    case ParameterType::Float3:
                        m_objectSrg->SetConstant(index, param.m_value.m_float3Input);
                        break;
                    }
                }
                m_objectSrg->Compile();
            }
        }
    }

    void GrassFeatureProcessor::SetParameters(const AZStd::vector<ShaderParameterUnion>& shaderParameters)
    {
        for (const auto& newParam : shaderParameters)
        {
            for (auto& currentParam : m_shaderParameters)
            {
                if (newParam.m_parameterName == currentParam.m_parameterName)
                {
                    currentParam.m_value = newParam.m_value;
                    break;
                }
            }
            if (newParam.m_parameterName == AZ::Name("m_u_vertexCountPerMesh"))
            {
                m_vertexCountPerMesh = newParam.m_value.m_uintInput;
            }
        }

        m_updateShaderConstants = true;
    }

} // namespace Grass
