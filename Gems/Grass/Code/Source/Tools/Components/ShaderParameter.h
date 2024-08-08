#pragma once

#include <AzCore/Asset/AssetCommon.h>
#include <AzCore/Component/TickBus.h>
#include <AzFramework/Entity/EntityContextBus.h>
#include <AzFramework/Entity/EntityDebugDisplayBus.h>
#include <AzFramework/Spawnable/SpawnableEntitiesInterface.h>
#include <AzCore/Component/TransformBus.h>
#include <Grass/GrassFeatureProcessorInterface.h>
#include <AzFramework/Scene/Scene.h>
namespace Grass
{

    class ShaderParameter
    {
    public:
        AZ_TYPE_INFO(ShaderParameter, "{018fba15-560f-78cb-afb4-cf4d00cefc22}");
        ShaderParameter() = default;
        ShaderParameter(ShaderParameterUnion shaderParameterUnion);
        ~ShaderParameter() = default;

        static void Reflect(AZ::ReflectContext* context);

        ShaderParameterUnion ToShaderParameterUnion() const;
        AZ::Crc32 OnTypeChanged();
        
        AZStd::string m_parameterName;

        ParameterType m_parameterType;

        uint32_t m_uintInput;
        float m_floatInput;
        AZ::Vector2 m_float2Input;
        AZ::Vector3 m_float3Input;
    private:
        bool IsUintInputVisible() const;
        bool IsFloatInputVisible() const;
        bool IsFloat2InputVisible() const;
        bool IsFloat3InputVisible() const;





    };
} // namespace Grass
