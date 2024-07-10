/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/base.h>
#include <Atom/RPI.Public/FeatureProcessor.h>
#include <AzCore/Component/Component.h>
#include <AzCore/Math/Transform.h>

namespace Pointcloud {
    class Pointcloud;

    using PointcloudHandle = AZStd::shared_ptr<Pointcloud>;

    enum class ParameterType {
        Float,
        Float2,
        Float3,
        uint,
        Texture2D,
    };

    AZ_TYPE_INFO_SPECIALIZE(ParameterType, "{7ebef8a5-b40d-4a9a-8511-162da1dc0211}");


    struct ShaderParameterUnion {
        AZ_TYPE_INFO(ShaderParameterUnion, "{018fba15-560f-78cb-afb4-cf4d00cefc33}");

        AZ::Name m_parameterName;
        ParameterType m_parameterType;

        union ParameterValue {
            uint32_t m_uintInput;
            float m_floatInput;
            AZ::Vector2 m_float2Input;
            AZ::Vector3 m_float3Input;
        } m_value;

        ShaderParameterUnion() = default;
        ShaderParameterUnion(AZ::Name name, ParameterType type, ParameterValue value) : m_parameterName(name), m_parameterType(type), m_value(value) {}
        ShaderParameterUnion(AZ::Name name, ParameterType type) : m_parameterName(name), m_parameterType(type) {
            switch (type) {
                case ParameterType::Float:
                    m_value.m_floatInput = 0.0f;
                    break;
                case ParameterType::Float2:
                    m_value.m_float2Input = AZ::Vector2(0.0f, 0.0f);
                    break;
                case ParameterType::Float3:
                    m_value.m_float3Input = AZ::Vector3(0.0f, 0.0f, 0.0f);
                    break;
                case ParameterType::uint:
                    m_value.m_uintInput = 0;
                    break;
            }
        }
    };

    //using ShaderParameter = AZStd::tuple<AZ::Name,ParameterType,AZ::Vector3>;

    // PointcloudFeatureProcessorInterface provides an interface to the feature processor for code outside of Atom
    class PointcloudFeatureProcessorInterface
            : public AZ::RPI::FeatureProcessor {
    public:
        AZ_RTTI(PointcloudFeatureProcessorInterface, "{8597AF27-EB4E-4363-8889-3BFC2AF5D2EC}",
                AZ::RPI::FeatureProcessor);

        struct CloudVertex {
            AZStd::array<float, 3> m_position;
            //AZStd::array<float, 3> m_normal;
            uint32_t m_color;
        };

        // virtual void SetTransform(const AZ::Transform &transform)= 0;
        // virtual void SetPointSize(float pointSize)= 0;
        virtual void SetParameters(const AZStd::vector<ShaderParameterUnion> &shaderParameters) = 0;

        virtual AZStd::vector<ShaderParameterUnion> GetParameters() = 0;

        virtual void ForceUpdate(uint32_t totalVertices) = 0;
    };
}
