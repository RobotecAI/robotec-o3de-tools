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

namespace Grass {
    class Grass;

    using GrassHandle = AZStd::shared_ptr<Grass>;

    enum class ParameterType {
        Float,
        Float2,
        Float3,
        uint,
        Texture2D,
    };

    AZ_TYPE_INFO_SPECIALIZE(ParameterType, "{82a0b767-e308-4584-a1dc-905e1d7bd29b}");

    struct ShaderParameterUnion {
        AZ_TYPE_INFO(ShaderParameterUnion, "{d09f02ad-0378-4ce9-ba7d-23c7401a83c5}");

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

    // GrassFeatureProcessorInterface provides an interface to the feature processor for code outside of Atom
    class GrassFeatureProcessorInterface
            : public AZ::RPI::FeatureProcessor {
    public:
        AZ_RTTI(GrassFeatureProcessorInterface, "{7f5ba0cc-8df3-4361-8e4f-e35a14f8569a}", AZ::RPI::FeatureProcessor);

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
