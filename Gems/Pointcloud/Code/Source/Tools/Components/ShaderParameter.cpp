#include "ShaderParameter.h"
#include <AzCore/Serialization/EditContext.h>

namespace Pointcloud {
    ShaderParameter::ShaderParameter(ShaderParameterUnion shaderParameterUnion) {
        m_parameterName = shaderParameterUnion.m_parameterName.GetStringView();
        m_parameterType = shaderParameterUnion.m_parameterType;
        switch (m_parameterType) {
            case ParameterType::uint:
                m_uintInput = shaderParameterUnion.m_value.m_uintInput;
                break;
            case ParameterType::Float:
                m_floatInput = shaderParameterUnion.m_value.m_floatInput;
                break;
            case ParameterType::Float2:
                m_float2Input = shaderParameterUnion.m_value.m_float2Input;
                break;
            case ParameterType::Float3:
                m_float3Input = shaderParameterUnion.m_value.m_float3Input;
                break;
        }
    }

    void ShaderParameter::Reflect(AZ::ReflectContext *context) {
        AZ::SerializeContext *serializeContext = azrtti_cast<AZ::SerializeContext *>(context);
        if (serializeContext) {
            serializeContext->Class<ShaderParameter>()
                    ->Version(2)
                    ->Field("Parameter Name", &ShaderParameter::m_parameterName)
                    ->Field("Constant Type", &ShaderParameter::m_parameterType)
                    ->Field("Uint Input", &ShaderParameter::m_uintInput)
                    ->Field("Float Input", &ShaderParameter::m_floatInput)
                    ->Field("Float2 Input", &ShaderParameter::m_float2Input)
                    ->Field("Float3 Input", &ShaderParameter::m_float3Input);


            AZ::EditContext *editContext = serializeContext->GetEditContext();
            if (editContext) {
                editContext->Class<ShaderParameter>("PointcloudEditorComponent", "PointcloudEditorComponent")
                        ->ClassElement(AZ::Edit::ClassElements::EditorData, "PointcloudEditorComponent")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                        ->DataElement(AZ::Edit::UIHandlers::Default, &ShaderParameter::m_parameterName,
                                      "Parameter Name",
                                      "Parameter Name")
                        ->Attribute(AZ::Edit::Attributes::ReadOnly, true)
                        ->DataElement(AZ::Edit::UIHandlers::ComboBox, &ShaderParameter::m_parameterType,
                                      "Constant Type",
                                      "Constant Type")
                        ->Attribute(AZ::Edit::Attributes::ReadOnly, true)
                        ->Attribute(AZ::Edit::Attributes::ChangeNotify, &ShaderParameter::OnTypeChanged)
                        ->DataElement(AZ::Edit::UIHandlers::Default, &ShaderParameter::m_uintInput, "Uint Input",
                                      "Uint Input")
                        ->Attribute(AZ::Edit::Attributes::Visibility, &ShaderParameter::IsUintInputVisible)
                        ->DataElement(AZ::Edit::UIHandlers::Default, &ShaderParameter::m_floatInput, "Float Input",
                                      "Float Input")
                        ->Attribute(AZ::Edit::Attributes::Visibility, &ShaderParameter::IsFloatInputVisible)
                        ->DataElement(AZ::Edit::UIHandlers::Default, &ShaderParameter::m_float2Input, "Float2 Input",
                                      "Float2 Input")
                        ->Attribute(AZ::Edit::Attributes::Visibility, &ShaderParameter::IsFloat2InputVisible)
                        ->DataElement(AZ::Edit::UIHandlers::Default, &ShaderParameter::m_float3Input, "Float3 Input",
                                      "Float3 Input")
                        ->Attribute(AZ::Edit::Attributes::Visibility, &ShaderParameter::IsFloat3InputVisible);
            }
        }
    }

    ShaderParameterUnion ShaderParameter::ToShaderParameterUnion() const {
        ShaderParameterUnion::ParameterValue value;
        switch (m_parameterType) {
            case ParameterType::uint:
                value.m_uintInput = m_uintInput;
                break;
            case ParameterType::Float:
                value.m_floatInput = m_floatInput;
                break;
            case ParameterType::Float2:
                value.m_float2Input = m_float2Input;
                break;
            case ParameterType::Float3:
                value.m_float3Input = m_float3Input;
                break;
        }
        return {
            AZ::Name(m_parameterName),
            m_parameterType,
            value
        };
    }


    bool ShaderParameter::IsUintInputVisible() const {
        return m_parameterType == ParameterType::uint;
    }

    bool ShaderParameter::IsFloatInputVisible() const {
        return m_parameterType == ParameterType::Float;
    }

    bool ShaderParameter::IsFloat2InputVisible() const {
        return m_parameterType == ParameterType::Float2;
    }

    bool ShaderParameter::IsFloat3InputVisible() const {
        return m_parameterType == ParameterType::Float3;
    }

    AZ::Crc32 ShaderParameter::OnTypeChanged() {
        return AZ::Edit::PropertyRefreshLevels::EntireTree;
    }

}
