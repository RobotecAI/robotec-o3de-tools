/* Copyright 2025, Robotec.ai sp. z o.o.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <Atom/RPI.Public/AuxGeom/AuxGeomDraw.h>
#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <Eigen/Dense>
namespace WheelAnimTool
{
    enum class AnimationType
    {
        Mecanum,
        Differential,
    };
    class WheelAnimComponent
        : public AZ::Component
        , public AZ::TickBus::Handler
    {
    public:
        AZ_COMPONENT(WheelAnimComponent, "{12345678-1234-5678-9012-123456789012}");

        static void Reflect(AZ::ReflectContext* context);

        WheelAnimComponent() = default;

        ~WheelAnimComponent() = default;

        void Activate() override;

        void Deactivate() override;

        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

    private:
        AZ::Crc32 RollerDirectionVisible() const; //< Get visibility of roller direction property based on animation type
        AZStd::vector<AZ::EntityId> m_wheelEntities; //< Wheel entities to animate
        AZStd::vector<AZ::Vector2> m_rollerDirections; //< Directions of rollers for mecanum wheels
        AnimationType m_animationType = AnimationType::Mecanum; // Type of wheel animation
        AZ::Vector3 m_wheelAxis = AZ::Vector3::CreateAxisY(); // Axis of rotation for the wheels
        bool m_debugDraw = false; // Enable debug drawing
        float m_wheelRadius = 0.1f; // Radius of the wheel for drawing purposes

        bool InitJacobian(); //< Initialize the Jacobian matrix based on wheel directions and animation type
        Eigen::MatrixXd m_jacobian; // Jacobian matrix of size (numWheels, 3) for 3D to 1D speed transform
        AZ::RPI::AuxGeomDrawPtr m_drawQueue;
    };
} // namespace WheelAnimTool
