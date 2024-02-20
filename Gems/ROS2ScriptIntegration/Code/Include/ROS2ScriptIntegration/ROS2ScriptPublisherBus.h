/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/EntityId.h>
#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>
#include <AzCore/Math/Quaternion.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/Math/Vector3.h>
#include <AzCore/std/string/string.h>
namespace ROS2ScriptIntegration
{

    //! This bus is used to publish messages to ROS2 topics
    //! It has global scope and can be used by any component
    //! The bus handler is implementing the logic to publish the message to the topic.
    //! The bus handler is responsible for the actual communication with ROS2.
    class PublisherRequests : public AZ::EBusTraits
    {
    public:
        AZ_RTTI(PublisherRequests, "{b8356874-f7ba-4436-8d98-a342d7c720d9}");
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;

        virtual void PublishStdMsgsString(const AZStd::string& topicName, const AZStd::string& value) = 0;
        virtual void PublishStdMsgEmpty(const AZStd::string& topicName) = 0;
        virtual void PublishStdMsgUInt32(const AZStd::string& topicName, const uint32_t value) = 0;
        virtual void PublishStdMsgInt32(const AZStd::string& topicName, const int32_t value) = 0;
        virtual void PublishStdMsgFloat32(const AZStd::string& topicName, const float value) = 0;
        virtual void PublishStdMsgBool(const AZStd::string& topicName, const bool value) = 0;
        virtual void PublishGeometryMsgsTwist(const AZStd::string& topicName, const AZ::Vector3& linear, const AZ::Vector3& angular) = 0;
        virtual void PublishGeometryMsgTransform(const AZStd::string& topicName, const AZ::Transform& transform) = 0;
        virtual void PublishGeometryMsgVector3(const AZStd::string& topicName, const AZ::Vector3& vector) = 0;
        virtual void PublishGeometryMsgQuaternion(const AZStd::string& topicName, const AZ::Quaternion& quaternion) = 0;
        virtual void PublishGeometryMsgPoint32(const AZStd::string& topicName, const AZ::Vector3& point) = 0;
    };

    using PublisherRequestBus = AZ::EBus<PublisherRequests>;
    using PublisherRequesIterface = AZ::Interface<PublisherRequests>;
} // namespace ROS2ScriptIntegration
