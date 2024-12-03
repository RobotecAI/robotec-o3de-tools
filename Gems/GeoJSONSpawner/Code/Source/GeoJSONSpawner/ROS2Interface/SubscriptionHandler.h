/**
 * Copyright (C) Robotec AI - All Rights Reserved
 *
 * This source code is protected under international copyright law.  All rights
 * reserved and protected by the copyright holders.
 * This file is confidential and only available to authorized individuals with the
 * permission of the copyright holders.  If you encounter this file and do not have
 * permission, please contact the copyright holders and delete this file.
 */

#pragma once

#include <AzCore/std/function/function_template.h>

#include <ROS2/RobotControl/ControlSubscriptionHandler.h>

namespace GeoJSONSpawner::ROS2Interface
{

    //! Auxiliary class that wraps subscriber for a ROS2 topic
    template<typename MsgType>
    class SubscriptionHandler : public ROS2::ControlSubscriptionHandler<MsgType>
    {
    public:
        using MessageCallback = AZStd::function<void(const MsgType&)>;

        explicit SubscriptionHandler(MessageCallback callback)
            : m_messageCallback(callback)
        {
        }

    private:
        // ROS2::ControlSubscriptionHandler overrides
        void SendToBus(const MsgType& message) override
        {
            m_messageCallback(message);
        }

        MessageCallback m_messageCallback;
    };
} // namespace GeoJSONSpawner::ROS2Interface