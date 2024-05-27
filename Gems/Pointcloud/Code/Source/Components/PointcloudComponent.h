/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <Components/PointcloudComponentController.h>
#include <AzFramework/Components/ComponentAdapter.h>

namespace Pointcloud
{
    inline constexpr AZ::TypeId PointcloudComponentTypeId { "{2DEF80D1-2D5F-4912-80AF-D2001FF0EEE3}" };

    class PointcloudComponent final
        : public AzFramework::Components::ComponentAdapter<PointcloudComponentController, PointcloudComponentConfig>
    {
    public:
        using BaseClass = AzFramework::Components::ComponentAdapter<PointcloudComponentController, PointcloudComponentConfig>;
        AZ_COMPONENT(PointcloudComponent, PointcloudComponentTypeId, BaseClass);

        PointcloudComponent() = default;
        PointcloudComponent(const PointcloudComponentConfig& config);

        static void Reflect(AZ::ReflectContext* context);
    };
}
