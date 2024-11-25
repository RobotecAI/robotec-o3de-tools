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

#include "GeoJSONSpawner/GeoJSONSpawnerTypeIds.h"
#include "GeoJSONSpawnerUtils.h"

#include <AzToolsFramework/ToolsComponents/EditorComponentBase.h>
#include <AzFramework/Spawnable/SpawnableEntitiesInterface.h>

namespace GeoJSONSpawner
{

    class GeoJSONSpawnerEditorComponent : public AzToolsFramework::Components::EditorComponentBase
    {
    public:
        AZ_EDITOR_COMPONENT(GeoJSONSpawnerEditorComponent, GeoJSONSpawnerEditorComponentTypeId);

        static void Reflect(AZ::ReflectContext* context);

        void BuildGameEntity(AZ::Entity* gameEntity) override;

    private:
        void SpawnEntities();

        void OnSpawnButton();

        GeoJSONUtils::GeoJSONSpawnerConfiguration m_configuration;
        AZStd::unordered_map<int, AZStd::vector<AzFramework::EntitySpawnTicket>> m_spawnedTicketsGroups;
    };

} // namespace GeoJSONSpawner
