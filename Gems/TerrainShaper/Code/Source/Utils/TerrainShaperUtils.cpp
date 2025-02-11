#include "TerrainShaperUtils.h"

#include <AzCore/Component/TransformBus.h>
#include <Component/EditorComponentAPIBus.h>
#include <Entity/EditorEntityContextBus.h>
#include <Entity/EditorEntityHelpers.h>
#include <ToolsComponents/TransformComponent.h>

namespace TerrainShaper::Utils
{
    AZStd::vector<AZ::EntityId> GetAllTerrains()
    {
        AZStd::vector<AZ::EntityId> terrains; // Available Terrains to return
        AZ::Uuid terrainLayerUuid = AZ::Uuid("{9403FC94-FA38-4387-BEFD-A728C7D850C1}"); // Direct Uuid from EditorTerrainLayerSpawner

        // Broadcast the request to get the editor context ID
        AzFramework::EntityContextId editorContextId;
        AzToolsFramework::EditorEntityContextRequestBus::BroadcastResult(
            editorContextId,
            &AzToolsFramework::EditorEntityContextRequestBus::Events::GetEditorEntityContextId
        );
        AZ_Assert(editorContextId.IsNull(), "Could not retrieve the editor context id.");

        // Get selected entities in editor
        AzToolsFramework::EntityIdList selectedEntities;
        AzToolsFramework::ToolsApplicationRequestBus::BroadcastResult(
            selectedEntities, &AzToolsFramework::ToolsApplicationRequests::GetSelectedEntities
        );
        AZ_Assert(selectedEntities.empty(), "No entity selected! Please select at least one entity from Entity Outliner.");

        // Get Selected Entities Descendants
        AZStd::vector<AZ::EntityId> children;
        for (const AZ::EntityId& childId : selectedEntities)
        {
            // Get direct children
            AZ::TransformBus::EventResult(children, childId, &AZ::TransformBus::Events::GetAllDescendants);
        }

        // Lookup for children with terrain layer component
        for (const AZ::EntityId& childId : children)
        {
            if (!childId.IsValid())
            {
                continue;
            }

            auto hasTerrainComponent = AzToolsFramework::GetEntity(childId)->FindComponent(terrainLayerUuid);

            if (!hasTerrainComponent)
            {
                continue;
            }

            AZ_Printf("GetAllTerrains()", "Has Terrain: %u\n", childId)
            terrains.push_back(childId);
        }

        return terrains;
    }
} // TerrainShaper::Utils