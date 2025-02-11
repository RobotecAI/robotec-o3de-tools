#include "TerrainShaperUtils.h"

#include "AzCore/Component/TransformBus.h"
#include "Component/EditorComponentAPIBus.h"
#include "Entity/EditorEntityContextBus.h"
#include "Entity/EditorEntityHelpers.h"
#include "ToolsComponents/TransformComponent.h"

namespace TerrainShaper::Utils
{
    AZStd::vector<AZ::EntityId> GetAllTerrains()
    {
        AZStd::vector<AZ::EntityId> terrains;

        // Broadcast the request to get the editor context ID
        AzFramework::EntityContextId editorContextId;
        AzToolsFramework::EditorEntityContextRequestBus::BroadcastResult(
            editorContextId,
            &AzToolsFramework::EditorEntityContextRequestBus::Events::GetEditorEntityContextId
        );

        AZ_Printf("GetAllTerrains()", "Terrain Context: %u\n", editorContextId);

        // AZ::EntityUtils::EnumerateEntityIds()

        AZ::EntityId rootId = AZ::EntityId(editorContextId.size());

        // Get direct children
        AZStd::vector<AZ::EntityId> children;
        AZ::TransformBus::EventResult(children, rootId, &AZ::TransformBus::Events::GetAllDescendants);

        AZ::Uuid terrainLayerUuid = AZ::Uuid("{3848605F-A4EA-478C-B710-84AB8DCA9EC5}");

        for (const AZ::EntityId& child : children)
        {
            AZ_Printf("GetAllTerrains()", "Child Id: %u\n", child)
            bool hasTerrainComponent = AzToolsFramework::GetEntity(child)->FindComponent(terrainLayerUuid);

            if (!child.IsValid() ||!hasTerrainComponent)
            {
                continue;
            }

            terrains.push_back(child);
        }

        return terrains;
    }
} // TerrainShaper::Utils