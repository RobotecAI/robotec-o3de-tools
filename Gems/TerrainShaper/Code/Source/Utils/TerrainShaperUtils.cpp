#include "TerrainShaperUtils.h"

#include <Entity/EditorEntityContextBus.h>

namespace TerrainShaper::Utils
{
    AZStd::vector<AZ::EntityId> GetAllTerrains()
    {
        AZStd::vector<AZ::EntityId> terrains;

        AzToolsFramework::EditorEntityContextRequestBus::BroadcastResult(
            terrains, &AzToolsFramework::EditorEntityContextRequests::,
            azrtti_typeid<Terrain::TerrainLayerSpawnerComponent>()
        );

        return terrains;
    }
} // TerrainShaper::Utils