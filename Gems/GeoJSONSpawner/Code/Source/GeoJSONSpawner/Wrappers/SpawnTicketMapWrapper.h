#pragma once

#include "GeoJSONSpawner/GeoJSONSpawnerTypeIds.h"

#include <AzCore/RTTI/BehaviorContext.h>
#include <AzCore/std/containers/unordered_map.h>
#include <AzCore/std/containers/vector.h>
#include <AzFramework/Spawnable/SpawnableEntitiesInterface.h>

namespace GeoJSONSpawner::GeoJSONWrappers
{
    class SpawnTicketMapWrapper
    {
    public:
        AZ_TYPE_INFO(SpawnTicketMapWrapper, GeoJSONSpawnerSpawnTicketMapWrapperTypeId);
        AZ_CLASS_ALLOCATOR(SpawnTicketMapWrapper, AZ::SystemAllocator, 0);

        static void Reflect(AZ::ReflectContext* context);

        void Insert(int key, const AzFramework::EntitySpawnTicket& ticket);
        void Clear();
        [[nodiscard]] AZStd::vector<int> GetKeys() const;
        [[nodiscard]] AZStd::vector<AzFramework::EntitySpawnTicket> GetValues(int key) const;
        [[nodiscard]] const AZStd::unordered_map<int, AZStd::vector<AzFramework::EntitySpawnTicket>>& GetMap() const;
        void SetValues(int key, const AZStd::vector<AzFramework::EntitySpawnTicket>& values);
        void SetMap(const AZStd::unordered_map<int, AZStd::vector<AzFramework::EntitySpawnTicket>>& m_map);

    private:
        AZStd::unordered_map<int, AZStd::vector<AzFramework::EntitySpawnTicket>> m_map;
    };
} // namespace GeoJSONSpawner::GeoJSONWrappers
