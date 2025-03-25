#include "SpawnTicketMapWrapper.h"

namespace GeoJSONSpawner::GeoJSONWrappers
{
    void SpawnTicketMapWrapper::Reflect(AZ::ReflectContext* context)
    {
        if (auto behaviorContext = azrtti_cast<AZ::BehaviorContext*>(context))
        {
            behaviorContext->Class<SpawnTicketMapWrapper>("SpawnTicketMapWrapper")
                ->Attribute(AZ::Script::Attributes::Category, "GeoJSONSpawner/Wrappers")
                ->Method("Insert", &SpawnTicketMapWrapper::Insert)
                ->Method("Clear", &SpawnTicketMapWrapper::Clear)
                ->Method("GetKeys", &SpawnTicketMapWrapper::GetKeys)
                ->Method("GetValues", &SpawnTicketMapWrapper::GetValues);
        }
    }

    void SpawnTicketMapWrapper::Insert(int key, const AzFramework::EntitySpawnTicket& ticket)
    {
        m_map[key].push_back(ticket);
    }

    void SpawnTicketMapWrapper::Clear()
    {
        m_map.clear();
    }

    AZStd::vector<int> SpawnTicketMapWrapper::GetKeys() const
    {
        AZStd::vector<int> keys;
        for (const auto& pair : m_map)
        {
            keys.push_back(pair.first);
        }
        return keys;
    }

    AZStd::vector<AzFramework::EntitySpawnTicket> SpawnTicketMapWrapper::GetValues(int key) const
    {
        auto it = m_map.find(key);
        if (it != m_map.end())
        {
            return it->second;
        }
        return {};
    }

    void SpawnTicketMapWrapper::SetValues(int key, const AZStd::vector<AzFramework::EntitySpawnTicket>& values)
    {
        m_map[key] = values;
    }

    const AZStd::unordered_map<int, AZStd::vector<AzFramework::EntitySpawnTicket>>& SpawnTicketMapWrapper::GetMap() const
    {
        return m_map;
    }

    void SpawnTicketMapWrapper::SetMap(const AZStd::unordered_map<int, AZStd::vector<AzFramework::EntitySpawnTicket>>& m_map)
    {
        this->m_map = m_map;
    }
} // namespace GeoJSONSpawner::GeoJSONWrappers
