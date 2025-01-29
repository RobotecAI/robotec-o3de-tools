/**
 * Copyright (C) Robotec AI - All Rights Reserved
 *
 * This source code is protected under international copyright law.  All rights
 * reserved and protected by the copyright holders.
 * This file is confidential and only available to authorized individuals with the
 * permission of the copyright holders.  If you encounter this file and do not have
 * permission, please contact the copyright holders and delete this file.
 */

#include <AzCore/Asset/AssetCommon.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/Math/Vector3.h>
#include <AzCore/std/string/string.h>
#include <AzFramework/Spawnable/SpawnableEntitiesInterface.h>
#include <CsvSpawner/CsvSpawnerUtils.h>
#include <ROS2/Georeference/GeoreferenceBus.h>
#include <csv/csv.hpp>

namespace CsvSpawner::CsvSpawnerUtils
{
    AZStd::optional<int> GetColumn(const csv::CSVReader& reader, const AZStd::string& columnName)
    {
        const int column = reader.index_of(columnName.c_str());
        return (column < 0) ? AZStd::nullopt : AZStd::make_optional(column);
    }

    AZStd::vector<CsvSpawnableEntityInfo> GetSpawnableEntityInfoFromCSV(const AZStd::string& csvFilePath)
    {
        try
        {
            AZStd::vector<CsvSpawnableEntityInfo> ret;

            csv::CSVReader reader(csvFilePath.c_str());
            AZStd::vector<CsvSpawnableEntityInfo> parsedData;

            const auto index_id = GetColumn(reader, "id");
            const auto index_X = GetColumn(reader, "x");
            const auto index_Y = GetColumn(reader, "y");
            const auto index_Z = GetColumn(reader, "z");
            const auto index_lat = GetColumn(reader, "lat");
            const auto index_lon = GetColumn(reader, "lon");
            const auto index_alt = GetColumn(reader, "alt");

            const auto index_name = GetColumn(reader, "name");
            const auto index_seed = GetColumn(reader, "seed");

            const bool isLocalCoordinates = index_X && index_Y && index_name;
            const bool isWGSCoordinates = index_lat && index_lon && index_alt && index_name;
            if (!isLocalCoordinates && !isWGSCoordinates)
            {
                AZ_Error("CsvSpawnerEditorComponent", false, "CSV file must have columns named x, y, name or lat, lon, alt, name");
                return {};
            }

            unsigned int id = 0;
            for (csv::CSVRow& row : reader)
            {
                CsvSpawnableEntityInfo info;
                // handle id column
                if (index_id)
                {
                    info.m_id = row[index_id.value()].get<uint64_t>();
                }
                else
                {
                    info.m_id = id++;
                }
                info.m_transform = AZ::Transform::CreateIdentity();

                if (isLocalCoordinates)
                {
                    // handle Z column
                    if (index_Z)
                    {
                        info.m_transform.SetTranslation(AZ::Vector3(
                            row[index_X.value()].get<float>(), row[index_Y.value()].get<float>(), row[index_Z.value()].get<float>()));
                    }
                    else
                    {
                        info.m_transform.SetTranslation(
                            AZ::Vector3(row[index_X.value()].get<float>(), row[index_Y.value()].get<float>(), 0));
                    }
                }
                else if (isWGSCoordinates)
                {
                    ROS2::WGS::WGS84Coordinate coordinate;
                    coordinate.m_latitude = row[index_lat.value()].get<double>();
                    coordinate.m_longitude = row[index_lon.value()].get<double>();
                    coordinate.m_altitude = row[index_alt.value()].get<double>();
                    AZ::Vector3 coordinateInLevel = AZ::Vector3(-1);
                    ROS2::GeoreferenceRequestsBus::BroadcastResult(
                        coordinateInLevel, &ROS2::GeoreferenceRequests::ConvertFromWGS84ToLevel, coordinate);
                    AZ_Printf(
                        "TreeSpawnerEditorComponent",
                        "Converted WGS84 coordinate to level coordinate: %f, %f, %f WGS84: %f, %f, %f\n",
                        coordinateInLevel.GetX(),
                        coordinateInLevel.GetY(),
                        coordinateInLevel.GetZ(),
                        coordinate.m_latitude,
                        coordinate.m_longitude,
                        coordinate.m_altitude);
                    info.m_transform.SetTranslation(coordinateInLevel);
                }
                // handle seed column
                if (index_seed)
                {
                    info.m_seed = row[index_seed.value()].get<uint64_t>();
                }

                info.m_name = AZStd::string(row[index_name.value()].get<std::string>().c_str());

                ret.emplace_back(AZStd::move(info));
            }

            return ret;
        } catch (std::runtime_error& exception)
        {
            AZ_Error("CsvSpawnerEditorComponent", false, "Error parsing CSV file: %s", exception.what());
        }
        return {};
    }

} // namespace CsvSpawner::CsvSpawnerUtils
