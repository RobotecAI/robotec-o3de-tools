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

#include <AzCore/Asset/AssetCommon.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/Math/Vector3.h>
#include <AzCore/std/string/string.h>
#include <AzFramework/Spawnable/SpawnableEntitiesInterface.h>
#include <CsvSpawner/CsvSpawnerUtils.h>

namespace CsvSpawner::CsvSpawnerUtils
{
    //! Returns a vector of @class SpawnableEntityInfo from a CSV file, this function is used to parse the CSV file, in Editor
    //! or during the compilation of entity that has a component @class CsvSpawnerEditorComponent .
    //! This function allows to move information about spawnables outside of the prefab.
    //! The CSV should be formatted as follows:
    //!
    //! ```
    //!     x,y,z,name
    //!     1,2,3,tree1
    //! ```
    //! Optional columns:
    //!  - seed : allows to manually set the seed for the random generator
    //!  - z    : allows to set the height of the entity
    //!  - id   : allows to set the id of the tree, if not set, the id will be set to the row number in CSV file
    //! @param csvFilePath the path to the CSV file
    //! @return vector of SpawnableEntityInfo
    AZStd::vector<CsvSpawnableEntityInfo> GetSpawnableEntityInfoFromCSV(const AZStd::string& csvFilePath);

} // namespace CsvSpawner::CsvSpawnerUtils
