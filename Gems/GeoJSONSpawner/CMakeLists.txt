
# Query the gem name from the gem.json file if possible
# otherwise fallback to using GeoJSONSpawner
o3de_find_ancestor_gem_root(gem_path gem_name "${CMAKE_CURRENT_SOURCE_DIR}")
if (NOT gem_name)
    set(gem_name "GeoJSONSpawner")
endif()

# Fallback to using the current source CMakeLists.txt directory as the gem root path
if (NOT gem_path)
    set(gem_path ${CMAKE_CURRENT_SOURCE_DIR})
endif()

set(gem_json ${gem_path}/gem.json)

o3de_restricted_path(${gem_json} gem_restricted_path gem_parent_relative_path)

o3de_pal_dir(pal_dir ${CMAKE_CURRENT_SOURCE_DIR}/Platform/${PAL_PLATFORM_NAME} "${gem_restricted_path}" "${gem_path}" "${gem_parent_relative_path}")

ly_add_external_target_path(${CMAKE_CURRENT_SOURCE_DIR}/3rdParty)

add_subdirectory(Code)
