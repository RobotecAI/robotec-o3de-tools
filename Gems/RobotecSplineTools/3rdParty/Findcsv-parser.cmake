# Fetches the "CSV for C++" library's generated single-header release asset
# (vincentlaucsb/csv-parser) at configure time.
if (TARGET 3rdParty::csv-parser)
    return()
endif()

set(CSV_PARSER_VERSION "5.3.0")
set(CSV_PARSER_URL "https://github.com/vincentlaucsb/csv-parser/releases/download/${CSV_PARSER_VERSION}/csv.hpp")
set(CSV_PARSER_SHA256 "330b0d8950ff8b566ae726c022e2396f1dcdee36cb9ad6f6c1badc8febbf4319")

include(FetchContent)
FetchContent_Declare(
    csv-parser
    URL ${CSV_PARSER_URL}
    URL_HASH SHA256=${CSV_PARSER_SHA256}
    DOWNLOAD_NO_EXTRACT TRUE
)
FetchContent_MakeAvailable(csv-parser)

message(STATUS "SplineTools Gem uses csv-parser-${CSV_PARSER_VERSION} (MIT License) ${CSV_PARSER_URL}")

add_library(csv-parser INTERFACE)
target_include_directories(csv-parser INTERFACE ${csv-parser_SOURCE_DIR})

# The header defaults CSV_ENABLE_THREADS to 1, which pulls in std::thread/pthread.
target_compile_definitions(csv-parser INTERFACE CSV_ENABLE_THREADS=0)

get_property(this_gem_root GLOBAL PROPERTY "@GEMROOT:${gem_name}@")
ly_get_engine_relative_source_dir(${this_gem_root} relative_this_gem_root)
set_property(TARGET csv-parser PROPERTY FOLDER "${relative_this_gem_root}/External")

add_library(3rdParty::csv-parser ALIAS csv-parser)

set(csv-parser_FOUND TRUE)
