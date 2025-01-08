#pragma once

namespace GeoJSONSpawner
{
    inline constexpr const char* GeoJSONSchema = R"({
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "Extended GeoJSON with Spawnable Name and ID in Properties",
  "type": "object",
  "properties": {
    "type": {
      "type": "string",
      "enum": ["FeatureCollection"]
    },
    "features": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "type": {
            "type": "string",
            "enum": ["Feature"]
          },
          "geometry": {
            "type": "object",
            "oneOf": [
              {
                "type": "object",
                "properties": {
                  "type": {
                    "type": "string",
                    "enum": [
                      "Point",
                      "MultiPoint",
                      "LineString",
                      "MultiLineString",
                      "Polygon",
                      "MultiPolygon"
                    ]
                  },
                  "coordinates": {
                    "oneOf": [
                      {
                        "type": "array",
                        "items": [
                          { "type": "number" },
                          { "type": "number" }
                        ],
                        "minItems": 2,
                        "maxItems": 2
                      },
                      {
                        "type": "array",
                        "items": {
                          "type": "array",
                          "items": [
                            { "type": "number" },
                            { "type": "number" }
                          ],
                          "minItems": 2,
                          "maxItems": 2
                        }
                      },
                      {
                        "type": "array",
                        "items": {
                          "type": "array",
                          "items": {
                            "type": "array",
                            "items": [
                              { "type": "number" },
                              { "type": "number" }
                            ],
                            "minItems": 2,
                            "maxItems": 2
                          }
                        }
                      },
                      {
                        "type": "array",
                        "items": {
                          "type": "array",
                          "items": {
                            "type": "array",
                            "items": {
                              "type": "array",
                              "items": [
                                { "type": "number" },
                                { "type": "number" }
                              ],
                              "minItems": 2,
                              "maxItems": 2
                            }
                          }
                        }
                      }
                    ]
                  }
                },
                "required": ["type", "coordinates"]
              },
              {
                "type": "object",
                "properties": {
                  "type": {
                    "type": "string",
                    "enum": ["GeometryCollection"]
                  },
                  "geometries": {
                    "type": "array",
                    "items": {
                      "type": "object",
                      "$ref": "#/properties/features/items/properties/geometry/oneOf/0"
                    }
                  }
                },
                "required": ["type", "geometries"]
              }
            ]
          },
          "properties": {
            "type": "object",
            "properties": {
              "spawnable_name": {
                "type": "string"
              },
              "id": {
                "type": "integer"
              }
            },
            "required": ["spawnable_name", "id"]
          }
        },
        "required": ["type", "geometry", "properties"]
      }
    }
  },
  "required": ["type", "features"]
})";
} // namespace GeoJSONSpawner
