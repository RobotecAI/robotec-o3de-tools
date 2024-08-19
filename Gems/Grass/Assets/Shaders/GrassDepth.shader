{
    "Source": "GrassShadow.azsli",
    "DepthStencilState": {
      "Depth": {
        "Enable": true,
        "CompareFunc": "GreaterEqual"
      }
    },

    "RasterState" :
    {
        "CullMode" : "None"
    },
  "DrawList": "depth",
  "ProgramSettings": {
    "EntryPoints": [
      {
        "name": "MainVS"
      }
    ]
  }
}