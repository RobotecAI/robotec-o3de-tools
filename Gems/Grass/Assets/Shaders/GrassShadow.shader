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
  "DrawList": "shadow",
  "ProgramSettings": {
    "EntryPoints": [
      {
        "name": "MainVS"
      }
    ]
  }
}