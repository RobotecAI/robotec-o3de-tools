{
    "Source": "Grass.azsli",
    "DepthStencilState": {
      "Depth": {
        "Enable": true,
        "CompareFunc": "GreaterEqual"
      },
      "Stencil" :
      {
          "Enable" : false
      }
    },
    "GlobalTargetBlendState" :
    {
        "Enable" : false,
        "BlendSource" : "AlphaSource",
        "BlendDest" : "AlphaSourceInverse",
        "BlendAlphaOp" : "Add"
    },

    "RasterState" :
    {
        "CullMode" : "None"
    },
  "DrawList": "forward",
  "ProgramSettings": {
    "EntryPoints": [
      {
        "name": "MainVS",
        "type": "Vertex"
      },
      {
        "name": "MainPS",
        "type": "Fragment"
      }
    ]
  }
}