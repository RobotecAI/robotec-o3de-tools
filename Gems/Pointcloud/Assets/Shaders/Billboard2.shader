{
    "Source": "Billboard2.azsl",
    "DepthStencilState": {
      "Depth": {
        "Enable": true,
        "CompareFunc": "GreaterEqual"
      }
    },
    "GlobalTargetBlendState" :
    {
        "Enable" : true,
        "BlendSource" : "One",
        "BlendDest" : "AlphaSourceInverse",
        "BlendAlphaOp" : "Maximum"
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