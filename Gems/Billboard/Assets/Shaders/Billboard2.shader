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
        "BlendSource" : "AlphaSource",
        "BlendDest" : "AlphaSourceInverse",
        "BlendAlphaOp" : "Add"
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
