{
  "Source": "Pointclouds.azsl",
  "DepthStencilState": {
    "Depth": {
      "Enable": true,
      "CompareFunc": "GreaterEqual"
    }
  },
  "GlobalTargetBlendState" : {
      "Enable" : false,
      "BlendSource" : "Zero",
      "BlendDest" : "One",
      "BlendOp" : "Add"
  },
  "DrawList": "auxgeom",
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