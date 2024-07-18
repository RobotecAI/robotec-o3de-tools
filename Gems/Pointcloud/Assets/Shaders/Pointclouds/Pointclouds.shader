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
