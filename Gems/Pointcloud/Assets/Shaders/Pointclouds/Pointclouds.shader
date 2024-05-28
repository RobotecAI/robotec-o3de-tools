{
  "Source": "Pointclouds.azsl",
  "DepthStencilState": {
    "Depth": {
      "Enable": true,
      "CompareFunc": "GreaterEqual"
    }
  },
  // "GlobalTargetBlendState" : {
  //     "Enable" : true,
  //     "BlendSource" : "AlphaSource",
  //     "BlendDest" : "One",
  //     "BlendOp" : "Minimum"
  // },
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
