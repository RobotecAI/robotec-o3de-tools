{
    "Source" : "./AutoBrick_ForwardPass.azsl",

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
        "BlendAlphaOp" : "Add"
    },

    
    "ProgramSettings":
    {
      "EntryPoints":
      [
        {
          "name": "AutoBrick_ForwardPassVS",
          "type": "Vertex"
        },
        {
          "name": "AutoBrick_ForwardPassPS",
          "type": "Fragment"
        }
      ]
    },

    "DrawList" : "forward"
  }
