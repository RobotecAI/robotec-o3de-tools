{
  "Source" : "./AutoBrick_DepthPass.azsl",

  "DepthStencilState" : { 
      "Depth" : { "Enable" : true, "CompareFunc" : "GreaterEqual" }
  },

  "ProgramSettings" : 
  {
      "EntryPoints":
      [
          {
              "name": "DepthPassVS",
              "type" : "Vertex"
          }
      ] 
  },

  "DrawList" : "depth"
}
