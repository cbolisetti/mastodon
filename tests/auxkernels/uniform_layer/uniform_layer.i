[Mesh]
  type = GeneratedMesh
  dim = 3
  nx = 20
  ny = 20
  nz = 20
  xmax = 100
  ymax = 100
  zmax = 100
[]

[AuxVariables]
  [./layer_id]
    family = MONOMIAL
    order = CONSTANT
  [../]
[]

[AuxKernels]
  [./layers]
    type = UniformLayerAuxKernel
    variable = layer_id
    interfaces = '10 20 50 1000'
    direction = '1 1 2'
    execute_on = initial
  [../]
[]

[Executioner]
  type = Steady
[]

[Problem]
  solve = false
[]

[Outputs]
  exodus = true
[]
