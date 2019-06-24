[Mesh]
  type = FileMesh
  file = XE_NPP_outer_structure_100.e
[]

[GlobalParams]
  displacements = 'disp_x disp_y disp_z'
[]

[Variables]
  [./disp_x]
  [../]
  [./disp_y]
  [../]
  [./disp_z]
  [../]
[]

[AuxVariables]
  [./vel_x]
  [../]
  [./accel_x]
  [../]
  [./vel_y]
  [../]
  [./accel_y]
  [../]
  [./vel_z]
  [../]
  [./accel_z]
  [../]
[]

[Kernels]
  [./DynamicTensorMechanics]
    displacements = 'disp_x disp_y disp_z'
    # zeta = 0.000781
  [../]
  [./inertia_x]
    type = InertialForce
    variable = disp_x
    # velocity = vel_x
    # acceleration = accel_x
    # beta = 0.25
    # gamma = 0.5
    # eta = 0.64026
  [../]
  [./inertia_y]
    type = InertialForce
    variable = disp_y
    # velocity = vel_y
    # acceleration = accel_y
    # beta = 0.25
    # gamma = 0.5
    # eta = 0.64026
  [../]
  [./inertia_z]
    type = InertialForce
    variable = disp_z
    # velocity = vel_z
    # acceleration = accel_z
    # beta = 0.25
    # gamma = 0.5
    # eta = 0.64026
  [../]
[]

[AuxKernels]
  [./accel_x]
    type = TestNewmarkTI
    variable = accel_x
    displacement = disp_x
    first = false
  [../]
  [./vel_x]
    type = TestNewmarkTI
    variable = vel_x
    displacement = disp_x
  [../]
  [./accel_y]
    type = TestNewmarkTI
    variable = accel_y
    displacement = disp_y
    first = false
  [../]
  [./vel_y]
    type = TestNewmarkTI
    variable = vel_y
    displacement = disp_x
  [../]
  [./accel_z]
    type = TestNewmarkTI
    variable = accel_z
    displacement = disp_z
    first = false
  [../]
  [./vel_z]
    type = TestNewmarkTI
    variable = vel_z
    displacement = disp_z
  [../]
  # [./accel_x]
  #   type = NewmarkAccelAux
  #   variable = accel_x
  #   displacement = disp_x
  #   velocity = vel_x
  #   beta = 0.25
  #   execute_on = timestep_end
  # [../]
  # [./vel_x]
  #   type = NewmarkVelAux
  #   variable = vel_x
  #   acceleration = accel_x
  #   gamma = 0.5
  #   execute_on = timestep_end
  # [../]
  # [./accel_y]
  #   type = NewmarkAccelAux
  #   variable = accel_y
  #   displacement = disp_y
  #   velocity = vel_y
  #   beta = 0.25
  #   execute_on = timestep_end
  # [../]
  # [./vel_y]
  #   type = NewmarkVelAux
  #   variable = vel_y
  #   acceleration = accel_y
  #   gamma = 0.5
  #   execute_on = timestep_end
  # [../]
  # [./accel_z]
  #   type = NewmarkAccelAux
  #   variable = accel_z
  #   displacement = disp_z
  #   velocity = vel_z
  #   beta = 0.25
  #   execute_on = timestep_end
  # [../]
  # [./vel_z]
  #   type = NewmarkVelAux
  #   variable = vel_z
  #   acceleration = accel_z
  #   gamma = 0.5
  #   execute_on = timestep_end
  # [../]
[]

[BCs]
  [./bottom_z]
    type = DirichletBC
    variable = disp_z
    boundary = 1
    value = 0.0
  [../]
  [./bottom_y]
    type = DirichletBC
    variable = disp_y
    boundary = 1
    value = 0.0
  [../]
  [./bottom_disp]
    type = FunctionDirichletBC
    boundary = 1
    function = disp_bottom
    variable = 'disp_x'
    # beta = 0.25
    # acceleration = 'accel_x'
    # velocity = 'vel_x'
  [../]
  # [./Periodic]
  #   [./floor_7_x]
  #     primary = 2
  #     secondary = 3
  #     translation = '72.5 0 0'
  #   [../]
  #   [./floor_7_y]
  #     primary = 4
  #     secondary = 5
  #     translation = '0 61.92 0'
  #   [../]
  #   [./floor_6_x]
  #     primary = 6
  #     secondary = 7
  #     translation = '72.5 0 0'
  #   [../]
  #   [./floor_6_y]
  #     primary = 8
  #     secondary = 9
  #     translation = '0 61.92 0'
  #   [../]
  #   [./floor_5_x]
  #     primary = 10
  #     secondary = 11
  #     translation = '72.5 0 0'
  #   [../]
  #   [./floor_5_y]
  #     primary = 12
  #     secondary = 13
  #     translation = '0 61.92 0'
  #   [../]
  #   [./floor_4_x]
  #     primary = 14
  #     secondary = 15
  #     translation = '72.5 0 0'
  #   [../]
  #   [./floor_4_y]
  #     primary = 16
  #     secondary = 17
  #     translation = '0 61.92 0'
  #   [../]
  #   [./floor_3_x]
  #     primary = 18
  #     secondary = 19
  #     translation = '72.5 0 0'
  #   [../]
  #   [./floor_3_y]
  #     primary = 20
  #     secondary = 21
  #     translation = '0 61.92 0'
  #   [../]
  #   [./floor_2_x]
  #     primary = 22
  #     secondary = 23
  #     translation = '72.5 0 0'
  #   [../]
  #   [./floor_2_y]
  #     primary = 24
  #     secondary = 25
  #     translation = '0 61.92 0'
  #   [../]
  #   [./floor_1_x]
  #     primary = 26
  #     secondary = 27
  #     translation = '72.5 0 0'
  #   [../]
  #   [./floor_1_y]
  #     primary = 28
  #     secondary = 29
  #     translation = '0 61.92 0'
  #   [../]
  # [../]
[]

[Functions]
  [./disp_bottom]
     type = ParsedFunction
     value = 0.003*t*t*sin(2*pi*t/0.33)*cos(2*pi*t/0.1)
  [../]
[]

[Materials]
  [./elasticity_tensor_block]
    type = ComputeIsotropicElasticityTensor
    youngs_modulus = 5.8e8 # In psf. Assuming fc' of 5000psi.
    poissons_ratio = 0.2
    block = '1 2'
  [../]
  [./strain_block]
    type = ComputeIncrementalSmallStrain
    block = '1 2'
    displacements = 'disp_x disp_y disp_z'
    implicit = false
  [../]
  [./stress_block]
    type = ComputeFiniteStrainElasticStress
    block = '1 2'
  [../]
  [./density]
    type = GenericConstantMaterial
    block = '1 2'
    prop_names = density
    prop_values = 4.658 # for 150 pcf weight density. Mass density = 150/32.2 pcf/g
  [../]
[]

# [Preconditioning]
#   [./andy]
#     type = SMP
#     full = true
#   [../]
# []

[Executioner]
  type = Transient
  start_time = 0
  end_time = 4
  dt = 0.000048
  timestep_tolerance = 1e-6
  [./TimeIntegrator]
    type = CentralDifference
  [../]
[]

[VectorPostprocessors]
  [./nodal_histories]
    type = ResponseHistoryBuilder
    boundary = 100
    variables = 'disp_x accel_x'
  [../]
  [./accel_spec]
    type = ResponseSpectraCalculator
    vectorpostprocessor = nodal_histories
    regularize_dt = 0.005
    outputs = specout
  [../]
[]

[Postprocessors]
  [./base_input]
    type = PointValue
    point = '-39.25 -33.96 0.0'
    variable = 'disp_x'
  [../]
[]

[Outputs]
  csv = true
  exodus = false
  perf_graph = true
  print_linear_residuals = true
  [./specout]
    type = CSV
    execute_on = 'final'
  [../]
  [./exout]
    type = Exodus
    interval = 500
  [../]

[]
