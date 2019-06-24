[Mesh]
  type = FileMesh
  file = 'XE_NPP_outer_structure_100.e'
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
  [../]
  [./inertia_x]
    type = InertialForce
    variable = disp_x
    velocity = vel_x
    acceleration = accel_x
    beta = 0.25
    gamma = 0.5
  [../]
  [./inertia_y]
    type = InertialForce
    variable = disp_y
    velocity = vel_y
    acceleration = accel_y
    beta = 0.25
    gamma = 0.5
  [../]
  [./inertia_z]
    type = InertialForce
    variable = disp_z
    velocity = vel_z
    acceleration = accel_z
    beta = 0.25
    gamma = 0.5
  [../]
[]

[AuxKernels]
  [./accel_x]
    type = NewmarkAccelAux
    variable = accel_x
    displacement = disp_x
    velocity = vel_x
    beta = 0.25
    execute_on = timestep_end
  [../]
  [./vel_x]
    type = NewmarkVelAux
    variable = vel_x
    acceleration = accel_x
    gamma = 0.5
    execute_on = timestep_end
  [../]
  [./accel_y]
    type = NewmarkAccelAux
    variable = accel_y
    displacement = disp_y
    velocity = vel_y
    beta = 0.25
    execute_on = timestep_end
  [../]
  [./vel_y]
    type = NewmarkVelAux
    variable = vel_y
    acceleration = accel_y
    gamma = 0.5
    execute_on = timestep_end
  [../]
  [./accel_z]
    type = NewmarkAccelAux
    variable = accel_z
    displacement = disp_z
    velocity = vel_z
    beta = 0.25
    execute_on = timestep_end
  [../]
  [./vel_z]
    type = NewmarkVelAux
    variable = vel_z
    acceleration = accel_z
    gamma = 0.5
    execute_on = timestep_end
  [../]
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
    type = PresetDisplacement
    boundary = 1
    function = disp_bottom
    variable = 'disp_x'
    beta = 0.25
    acceleration = 'accel_x'
    velocity = 'vel_x'
  [../]
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
    # implicit = false
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

[Preconditioning]
  [./full_jacobian]
    type = SMP
    full = true
  [../]
[]

[Executioner]
  type = Transient
  solve_type = NEWTON
  nl_abs_tol = 1e-3
  nl_rel_tol = 1e-6
  l_tol = 1e-6
  l_max_its = 50
  start_time = 0
  end_time = 4
  dt = 0.005
  timestep_tolerance = 1e-6
  [./TimeIntegrator]
    type = NewmarkBeta
  [../]
  petsc_options = '-snes_ksp_ew'
  petsc_options_iname = '-ksp_gmres_restart -pc_type -pc_hypre_type -pc_hypre_boomeramg_max_iter'
  petsc_options_value = '201                hypre    boomeramg      4'
  line_search = 'none'
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
  exodus = true
  perf_graph = true
  print_linear_residuals = false
  [./specout]
    type = CSV
    execute_on = 'final'
  [../]
[]
