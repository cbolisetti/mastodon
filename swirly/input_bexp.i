[Mesh]
  type = FileMesh
  file = 'ex05_mesh.e'
[]

[MeshModifiers]
  [./soilbox]
    type = BoundingBoxNodeSet
    top_right = '24 24 48'
    bottom_left = '-24 -24 0'
    new_boundary = '123'
  [../]
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
  [./stress_xy]
    order = CONSTANT
    family = MONOMIAL
  [../]
  [./stress_xx]
    order = CONSTANT
    family = MONOMIAL
  [../]
  [./stress_yy]
    order = CONSTANT
    family = MONOMIAL
  [../]
  [./stress_zz]
    order = CONSTANT
    family = MONOMIAL
  [../]
  [./stress_yz]
    order = CONSTANT
    family = MONOMIAL
  [../]
  [./stress_zx]
    order = CONSTANT
    family = MONOMIAL
  [../]
  [./strain_yz]
    order = CONSTANT
    family = MONOMIAL
  [../]
  [./strain_zx]
    order = CONSTANT
    family = MONOMIAL
  [../]
  [./layer_id]
    order = CONSTANT
    family = MONOMIAL
  [../]
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
  [./TensorMechanics]
    displacements = 'disp_x disp_y disp_z'
  [../]
  # [./gravity]
  #   type = Gravity
  #   variable = disp_z
  #   value = -386.09
  # [../]
  [./inertia_x]
    type = InertialForce
    variable = disp_x
    velocity = vel_x
    acceleration = accel_x
    beta = 0.25
    gamma = 0.5
    eta=0.0
  [../]
  [./inertia_y]
    type = InertialForce
    variable = disp_y
    velocity = vel_y
    acceleration = accel_y
    beta = 0.25
    gamma = 0.5
    eta=0.0
  [../]
  [./inertia_z]
    type = InertialForce
    variable = disp_z
    velocity = vel_z
    acceleration = accel_z
    beta = 0.25
    gamma = 0.5
    eta = 0.0
  [../]
[]

[AuxKernels]
  [./stress_zx]
    type = RankTwoAux
    rank_two_tensor = stress
    variable = stress_zx
    index_i = 2
    index_j = 0
  [../]
  [./strain_zx]
    type = RankTwoAux
    rank_two_tensor = total_strain
    variable = strain_zx
    index_i = 2
    index_j = 0
  [../]
  [./stress_xx]
    type = RankTwoAux
    rank_two_tensor = stress
    variable = stress_xx
    index_i = 0
    index_j = 0
  [../]
  [./stress_yy]
    type = RankTwoAux
    rank_two_tensor = stress
    variable = stress_yy
    index_i = 1
    index_j = 1
  [../]
  [./stress_zz]
    type = RankTwoAux
    rank_two_tensor = stress
    variable = stress_zz
    index_i = 2
    index_j = 2
  [../]
  [./stress_xy]
    type = RankTwoAux
    rank_two_tensor = stress
    variable = stress_xy
    index_i = 0
    index_j = 1
  [../]
  [./stress_yz]
    type = RankTwoAux
    rank_two_tensor = stress
    variable = stress_yz
    index_i = 1
    index_j = 2
  [../]
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
  [./layer_1]
    type = UniformLayerAuxKernel
    variable = layer_id
    interfaces = '48.24'
    direction = '0.0 0.0 1.0'
    block = 2
    execute_on = initial
  [../]
[]

[BCs]
  [./fix_x_soil]
    type = PresetBC
    variable = disp_x
    boundary = 100
    value = 0.0
  [../]
  [./fix_y_soil]
    type = PresetBC
    variable = disp_y
    boundary = 100
    value = 0.0
  [../]
  [./fix_z_soil]
    type = PresetBC
    variable = disp_z
    boundary = 100
    value = 0.0
  [../]
  [./concrete_pressure]
    type = Pressure
    boundary = 101
    variable = disp_z
    component = 2
    factor = 5 #psi
  [../]
  [./top_x]
    type = PresetDisplacement
    boundary = 1000
    variable = disp_x
    beta = 0.25
    velocity = vel_x
    acceleration = accel_x
    function = loading_bc
  [../]
[]

[Functions]
  [./loading_bc]
    type = PiecewiseLinear
    data_file = 'const_sine.csv'
    format = columns
  [../]
  [./initial_z]
    type = ParsedFunction
    value = '5 + (0.0868 * (6 + (48.24 - z)))' #normal pressure = 5 psi
  [../]
  [./initial_xy]
    type = ParsedFunction
    value = '(5 + (0.0868 * (6 + (48.24 - z)))) * 0.15/0.85'
  [../]
[]

[Materials]
  [./elasticity_tensor_block]
    youngs_modulus = 4e6 #psi
    poissons_ratio = 0.25
    type = ComputeIsotropicElasticityTensor
    block = 3
  [../]
  [./strain_block]
    type = ComputeIncrementalSmallStrain
    block = 3
    displacements = 'disp_x disp_y disp_z'
  [../]
  [./stress_block]
    type = ComputeFiniteStrainElasticStress
    block = 3
  [../]
  [./den_block]
    type = GenericConstantMaterial
    block = 3
    prop_names = density
    prop_values = 0.0002248 #slug/in^3
  [../]

  [./elasticity_tensor_soil]
    youngs_modulus = 1.3983e+05 #psi
    poissons_ratio = 0.3
    type = ComputeIsotropicElasticityTensor
    block = 1
  [../]
  [./strain_soil]
    type = ComputeIncrementalSmallStrain
    block = 1
    displacements = 'disp_x disp_y disp_z'
  [../]
  [./stress_soil]
    type = ComputeFiniteStrainElasticStress
    block = 1
  [../]
  [./den_soil]
    type = GenericConstantMaterial
    block = 1
    prop_names = density
    prop_values = 0.0001356 #slug/in^3
  [../]
  [./I_Soil]
    [./thinlayer]
      layer_variable = layer_id
      layer_ids = '0'
      soil_type = 'thin_layer'
      poissons_ratio = '0.4'
      block = 2
      # initial_soil_stress = 'initial_xy 0 0  0 initial_xy 0  0 0 initial_z'
      density = 0.0002248 #slug/in^3
      p_ref = 5.52 #psi
      initial_shear_modulus = '1.739e6' #psi
      friction_coefficient = '0.7'
      hardening_ratio = '0.001'
      tension_pressure_cut_off = -0.01
      # pressure_dependency = false
      # b_exp = 0
    [../]
  [../]
[]

[Preconditioning]
  [./andy]
    type = SMP
    full = true
  [../]
[]

[Postprocessors]
  [./_dt]
    type = TimestepSize
  [../]
  [./stress_xx_interface]
    type = ElementAverageValue
    block = '2'
    variable = stress_xx
  [../]
  [./stress_xy_interface]
    type = ElementAverageValue
    block = '2'
    variable = stress_xy
  [../]
  [./stress_yy_interface]
    type = ElementAverageValue
    block = '2'
    variable = stress_yy
  [../]
  [./stress_yz_interface]
    type = ElementAverageValue
    block = '2'
    variable = stress_yz
  [../]
  [./stress_zz_interface]
    type = ElementAverageValue
    block = '2'
    variable = stress_zz
  [../]
  [./strain_zx_interface]
    type = ElementAverageValue
    block = '2'
    variable = strain_zx
  [../]
  [./stress_zx_interface]
    type = ElementAverageValue
    block = '2'
    variable = stress_zx
  [../]
[]

[Controls]
  [./inertia_switch]
    type = TimePeriod
    start_time = 0.97
    end_time = 1
    # enable_objects = ''
    disable_objects = '*/inertia_x */inertia_y */inertia_z */vel_x */vel_y */vel_z */accel_x */accel_y */accel_z'
    set_sync_times = true
  [../]
[]

[Executioner]
  type = Transient
  solve_type = PJFNK
  nl_abs_tol = 1e-2
  nl_rel_tol = 1e-4
  l_tol = 1e-2
  l_max_its = 20
  start_time = 0.97
  end_time = 2
  dt = 0.01
  timestep_tolerance = 1e-3
  petsc_options = '-snes_ksp_ew'
  petsc_options_iname = '-ksp_gmres_restart -pc_type -pc_hypre_type -pc_hypre_boomeramg_max_iter'
  petsc_options_value = '201                hypre    boomeramg      4'
  line_search = 'none'
[]

[Outputs]
  csv = true
  exodus = true
  # file_base = outcyclin
  print_linear_residuals = false
  [./screen]
    type = Console
    max_rows = 1
  [../]
[]
