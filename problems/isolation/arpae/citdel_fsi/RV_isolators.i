[Mesh]
  [file]
    type = FileMeshGenerator
    file = RV_isolators.e
  []
  [./interface1]
    type = SideSetsBetweenSubdomainsGenerator
    input = file
    primary_block = '1'
    paired_block = '2'
    new_boundary = 'Interface'
  [../]
[]

[Variables]
  [./p]
   block = 2
  [../]
  [./disp_x]
    block = '1 3 4 5'
  [../]
  [./disp_y]
    block = '1 3 4 5'
  [../]
  [./disp_z]
    block = '1 3 4 5'
  [../]
  [./rot_x]
    block = '4 5'
    order = FIRST
    family = LAGRANGE
  [../]
  [./rot_y]
    block = '4 5'
    order = FIRST
    family = LAGRANGE
  [../]
  [./rot_z]
    block = '4 5'
    order = FIRST
    family = LAGRANGE
  [../]
[]

[AuxVariables]
  [./Wave1]
    block = 2
  [../]
  [./vel_x]
    order = FIRST
    family = LAGRANGE
    block = '1 3 4 5'
  [../]
  [./accel_x]
    order = FIRST
    family = LAGRANGE
    block = '1 3 4 5'
  [../]
  [./vel_y]
    order = FIRST
    family = LAGRANGE
    block = '1 3 4 5'
  [../]
  [./accel_y]
    order = FIRST
    family = LAGRANGE
    block = '1 3 4 5'
  [../]
  [./vel_z]
    order = FIRST
    family = LAGRANGE
    block = '1 3 4 5'
  [../]
  [./accel_z]
    order = FIRST
    family = LAGRANGE
    block = '1 3 4 5'
  [../]
  [./rot_vel_x]
    block = '4 5'
    order = FIRST
    family = LAGRANGE
  [../]
  [./rot_vel_y]
    block = '4 5'
    order = FIRST
    family = LAGRANGE
  [../]
  [./rot_vel_z]
    block = '4 5'
    order = FIRST
    family = LAGRANGE
  [../]
  [./rot_accel_x]
    block = '4 5'
    order = FIRST
    family = LAGRANGE
  [../]
  [./rot_accel_y]
    block = '4 5'
    order = FIRST
    family = LAGRANGE
  [../]
  [./rot_accel_z]
    block = '4 5'
    order = FIRST
    family = LAGRANGE
  [../]
[]

[Modules/TensorMechanics/LineElementMaster]
    displacements = 'disp_x disp_y disp_z'
    rotations = 'rot_x rot_y rot_z'

    # dynamic simulation using consistent mass/inertia matrix
    dynamic_nodal_translational_inertia = true

    velocities = 'vel_x vel_y vel_z'
    accelerations = 'accel_x accel_y accel_z'
    rotational_velocities = 'rot_vel_x rot_vel_y rot_vel_z'
    rotational_accelerations = 'rot_accel_x rot_accel_y rot_accel_z'

    # beta = 0.25 # Newmark time integration parameter
    # gamma = 0.5 # Newmark time integration parameter

    beta = 0.276
    gamma = 0.45
    alpha = -0.05

    # beta = 0.3025
    # gamma = 0.6
    # alpha = -0.1

    [./block_5]
      block = 5
      area = 130.06
      Iy = 24166.729
      Iz = 24166.729
      y_orientation = '0.0 0.0 1.0'

      nodal_mass = 1e-10
      boundary = IsoTop
    [../]
[]

[Kernels]
  [./diffusion]
    type = Diffusion
    variable = 'p'
    block = 2
  [../]
  [./inertia]
    type = AcousticInertia
    variable = p
    block = 2
  [../]
  [./DynamicTensorMechanics]
    displacements = 'disp_x disp_y disp_z'
    block = '1 3'
    alpha = -0.1
  [../]
  [./inertia_x1]
    type = InertialForce
    variable = disp_x
    block = '1 3'
  [../]
  [./inertia_y1]
    type = InertialForce
    variable = disp_y
    block = '1 3'
  [../]
  [./inertia_z1]
    type = InertialForce
    variable = disp_z
    block = '1 3'
  [../]
  [./lr_disp_x]
    block = 4
    type = StressDivergenceIsolator
    displacements = 'disp_x disp_y disp_z'
    rotations = 'rot_x rot_y rot_z'
    component = 0
    variable = disp_x
  [../]
  [./lr_disp_y]
    block = 4
    type = StressDivergenceIsolator
    displacements = 'disp_x disp_y disp_z'
    rotations = 'rot_x rot_y rot_z'
    component = 1
    variable = disp_y
  [../]
  [./lr_disp_z]
    block = 4
    type = StressDivergenceIsolator
    displacements = 'disp_x disp_y disp_z'
    rotations = 'rot_x rot_y rot_z'
    component = 2
    variable = disp_z
  [../]
  [./lr_rot_x]
    block = 4
    type = StressDivergenceIsolator
    displacements = 'disp_x disp_y disp_z'
    rotations = 'rot_x rot_y rot_z'
    component = 3
    variable = rot_x
  [../]
  [./lr_rot_y]
    block = 4
    type = StressDivergenceIsolator
    displacements = 'disp_x disp_y disp_z'
    rotations = 'rot_x rot_y rot_z'
    component = 4
    variable = rot_y
  [../]
  [./lr_rot_z]
    block = 4
    type = StressDivergenceIsolator
    displacements = 'disp_x disp_y disp_z'
    rotations = 'rot_x rot_y rot_z'
    component = 5
    variable = rot_z
  [../]
[]

[NodalKernels]
  [./force_x]
    type = UserForcingFunctionNodalKernel
    variable = disp_z
    boundary = IsoTop
    function = force_z
  [../]
[]

[AuxKernels]
  [./waves]
    type = WaveHeightAuxKernel
    variable = 'Wave1'
    pressure = p
    density = 1e-6
    gravity = 9.81
    execute_on = timestep_end
    block = 2
  [../]
  [./accel_x]
    type = TestNewmarkTI
    displacement = disp_x
    variable = accel_x
    first = false
    block = '1 3 4'
  [../]
  [./vel_x]
    type = TestNewmarkTI
    displacement = disp_x
    variable = vel_x
    block = '1 3 4'
  [../]
  [./accel_y]
    type = TestNewmarkTI
    displacement = disp_y
    variable = accel_y
    first = false
    block = '1 3 4'
  [../]
  [./vel_y]
    type = TestNewmarkTI
    displacement = disp_y
    variable = vel_y
    block = '1 3 4'
  [../]
  [./accel_z]
    type = TestNewmarkTI
    displacement = disp_z
    variable = accel_z
    first = false
    block = '1 3 4'
  [../]
  [./vel_z]
    type = TestNewmarkTI
    displacement = disp_z
    variable = vel_z
    block = '1 3 4'
  [../]
  [./rot_accel_x]
    block = 4
    type = TestNewmarkTI
    displacement = rot_x
    variable = rot_accel_x
    first = false
  [../]
  [./rot_vel_x]
    block = 4
    type = TestNewmarkTI
    displacement = rot_x
    variable = rot_vel_x
  [../]
  [./rot_accel_y]
    block = 4
    type = TestNewmarkTI
    displacement = rot_y
    variable = rot_accel_y
    first = false
  [../]
  [./rot_vel_y]
    block = 4
    type = TestNewmarkTI
    displacement = rot_y
    variable = rot_vel_y
  [../]
  [./rot_accel_z]
    block = 4
    type = TestNewmarkTI
    displacement = rot_z
    variable = rot_accel_z
    first = false
  [../]
  [./rot_vel_z]
    block = 4
    type = TestNewmarkTI
    displacement = rot_z
    variable = rot_vel_z
  [../]
[]

[InterfaceKernels]
  [./interface1]
    type = FSI_test
    variable = disp_x
    neighbor_var = p
    boundary = 'Interface'
    D = 1e-6
    component = 0
  [../]
  [./interface2]
    type = FSI_test
    variable = disp_y
    neighbor_var = p
    boundary = 'Interface'
    D = 1e-6
    component = 1
  [../]
  [./interface3]
    type = FSI_test
    variable = disp_z
    neighbor_var = p
    boundary = 'Interface'
    D = 1e-6
    component = 2
  [../]
[]

[BCs]
  # [./fixrx0]
  #   type = DirichletBC
  #   variable = rot_x
  #   boundary = 'IsoBottom'
  #   value = 0.0
  # [../]
  # [./fixry0]
  #   type = DirichletBC
  #   variable = rot_y
  #   boundary = 'IsoBottom'
  #   value = 0.0
  # [../]
  # [./fixrz0]
  #   type = DirichletBC
  #   variable = rot_z
  #   boundary = 'IsoBottom'
  #   value = 0.0
  # [../]
  # [./fixrx1]
  #   type = DirichletBC
  #   variable = rot_x
  #   boundary = 'IsoTop'
  #   value = 0.0
  # [../]
  # [./fixry1]
  #   type = DirichletBC
  #   variable = rot_y
  #   boundary = 'IsoTop'
  #   value = 0.0
  # [../]
  # [./fixrz1]
  #   type = DirichletBC
  #   variable = rot_z
  #   boundary = 'IsoTop'
  #   value = 0.0
  # [../]
  [./bottom_accel1]
    type = PresetAcceleration
    variable = disp_x
    velocity = vel_x
    acceleration = accel_x
    beta = 0.25
    function = 'ormsby'
    boundary = 'IsoBottom'
    # type = FunctionDirichletBC
    # variable = disp_x
    # boundary = 'IsoBottom'
    # function = accel_bottom
  [../]
  # [./bottom_accel2]
  #   type = PresetAcceleration
  #   variable = disp_y
  #   velocity = vel_y
  #   acceleration = accel_y
  #   beta = 0.25
  #   function = accel_bottom
  #   boundary = 'IsoBottom'
  # [../]
  [./disp_x2]
    type = DirichletBC
    variable = disp_y
    boundary = 'IsoBottom'
    value = 0.0
  [../]
  [./disp_x3]
    type = DirichletBC
    variable = disp_z
    boundary = 'IsoBottom'
    value = 0.0
  [../]
  [./free]
    type = FluidFreeSurfaceBC
    variable = p
    boundary = 'Fluid_top'
    alpha = '0.1'
  []
  [./disp_top]
    type = DirichletBC
    variable = disp_y
    boundary = 'Top'
    value = 0.0
  [../]
[]

[Functions]
  [force_z]
    type = PiecewiseLinear
    x='0.0 0.1 150.0'
    # Fluid volume is 87.17715 m^3
    # RV volume is 2.00486 m^3
    # RV slab volume is 33.13401 m^3
    y='0.0 -9.075e-5 -9.075e-5'
  []
  [ormsby]
    type = OrmsbyWavelet
    f1 = 0.0
    f2 = 0.1
    f3 = 50.0
    f4 = 55.0
    ts = 0.25
    scale_factor = 4.905
  []
  [accel_bottom]
    type = PiecewiseLinear
    data_file = Sine_0_5Hz0.csv
    format = 'columns'
    scale_factor = 0.1 # 4.905
  []
[]

[Materials]
  [./density]
    type = GenericConstantMaterial
    prop_names = inv_co_sq
    prop_values = 4.65e-7
    block = 2
  [../]
  [./density0]
    type = GenericConstantMaterial
    block = '1 3 4'
    prop_names = density
    prop_values = 7.85e-6
  [../]
  [./elasticity_base]
    type = ComputeIsotropicElasticityTensor
    youngs_modulus = 2e2
    poissons_ratio = 0.27
    block = '1'
  [../]
  [./elasticity_top]
    type = ComputeIsotropicElasticityTensor
    youngs_modulus = 2e2
    poissons_ratio = 0.27
    block = '3'
  [../]
  [./strain]
    type = ComputeFiniteStrain
    block = '1 3'
    displacements = 'disp_x disp_y disp_z'
  [../]
  [./stress]
    type =  ComputeFiniteStrainElasticStress
    block = '1 3'
  [../]
  [./deformation]
    type = ComputeIsolatorDeformation
    sd_ratio = 0.5
    y_orientation = '1.0 0.0 0.0'
    displacements = 'disp_x disp_y disp_z'
    rotations = 'rot_x rot_y rot_z'
    velocities = 'vel_x vel_y vel_z'
    block = 4
  [../]
  [./elasticity]
    type = ComputeFPIsolatorElasticity
    mu_ref = 0.06
    p_ref = 0.05 # 50e6
    block = 4
    diffusivity = 4.4e-6
    conductivity = 18
    a = 100
    r_eff = 0.25
    r_contact = 0.2
    uy = 0.001
    unit = 9
    beta = 0.3025
    gamma = 0.6
    # beta = 0.3025
    # gamma = 0.6
    pressure_dependent = false # true
    temperature_dependent = false # true
    velocity_dependent = false # true
    k_x = 78.53 # 7.853e10
    k_xx = 0.62282 # 622820743.6
    k_yy = 0.3114 # 311410371.8
    k_zz = 0.3114 # 311410371.8
  [../]
  [./elasticity_beam_rigid]
    type = ComputeElasticityBeam
    youngs_modulus = 2e4
    poissons_ratio = 0.27
    shear_coefficient = 0.85
    block = '5'
  [../]
  [./stress_beam]
    type = ComputeBeamResultants
    block = '5'
  [../]
[]

[Preconditioning]
  [./andy]
    type = SMP
    full = true
  [../]
[]

[Executioner]
  type = Transient
  petsc_options = '-ksp_snes_ew'
  petsc_options_iname = '-pc_type -pc_factor_mat_solver_package'
  petsc_options_value = 'lu       superlu_dist'
  solve_type = 'NEWTON'
  nl_rel_tol = 1e-12 # 1e-10
  nl_abs_tol = 1e-12 # 1e-8
  # l_tol = 1e-2
  # start_time = 2.0
  dt = 0.0025 # 0.0025
  end_time = 0.5 # 0.5
  timestep_tolerance = 1e-6
  automatic_scaling = true
  [TimeIntegrator]
    type = NewmarkBeta
    beta = 0.3025
    gamma = 0.6
  []
[]

[Postprocessors]
  [./Iso1_Bot_ReacX]
    # type = NodalVariableValue
    # nodeid = 3982
    # variable = reaction_x
    type = PointValue
    point = '0.0 -3.1 4.66125'
    variable = reaction_x
  [../]
  [./Iso1_Bot_DispX]
    # type = NodalVariableValue
    # nodeid = 3982
    # variable = disp_x
    type = PointValue
    point = '0.0 -3.1 4.66125'
    variable = disp_x
  [../]
  [./Iso1_Top_ReacX]
    # type = NodalVariableValue
    # nodeid = 3334
    # variable = reaction_x
    type = PointValue
    point = '0.0 -3.1 4.98125'
    variable = reaction_x
  [../]
  [./Iso1_Top_DispX]
    # type = NodalVariableValue
    # nodeid = 3334
    # variable = disp_x
    type = PointValue
    point = '0.0 -3.1 4.98125'
    variable = disp_x
  [../]
  [./AccelInp]
    # type = NodalVariableValue
    # nodeid = 3984
    # variable = accel_x
    type = PointValue
    point = '0.0 -3.1 4.66125'
    variable = accel_x
  [../]
  [./AccelTop]
    type = PointValue
    point = '0.0 0.0 5.13125'
    variable = accel_x
  [../]
  [./Iso1_Top_AccelX]
    type = PointValue
    point = '0.0 -3.1 4.98125'
    variable = accel_x
  [../]
  [./Iso1_Top_AccelY]
    type = PointValue
    point = '0.0 -3.1 4.98125'
    variable = accel_y
  [../]
  [./Iso1_Top_AccelZ]
    type = PointValue
    point = '0.0 -3.1 4.98125'
    variable = accel_z
  [../]
[]

[VectorPostprocessors]
  [./accel_hist_x]
    type = ResponseHistoryBuilder
    variables = 'accel_x'
    nodes = '6621 6620 6622 6623 6372 6367 6243 6082'
    outputs = out1
    block = 4
  [../]
  [./accel_spec_x]
    type = ResponseSpectraCalculator
    vectorpostprocessor = accel_hist_x
    regularize_dt = 0.001
    damping_ratio = 0.05
    start_frequency = 0.1
    end_frequency = 1000
    outputs = out1
  [../]

  [./accel_hist_y]
    type = ResponseHistoryBuilder
    variables = 'accel_y'
    nodes = '6621 6620 6622 6623 6372 6367 6243 6082'
    outputs = out1
    block = 4
  [../]
  [./accel_spec_y]
    type = ResponseSpectraCalculator
    vectorpostprocessor = accel_hist_y
    regularize_dt = 0.001
    damping_ratio = 0.05
    start_frequency = 0.1
    end_frequency = 1000
    outputs = out1
  [../]

  [./accel_hist_z]
    type = ResponseHistoryBuilder
    variables = 'accel_z'
    nodes = '6621 6620 6622 6623 6372 6367 6243 6082'
    outputs = out1
    block = 4
  [../]
  [./accel_spec_z]
    type = ResponseSpectraCalculator
    vectorpostprocessor = accel_hist_z
    regularize_dt = 0.001
    damping_ratio = 0.05
    start_frequency = 0.1
    end_frequency = 1000
    outputs = out1
  [../]
[]

[Outputs]
  exodus = true
  perf_graph = true
  csv = true
  # file_base = Ex_Isolator_verify
  [./out1]
    type = CSV
    execute_on = 'final'
    # file_base = Isolator_verify
  [../]
[]
