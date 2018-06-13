# Test for lead rubber isolator
# Axial direction

[Mesh]
  type = GeneratedMesh
  xmin = 0
  xmax = 1
  nx = 1
  dim = 1
  displacements = 'disp_x disp_y disp_z'
[]

[Variables]
  [./disp_x]
    order = FIRST
    family = LAGRANGE
  [../]
  [./disp_y]
    order = FIRST
    family = LAGRANGE
  [../]
  [./disp_z]
    order = FIRST
    family = LAGRANGE
  [../]
  [./rot_x]
    order = FIRST
    family = LAGRANGE
  [../]
  [./rot_y]
    order = FIRST
    family = LAGRANGE
  [../]
  [./rot_z]
    order = FIRST
    family = LAGRANGE
  [../]
[]

[AuxVariables]
  [./vel_x]
  order = FIRST
  family = LAGRANGE
  [../]
  [./vel_y]
  order = FIRST
  family = LAGRANGE
  [../]
  [./vel_z]
  order = FIRST
  family = LAGRANGE
  [../]
  [./accel_x]
  order = FIRST
  family = LAGRANGE
  [../]
  [./accel_y]
  order = FIRST
  family = LAGRANGE
  [../]
  [./accel_z]
  order = FIRST
  family = LAGRANGE
  [../]
  [./rot_vel_x]
  order = FIRST
  family = LAGRANGE
  [../]
  [./rot_vel_y]
  order = FIRST
  family = LAGRANGE
  [../]
  [./rot_vel_z]
  order = FIRST
  family = LAGRANGE
  [../]
  [./rot_accel_x]
  order = FIRST
  family = LAGRANGE
  [../]
  [./rot_accel_y]
  order = FIRST
  family = LAGRANGE
  [../]
  [./rot_accel_z]
  order = FIRST
  family = LAGRANGE
  [../]
  [./reaction_x]
  [../]
  [./reaction_y]
  [../]
  [./reaction_z]
  [../]
  [./reaction_xx]
  [../]
  [./reaction_yy]
  [../]
  [./reaction_zz]
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
  [./rot_accel_x]
    type = NewmarkAccelAux
    variable = rot_accel_x
    displacement = rot_x
    velocity = rot_vel_x
    beta = 0.25
    execute_on = timestep_end
  [../]
  [./rot_vel_x]
    type = NewmarkVelAux
    variable = rot_vel_x
    acceleration = rot_accel_x
    gamma = 0.5
    execute_on = timestep_end
  [../]
  [./rot_accel_y]
    type = NewmarkAccelAux
    variable = rot_accel_y
    displacement = rot_y
    velocity = rot_vel_y
    beta = 0.25
    execute_on = timestep_end
  [../]
  [./rot_vel_y]
    type = NewmarkVelAux
    variable = rot_vel_y
    acceleration = rot_accel_y
    gamma = 0.5
    execute_on = timestep_end
  [../]
  [./rot_accel_z]
    type = NewmarkAccelAux
    variable = rot_accel_z
    displacement = rot_z
    velocity = rot_vel_z
    beta = 0.25
    execute_on = timestep_end
  [../]
  [./rot_vel_z]
    type = NewmarkVelAux
    variable = rot_vel_z
    acceleration = rot_accel_z
    gamma = 0.5
    execute_on = timestep_end
  [../]
[]

[BCs]
  [./fixx0]
    type = DirichletBC
    variable = disp_x
    boundary = left
    value = 0.0
  [../]
  [./fixy0]
    type = DirichletBC
    variable = disp_y
    boundary = left
    value = 0.0
  [../]
  [./fixz0]
    type = DirichletBC
    variable = disp_z
    boundary = left
    value = 0.0
  [../]
  [./fixrx0]
    type = DirichletBC
    variable = rot_x
    boundary = left
    value = 0.0
  [../]
  [./fixry0]
    type = DirichletBC
    variable = rot_y
    boundary = left
    value = 0.0
  [../]
  [./fixrz0]
    type = DirichletBC
    variable = rot_z
    boundary = left
    value = 0.0
  [../]
  [./disp_x_1]
    type = PresetDisplacement
    boundary = right
    function = history
    variable = disp_x
    beta = 0.25
    acceleration = accel_x
    velocity = vel_x
  [../]
[]

# [NodalKernels]
#   [./force_x]
#     type = UserForcingFunctionNodalKernel
#     variable = disp_x
#     boundary = right
#     function = force_x
#   [../]
#   [./force_y]
#     type = UserForcingFunctionNodalKernel
#     variable = disp_y
#     boundary = right
#     function = force_y
#   [../]
#   [./force_z]
#     type = UserForcingFunctionNodalKernel
#     variable = disp_z
#     boundary = right
#     function = force_z
#   [../]
#   [./moment_x]
#     type = UserForcingFunctionNodalKernel
#     variable = rot_x
#     boundary = right
#     function = moment_x
#   [../]
#   [./moment_y]
#     type = UserForcingFunctionNodalKernel
#     variable = rot_y
#     boundary = right
#     function = moment_y
#   [../]
#   [./moment_z]
#     type = UserForcingFunctionNodalKernel
#     variable = rot_z
#     boundary = right
#     function = moment_z
#   [../]
# []

[Functions]
  [./force_x]
    type = PiecewiseLinear
    x = '0.0 1.0' # time
    y = '0.0 1.0'  # force
  [../]
  [./force_y]
    type = PiecewiseLinear
    x = '0.0 1.0' # time
    y = '0.0 2.0'  # force
  [../]
  [./force_z]
    type = PiecewiseLinear
    x = '0.0 1.0' # time
    y = '0.0 3.0'  # force
  [../]
  [./moment_x]
    type = PiecewiseLinear
    x = '0.0 1.0' # time
    y = '0.0 10.0'  # moment
  [../]
  [./moment_y]
    type = PiecewiseLinear
    x = '0.0 1.0' # time
    y = '0.0 20.0'  # moment
  [../]
  [./moment_z]
    type = PiecewiseLinear
    x = '0.0 1.0' # time
    y = '0.0 30.0'  # moment
  [../]
  [./history]
    type = PiecewiseLinear
    data_file = disp_axial.csv
    format = columns
    # x = '0.0 1.0 2.0 3.0 4.0 5.0 6.0 7.0 8.0 9.0 10.0 11.0 12.0'
    # y = '0.0 0.0 0.0 -0.1 0.0 0.2 0.0 -0.3 0.0 0.4 0.0 -0.5 0.0'
  [../]
[]

[Preconditioning]
  [./smp]
    type = SMP
    full = true
  [../]
[]

[Executioner]
  type = Transient
  solve_type = NEWTON
  line_search = none
  nl_rel_tol = 1e-8
  nl_abs_tol = 1e-8
  start_time = -0.015
  end_time = 10
  dt = 0.005
  dtmin = 0.001
  # num_steps = 2
  timestep_tolerance = 1e-6
[]

[Kernels]
  [./lr_disp_x]
    type = StressDivergenceIsolator
    block = '0'
    displacements = 'disp_x disp_y disp_z'
    rotations = 'rot_x rot_y rot_z'
    component = 0
    variable = disp_x
    save_in = reaction_x
  [../]
  [./lr_disp_y]
    type = StressDivergenceIsolator
    block = '0'
    displacements = 'disp_x disp_y disp_z'
    rotations = 'rot_x rot_y rot_z'
    component = 1
    variable = disp_y
    save_in = reaction_y
  [../]
  [./lr_disp_z]
    type = StressDivergenceIsolator
    block = '0'
    displacements = 'disp_x disp_y disp_z'
    rotations = 'rot_x rot_y rot_z'
    component = 2
    variable = disp_z
    save_in = reaction_z
  [../]
  [./lr_rot_x]
    type = StressDivergenceIsolator
    block = '0'
    displacements = 'disp_x disp_y disp_z'
    rotations = 'rot_x rot_y rot_z'
    component = 3
    variable = rot_x
    save_in = reaction_xx
  [../]
  [./lr_rot_y]
    type = StressDivergenceIsolator
    block = '0'
    displacements = 'disp_x disp_y disp_z'
    rotations = 'rot_x rot_y rot_z'
    component = 4
    variable = rot_y
    save_in = reaction_yy
  [../]
  [./lr_rot_z]
    type = StressDivergenceIsolator
    block = '0'
    displacements = 'disp_x disp_y disp_z'
    rotations = 'rot_x rot_y rot_z'
    component = 5
    variable = rot_z
    save_in = reaction_zz
  [../]
[]

[Materials]
  [./deformation]
    type = ComputeIsolatorDeformation
    block = 0
    sd_ratio = 0.5
    y_orientation = '0.0 1.0 0.0'
    displacements = 'disp_x disp_y disp_z'
    rotations = 'rot_x rot_y rot_z'
    velocities = 'vel_x vel_y vel_z'
  [../]
  [./elasticity]
    type = ComputeLRIsolatorElasticity
    block = 0
    fy = 207160
    alpha = 0.03812
    G_rubber = 8.7e6
    K_rubber = 2e9
    D1 = 0.1397
    D2 = 0.508
    ts = 0.00476
    tr = 0.009525
    n = 16
    tc = 0.0127
    kc = 20
    phi_m = 0.75
    ac = 1
    cd = 128000
    k_steel = 50
    rho_lead = 11200
    c_lead = 130
    cavitation = true
    horizontal_stiffness_variation = true
    vertical_stiffness_variation = true
    strength_degradation = true
    buckling_load_variation = true
  [../]
[]

[Postprocessors]
  [./disp_x]
    type = NodalVariableValue
    nodeid = 1
    variable = disp_x
  [../]
  [./reaction_x]
    type = NodalSum
    variable = reaction_x
    boundary = left
  [../]
  [./disp_y]
    type = NodalVariableValue
    nodeid = 1
    variable = disp_y
  [../]
  [./reaction_y]
    type = NodalSum
    variable = reaction_y
    boundary = left
  [../]
  [./disp_z]
    type = NodalVariableValue
    nodeid = 1
    variable = disp_z
  [../]
  [./reaction_z]
    type = NodalSum
    variable = reaction_z
    boundary = left
  [../]
  [./rot_x]
    type = NodalVariableValue
    nodeid = 1
    variable = rot_x
  [../]
  [./reaction_xx]
    type = NodalSum
    variable = reaction_xx
    boundary = left
  [../]
  [./rot_y]
    type = NodalVariableValue
    nodeid = 1
    variable = rot_y
  [../]
  [./reaction_yy]
    type = NodalSum
    variable = reaction_yy
    boundary = left
  [../]
  [./rot_z]
    type = NodalVariableValue
    nodeid = 1
    variable = rot_z
  [../]
  [./reaction_zz]
    type = NodalSum
    variable = reaction_zz
    boundary = left
  [../]
[]

[Outputs]
  csv = true
  exodus = true
  print_perf_log = true
[]
