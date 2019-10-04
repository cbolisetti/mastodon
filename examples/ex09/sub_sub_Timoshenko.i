[Mesh]
  type = GeneratedMesh
  dim = 1
  nx = 10
  xmin = 0.0
  xmax = 4.0
  displacements = 'disp_x disp_y disp_z'
[]

[AuxVariables]
  [./youngs_modulus1]
    order = CONSTANT
    family = MONOMIAL
  [../]
[]

[BCs]
  [./fixx1]
    type = DirichletBC
    variable = disp_x
    boundary = left
    value = 0.0
  [../]
  [./fixy1]
    type = DirichletBC
    variable = disp_y
    boundary = left
    value = 0.0
  [../]
  [./fixz1]
    type = DirichletBC
    variable = disp_z
    boundary = left
    value = 0.0
  [../]
  [./fixr1]
    type = DirichletBC
    variable = rot_x
    boundary = left
    value = 0.0
  [../]
  [./fixr2]
    type = DirichletBC
    variable = rot_y
    boundary = left
    value = 0.0
  [../]
  [./fixr3]
    type = DirichletBC
    variable = rot_z
    boundary = left
    value = 0.0
  [../]
  [./accel_right]
    type = PresetAcceleration
    boundary = 'right'
    function = accel_y
    variable = 'disp_y'
    beta = 0.25
    acceleration = 'accel_y'
    velocity = 'vel_y'
  [../]
[]

# [NodalKernels]
#   [./force_y2]
#     type = UserForcingFunctionNodalKernel
#     variable = disp_y
#     boundary = right
#     function = force
#   [../]
# []

[AuxKernels]
  [./accel_x]
    type = NewmarkAccelAux
    variable = accel_x
    displacement = disp_x
    velocity = vel_x
    beta = 0.25
    execute_on = 'timestep_end'
  [../]
  [./vel_x]
    type = NewmarkVelAux
    variable = vel_x
    acceleration = accel_x
    gamma = 0.5
    execute_on = 'timestep_end'
  [../]
  [./accel_y]
    type = NewmarkAccelAux
    variable = accel_y
    displacement = disp_y
    velocity = vel_y
    beta = 0.25
    execute_on = 'timestep_end'
  [../]
  [./vel_y]
    type = NewmarkVelAux
    variable = vel_y
    acceleration = accel_y
    gamma = 0.5
    execute_on = 'timestep_end'
  [../]
  [./accel_z]
    type = NewmarkAccelAux
    variable = accel_z
    displacement = disp_z
    velocity = vel_z
    beta = 0.25
    execute_on = 'timestep_end'
  [../]
  [./vel_z]
    type = NewmarkVelAux
    variable = vel_z
    acceleration = accel_z
    gamma = 0.5
    execute_on = 'timestep_end'
  [../]
  [./youngs_modulus1]
    type = ConstantAux
    variable = youngs_modulus1
    value = 1e04
  [../]
[]

[Functions]
  # [./force]
  #   type = PiecewiseLinear
  #   x = '0.0 0.1 0.2 10.0'
  #   y = '0.0 1e-2  0.0  0.0'
  # [../]
  # [./force]
  #   type = ConstantFunction
  #   value = 0.01
  # [../]
  [./accel_y]
    type = PiecewiseLinear
    data_file = 'GM_00.csv'
    format = columns
    scale_factor = 1.0
    xy_in_file_only = false
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
  solve_type = 'PJFNK'
  petsc_options = '-snes_ksp_ew'
  petsc_options_iname = '-ksp_gmres_restart -pc_type -pc_hypre_type -pc_hypre_boomeramg_max_iter'
  petsc_options_value = '201                hypre    boomeramg      4'
  end_time = 35.00
  dt = 0.01
  dtmin = 0.01
  nl_abs_tol = 1e-3
    nl_rel_tol = 1e-03
  l_tol = 1e-3
  l_max_its = 20
  timestep_tolerance = 1e-3
[]

[Modules/TensorMechanics/LineElementMaster]
  [./all]
    add_variables = true
    displacements = 'disp_x disp_y disp_z'
    rotations = 'rot_x rot_y rot_z'

    # Geometry parameters
    area = 0.01
    Iy = 1e-4
    Iz = 1e-4
    y_orientation = '0.0 1.0 0.0'

    # dynamic simulation using consistent mass/inertia matrix
    dynamic_nodal_translational_inertia = true
    nodal_mass = 18.99772

    dynamic_nodal_rotational_inertia = true
    nodal_Ixx = 2e-1
    nodal_Iyy = 1e-1
    nodal_Izz = 1e-1

    velocities = 'vel_x vel_y vel_z'
    accelerations = 'accel_x accel_y accel_z'
    rotational_velocities = 'rot_vel_x rot_vel_y rot_vel_z'
    rotational_accelerations = 'rot_accel_x rot_accel_y rot_accel_z'

    beta = 0.25 # Newmark time integration parameter
    gamma = 0.5 # Newmark time integration parameter

    boundary = right # Node set where nodal mass and nodal inertia are applied

    # parameters for 5% Rayleigh damping
    zeta = 0.0005438894818 # stiffness proportional damping
    eta = 3.26645357034 # Mass proportional Rayleigh damping
  [../]
[]

[Materials]
  [./elasticity]
    type = ComputeElasticityBeam
    youngs_modulus = youngs_modulus1
    poissons_ratio = 0.3
    shear_coefficient = 1.0
    block = 0
  [../]
  [./stress]
    type = ComputeBeamResultants
    block = 0
  [../]
[]

[Controls]
  [./stochastic]
    type = SamplerReceiver
  [../]
[]

[Postprocessors]
  [./disp_x]
    type = PointValue
    point = '4.0 0.0 0.0'
    variable = disp_x
  [../]
  [./disp_y]
    type = PointValue
    point = '4.0 0.0 0.0'
    variable = disp_y
  [../]
  [./vel_y]
    type = PointValue
    point = '4.0 0.0 0.0'
    variable = vel_y
  [../]
  [./accel_y]
    type = PointValue
    point = '4.0 0.0 0.0'
    variable = accel_y
  [../]
[]

[Outputs]
  file_base = 'Timoshenko_random'
  exodus = true
  csv = true
  perf_graph = true
[]
