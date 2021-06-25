[Mesh]
  [mesh_gen]
    type = FileMeshGenerator
    file = RV_citadel_and_isolators_nodesets.e
  []
  [translate]
    type = TransformGenerator
    input = mesh_gen
    transform = TRANSLATE
    vector_value = '15 0 0'
  []
  [slab_surface]
    type = BoundingBoxNodeSetGenerator
    input = translate
    bottom_left = '-4.0 -4.0 8.1'
    top_right = '4.0 4.0 8.5'
    new_boundary = 'rv_slab_top'
  []
  displacements = 'disp_x disp_y disp_z'
[]

[Variables]
  [disp_x]
    block = 'RV_housing RV_slab isolators rigid_beams extra_rigid_beams'
    order = FIRST
    family = LAGRANGE
  []
  [disp_y]
    block = 'RV_housing RV_slab isolators rigid_beams extra_rigid_beams'
    order = FIRST
    family = LAGRANGE
  []
  [disp_z]
    block = 'RV_housing RV_slab isolators rigid_beams extra_rigid_beams'
    order = FIRST
    family = LAGRANGE
  []
  [rot_x]
    block = 'isolators rigid_beams extra_rigid_beams'
    order = FIRST
    family = LAGRANGE
  []
  [rot_y]
    block = 'isolators rigid_beams extra_rigid_beams'
    order = FIRST
    family = LAGRANGE
  []
  [rot_z]
    block = 'isolators rigid_beams extra_rigid_beams'
    order = FIRST
    family = LAGRANGE
  []
[]

[AuxVariables]
  [vel_x]
    block = 'RV_housing RV_slab isolators rigid_beams extra_rigid_beams'
  []
  [accel_x]
    block = 'RV_housing RV_slab isolators rigid_beams extra_rigid_beams'
  []
  [vel_y]
    block = 'RV_housing RV_slab isolators rigid_beams extra_rigid_beams'
  []
  [accel_y]
    block = 'RV_housing RV_slab isolators rigid_beams extra_rigid_beams'
  []
  [vel_z]
    block = 'RV_housing RV_slab isolators rigid_beams extra_rigid_beams'
  []
  [accel_z]
    block = 'RV_housing RV_slab isolators rigid_beams extra_rigid_beams'
  []
  [rot_vel_x]
    block = 'isolators rigid_beams extra_rigid_beams'
    order = FIRST
    family = LAGRANGE
  []
  [rot_vel_y]
    block = 'isolators rigid_beams extra_rigid_beams'
    order = FIRST
    family = LAGRANGE
  []
  [rot_vel_z]
    block = 'isolators rigid_beams extra_rigid_beams'
    order = FIRST
    family = LAGRANGE
  []
  [rot_accel_x]
    block = 'isolators rigid_beams extra_rigid_beams'
    order = FIRST
    family = LAGRANGE
  []
  [rot_accel_y]
    block = 'isolators rigid_beams extra_rigid_beams'
    order = FIRST
    family = LAGRANGE
  []
  [rot_accel_z]
    block = 'isolators rigid_beams extra_rigid_beams'
    order = FIRST
    family = LAGRANGE
  []
  [Fb_x]
    block = 'isolators'
    order = CONSTANT
    family = MONOMIAL
  []
  [Fb_y]
    block = 'isolators'
    order = CONSTANT
    family = MONOMIAL
  []
  [Fb_z]
    block = 'isolators'
    order = CONSTANT
    family = MONOMIAL
  []
  [Defb_x]
    block = 'isolators'
    order = CONSTANT
    family = MONOMIAL
  []
  [Velb_x]
    block = 'isolators'
    order = CONSTANT
    family = MONOMIAL
  []
  [Defb_y]
    block = 'isolators'
    order = CONSTANT
    family = MONOMIAL
  []
  [Defb_z]
    block = 'isolators'
    order = CONSTANT
    family = MONOMIAL
  []
[]

[Modules/TensorMechanics/LineElementMaster]
  displacements = 'disp_x disp_y disp_z'
  rotations = 'rot_x rot_y rot_z'

  # use_displaced_mesh = true
  # dynamic simulation using consistent mass/inertia matrix
  # dynamic_nodal_translational_inertia = true

  velocities = 'vel_x vel_y vel_z'
  accelerations = 'accel_x accel_y accel_z'
  rotational_velocities = 'rot_vel_x rot_vel_y rot_vel_z'
  rotational_accelerations = 'rot_accel_x rot_accel_y rot_accel_z'

  eta = 0.4
  zeta = 0.008

  beta = 0.3025
  gamma = 0.6
  alpha = -0.1

  [block_16]
    block = 'rigid_beams extra_rigid_beams'
    area = 150
    Iy = 24166.729
    Iz = 24166.729
    y_orientation = '0.0 0.0 1.0'

    # nodal_mass = 1e-10
    # boundary = 'bottom_isolator'
  []
[]

[Kernels]
  [DynamicTensorMechanics]
    zeta = 0.002
    # alpha = -0.1
    displacements = 'disp_x disp_y disp_z'
    block = 'RV_housing RV_slab'
    static_initialization = true
  []
  [inertia_x]
    type = InertialForce
    block = 'RV_housing RV_slab'
    variable = disp_x
    eta = 0.038
    beta = 0.3025
    gamma = 0.6
    alpha = -0.1
    velocity = vel_x
    acceleration = accel_x
  []
  [inertia_y]
    type = InertialForce
    block = 'RV_housing RV_slab'
    variable = disp_y
    eta = 0.038
    beta = 0.3025
    gamma = 0.6
    alpha = -0.1
    velocity = vel_y
    acceleration = accel_x
  []
  [inertia_z]
    type = InertialForce
    block = 'RV_housing RV_slab'
    variable = disp_z
    eta = 0.038
    beta = 0.3025
    gamma = 0.6
    alpha = -0.1
    velocity = vel_z
    acceleration = accel_z
  []
  [gravity]
    type = Gravity
    variable = disp_z
    value = -9.81
    block = 'RV_slab RV_housing'
  []
  [lr_disp_x]
    block = 'isolators'
    type = StressDivergenceIsolator
    displacements = 'disp_x disp_y disp_z'
    rotations = 'rot_x rot_y rot_z'
    component = 0
    variable = disp_x
  []
  [lr_disp_y]
    block = 'isolators'
    type = StressDivergenceIsolator
    displacements = 'disp_x disp_y disp_z'
    rotations = 'rot_x rot_y rot_z'
    component = 1
    variable = disp_y
  []
  [lr_disp_z]
    block = 'isolators'
    type = StressDivergenceIsolator
    displacements = 'disp_x disp_y disp_z'
    rotations = 'rot_x rot_y rot_z'
    component = 2
    variable = disp_z
  []
  [lr_rot_x]
    block = 'isolators'
    type = StressDivergenceIsolator
    displacements = 'disp_x disp_y disp_z'
    rotations = 'rot_x rot_y rot_z'
    component = 3
    variable = rot_x
  []
  [lr_rot_y]
    block = 'isolators'
    type = StressDivergenceIsolator
    displacements = 'disp_x disp_y disp_z'
    rotations = 'rot_x rot_y rot_z'
    component = 4
    variable = rot_y
  []
  [lr_rot_z]
    block = 'isolators'
    type = StressDivergenceIsolator
    displacements = 'disp_x disp_y disp_z'
    rotations = 'rot_x rot_y rot_z'
    component = 5
    variable = rot_z
  []
[]

[NodalKernels]
  # [force_z]
  #   type = UserForcingFunctionNodalKernel
  #   variable = disp_z
  #   boundary = 'rv_slab_top'
  #   function = force_z
  # []
  # [force_x]
  #   type = UserForcingFunctionNodalKernel
  #   variable = disp_x
  #   boundary = 'rv_slab_top'
  #   function = sawtooth
  # []
[]

[AuxKernels]
  [Fb_x]
    type = MaterialRealCMMAux
    property = basic_forces
    row = 0
    column = 0
    variable = Fb_x
    block = 'isolators'
  []
  [Fb_y]
    type = MaterialRealCMMAux
    property = basic_forces
    row = 1
    column = 0
    variable = Fb_y
    block = 'isolators'
  []
  [Fb_z]
    type = MaterialRealCMMAux
    property = basic_forces
    row = 2
    column = 0
    variable = Fb_z
    block = 'isolators'
  []
  [Defb_x]
    type = MaterialRealCMMAux
    property = deformations
    row = 0
    column = 0
    variable = Defb_x
    block = 'isolators'
  []
  [Velb_x]
    type = MaterialRealCMMAux
    property = deformation_rates
    row = 0
    column = 0
    variable = Velb_x
    block = 'isolators'
  []
  [Defb_y]
    type = MaterialRealCMMAux
    property = deformations
    row = 1
    column = 0
    variable = Defb_y
    block = 'isolators'
  []
  [Defb_z]
    type = MaterialRealCMMAux
    property = deformations
    row = 2
    column = 0
    variable = Defb_z
    block = 'isolators'
  []
  [accel_x]
    # block = 'RV_housing RV_slab'
    type = NewmarkAccelAux
    variable = accel_x
    displacement = disp_x
    velocity = vel_x
    beta = 0.25
  []
  [vel_x]
    # block = 'RV_housing RV_slab'
    type = NewmarkVelAux
    variable = vel_x
    acceleration = accel_x
    gamma = 0.5
  []
  [accel_y]
    # block = 'RV_housing RV_slab'
    type = NewmarkAccelAux
    variable = accel_y
    displacement = disp_y
    velocity = vel_y
    beta = 0.25
  []
  [vel_y]
    # block = 'RV_housing RV_slab'
    type = NewmarkVelAux
    variable = vel_y
    acceleration = accel_y
    gamma = 0.5
  []
  [accel_z]
    # block = 'RV_housing RV_slab'
    type = NewmarkAccelAux
    variable = accel_z
    displacement = disp_z
    velocity = vel_z
    beta = 0.25
  []
  [vel_z]
    # block = 'RV_housing RV_slab'
    type = NewmarkVelAux
    variable = vel_z
    acceleration = accel_z
    gamma = 0.5
  []
  [rot_accel_x]
    # block = 'RV_housing RV_slab'
    type = NewmarkAccelAux
    variable = rot_accel_x
    displacement = rot_x
    velocity = rot_vel_x
    beta = 0.25
  []
  [rot_vel_x]
    # block = 'RV_housing RV_slab'
    type = NewmarkVelAux
    variable = rot_vel_x
    acceleration = rot_accel_x
    gamma = 0.5
  []
  [rot_accel_y]
    # block = 'RV_housing RV_slab'
    type = NewmarkAccelAux
    variable = rot_accel_y
    displacement = rot_y
    velocity = rot_vel_y
    beta = 0.25
  []
  [rot_vel_y]
    # block = 'RV_housing RV_slab'
    type = NewmarkVelAux
    variable = rot_vel_y
    acceleration = rot_accel_y
    gamma = 0.5
  []
  [rot_accel_z]
    # block = 'RV_housing RV_slab'
    type = NewmarkAccelAux
    variable = rot_accel_z
    displacement = rot_z
    velocity = rot_vel_z
    beta = 0.25
  []
  [rot_vel_z]
    # block = 'RV_housing RV_slab'
    type = NewmarkVelAux
    variable = rot_vel_z
    acceleration = rot_accel_z
    gamma = 0.5
  []
[]

[Functions]
  [force_z]
    type = PiecewiseLinear
    x='-1.0 -0.1 1000'
    # Fluid volume is 87.17715 m^3
    # RV volume is 2.00486 m^3
    # RV slab volume is 33.13401 m^3
    y='0.0 -5e-7 -5e-7'
  []
  [ormsby]
    type = OrmsbyWavelet
    f1 = 2.0
    f2 = 3.0
    f3 = 9.0
    f4 = 10.0
    ts = 0.25
    scale_factor = 4.905
  []
  [step_func]
    type = PiecewiseLinear
    x = '0 0.02 0.021 1000'
    y = '0    0      1    1'
  []
  # [sin_func]
  #   type = ParsedFunction
  #   # x = '0 1.5 1.75 2.0 2.25 2.50 2.75'
  #   # y = '0 0   0.1  0   -0.1    0  0.1'
  #   value = 9.81*sin(10*pi*(t+1)^1.5)
  #   # value = 0
  # []
  [x_excitation]
    type = CompositeFunction
    functions = 'step_func ormsby'
  []
  [ramp_z]
    type = PiecewiseLinear
    x = '0.0 0.1  1.0 10000'
    y = '0.0 0.0 9.81  9.81'
  []
  # [sawtooth]
  #   type = PiecewiseLinear
  #   x = '0  0.2  0.4  0.6  0.8  1.0   1.2'
  #   y = '0  0.0 -0.1 0.15 -0.2 0.25 -0.25'
  #   # scale_factor = 1e-7
  # []
[]

[Materials]
  [elasticity_1]
    type = ComputeIsotropicElasticityTensor
    block = 'RV_housing'
    youngs_modulus = 24.8
    poissons_ratio = 0.2
  []
  [strain_1]
    type = ComputeSmallStrain
    displacements = 'disp_x disp_y disp_z'
    block = 'RV_housing'
  []
  [stress_1]
    type = ComputeLinearElasticStress
    block = 'RV_housing'
  []
  [den_1]
    type = GenericConstantMaterial
    block = 'RV_housing'
    prop_names = density
    prop_values = 2.4e-6 #kg/mm3 #concrete
  []
  [elasticity_2]
    type = ComputeIsotropicElasticityTensor
    block = 'RV_slab'
    youngs_modulus = 170
    poissons_ratio = 0.3
  []
  [strain_2]
    type = ComputeSmallStrain
    displacements = 'disp_x disp_y disp_z'
    block = 'RV_slab'
  []
  [stress_2]
    type = ComputeLinearElasticStress
    block = 'RV_slab'
  []
  [den_rv]
    type = GenericConstantMaterial
    block = 'RV_slab'
    prop_names = density
    # prop_values = 7.85e-6 #kg/mm3
    # prop_values = 0.00001095551 #(4*0.0008902575/9.81)/33.134
    prop_values = 1.1e-5 #kg/mm3 #includes RV and internals (300T)
  []
  [elasticity_beam_rigid]
    type = ComputeElasticityBeam
    youngs_modulus = 2e4
    poissons_ratio = 0.27
    shear_coefficient = 0.85
    block = 'rigid_beams extra_rigid_beams'
  []
  [stress_beam]
    type = ComputeBeamResultants
    block = 'rigid_beams extra_rigid_beams'
  []
  [deformation]
    type = ComputeIsolatorDeformation
    sd_ratio = 0.5
    y_orientation = '1.0 0.0 0.0'
    displacements = 'disp_x disp_y disp_z'
    rotations = 'rot_x rot_y rot_z'
    velocities = 'vel_x vel_y vel_z'
    block = 'isolators'
  []
  [elasticity]
    type = ComputeFPIsolatorElasticity
    mu_ref = 0.06
    p_ref = 0.05 # 50e6
    block = 'isolators'
    diffusivity = 4.4e-6
    conductivity = 18
    a = 100
    r_eff = 0.25 #1Hz sliding frequency
    r_contact = 0.2
    uy = 0.001
    unit = 4
    # beta = 0.275625
    # gamma = 0.55
    beta = 0.3025
    gamma = 0.6
    pressure_dependent = false
    temperature_dependent = false
    velocity_dependent = false
    k_x = 78.53 # 7.853e10
    k_xx = 0.62282 # 622820743.6
    k_yy = 0.3114 # 311410371.8
    k_zz = 0.3114 # 311410371.8
  []
[]

[BCs]
  # [x_motion]
  #   type = PresetAcceleration
  #   boundary = 100
  #   acceleration = accel_x
  #   velocity = vel_x
  #   variable = disp_x
  #   beta = 0.25
  #   function = x_excitation
  # []
  # [x_motion]
  #   type = FunctionDirichletBC
  #   boundary = 'rv_slab_top'
  #   variable = disp_x
  #   function = sawtooth
  # []
  [fix_x]
    type = DirichletBC
    variable = disp_x
    boundary = 100
    preset = true
    value = 0.0
  []
  [fix_y]
    type = DirichletBC
    variable = disp_y
    boundary = 100
    preset = true
    value = 0.0
  []
  [fix_z]
    type = DirichletBC
    variable = disp_z
    boundary = 100
    preset = true
    value = 0.0
  []
  # [grav_z]
  #   type = PresetAcceleration
  #   acceleration = accel_z
  #   velocity = vel_z
  #   variable = disp_z
  #   beta = 0.25
  #   boundary = 100
  #   function = ramp_z
  # []
  [fix_xx]
    type = DirichletBC
    variable = rot_x
    boundary = '4 5'
    value = 0
  []
  [fix_yy]
    type = DirichletBC
    variable = rot_y
    boundary = '4 5'
    value = 0
  []
  [fix_zz]
    type = DirichletBC
    variable = rot_z
    boundary = '4 5'
    value = 0
  []
[]

[Preconditioning]
  [smp]
    type = SMP
    full = true
  []
[]

# [VectorPostprocessors]
#   [accel_hist]
#     type = ResponseHistoryBuilder
#     variables = 'accel_x'
#     nodes = '34579 34566 38419 37936 5073 5840 121 5252 2759 25805 25493 25182 24869 25541 25229 24917 24605 30946 31233'
#   []
#   [accel_spec]
#     type = ResponseSpectraCalculator
#     vectorpostprocessor = accel_hist
#     regularize_dt = 0.002
#     damping_ratio = 0.05
#     start_frequency = 0.1
#     end_frequency = 100
#     outputs = out
#   []
# []

[Executioner]
  type = Transient
  petsc_options = '-ksp_snes_ew'
  petsc_options_iname = '-pc_type -pc_factor_mat_solver_package'
  petsc_options_value = 'lu       superlu_dist'
  solve_type = 'NEWTON'
  # petsc_options = '-snes_ksp_ew'
  # petsc_options_iname = '-ksp_gmres_restart -pc_type -pc_hypre_type -pc_hypre_boomeramg_max_iter'
  # petsc_options_value = '201                hypre    boomeramg      4'
  # solve_type = NEWTON
  nl_rel_tol = 1e-6 # 1e-12 # 1e-10
  nl_abs_tol = 1e-16  # 1e-12 # 1e-8
  # start_time = 0.225
  dt = 0.005 # 0.0025
  start_time = 0.0
  end_time = 0.5
  timestep_tolerance = 1e-6
  automatic_scaling = true
  # [TimeIntegrator]
  #   type = NewmarkBeta
  #   beta = 0.25
  #   gamma = 0.5
  # []
[]

[Controls]
  [inertia_switch]
    type = TimePeriod
    start_time = 0.0
    end_time = 0.01
    disable_objects = '*/inertia_x */inertia_y */inertia_z
                       */vel_x */vel_y */vel_z
                       */rot_vel_x */rot_vel_y */rot_vel_z
                       */accel_x */accel_y */accel_z
                       */rot_accel_x */rot_accel_y */rot_accel_z'
    set_sync_times = true
    execute_on = 'timestep_begin timestep_end'
  []
[]

[Postprocessors]
  # [Cit_Bot_DispX]
  #   type = PointValue
  #   point = '-3.5 0.0 0' # '-15.0 -2.85 8.375'
  #   variable = disp_x
  # []
  # [Cit_Bot_AccX]
  #   type = PointValue
  #   point = '-3.5 0.0 0' # '-15.0 -2.85 8.375'
  #   variable = accel_x
  # []
  # [Iso1_Bot_DispX]
  #   type = PointValue
  #   point = '-3.5 0.0 7.325' # '-15.0 -2.85 8.375'
  #   variable = disp_x
  # []
  # [Iso1_Top_DispX]
  #   type = PointValue
  #   point = '-3.5 0.0 7.625' # '-15.0 -3.010056 8.675'
  #   variable = disp_x
  # []
  # [Iso1_Bot_AccX]
  #   type = PointValue
  #   point = '-3.5 0.0 7.325' # '-15.0 -2.85 8.375'
  #   variable = accel_x
  # []
  # [Iso1_Top_AccX]
  #   type = PointValue
  #   point = '-3.5 0.0 7.625' # '-15.0 -3.010056 8.675'
  #   variable = accel_x
  # []
  [Iso1_Top_AccZ]
    type = PointValue
    point = '-3.5 0.0 7.625' # '-15.0 -3.010056 8.675'
    variable = accel_z
  []
  [Iso1_Fb_X]
    type = PointValue
    point = '-3.5 0.0 7.5'
    variable = Fb_x
  []
  # [Iso1_Fb_Y]
  #   type = PointValue
  #   point = '-3.5 0.0 7.5'
  #   variable = Fb_y
  # []
  # [Iso1_Fb_Z]
  #   type = PointValue
  #   point = '-3.5 0.0 7.5'
  #   variable = Fb_z
  # []
  [Iso1_Defb_X]
    type = PointValue
    point = '-3.5 0.0 7.5'
    variable = Defb_x
  []
  # [Iso1_Defb_Y]
  #   type = PointValue
  #   point = '-3.5 0.0 7.5'
  #   variable = Defb_y
  # []
  # [Iso1_Defb_Z]
  #   type = PointValue
  #   point = '-3.5 0.0 7.5'
  #   variable = Defb_z
  # []
  # [Iso1_Velb_X]
  #   type = PointValue
  #   point = '-3.5 0.0 7.5'
  #   variable = Velb_x
  # []
  # [Iso1_Defb_Y]
  #   type = PointValue
  #   point = '-3.5 0.0 7.5'
  #   variable = Defb_y
  # []
  # [Iso1_Defb_Z]
  #   type = PointValue
  #   point = '-3.5 0.0 7.5'
  #   variable = Defb_z
  # []
[]

[Outputs]
  exodus = true
  perf_graph = true
  csv = true
  # file_base = Ex_Isolator_verify
  [out1]
    type = CSV
    execute_on = 'final'
    # file_base = Isolator_verify
  []
[]
