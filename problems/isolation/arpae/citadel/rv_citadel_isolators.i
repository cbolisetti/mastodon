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

# [Problem]
#   type = ReferenceResidualProblem
#   extra_tag_vectors = 'ref'
#   reference_vector = 'ref'
# []

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
  [stress_xy]
    block = 'RV_housing RV_slab'
    order = CONSTANT
    family = MONOMIAL
  []
  [stress_yz]
    block = 'RV_housing RV_slab'
    order = CONSTANT
    family = MONOMIAL
  []
  [stress_zx]
    block = 'RV_housing RV_slab'
    order = CONSTANT
    family = MONOMIAL
  []
  [strain_xy]
    block = 'RV_housing RV_slab'
    order = CONSTANT
    family = MONOMIAL
  []
  [strain_yz]
    block = 'RV_housing RV_slab'
    order = CONSTANT
    family = MONOMIAL
  []
  [strain_zx]
    block = 'RV_housing RV_slab'
    order = CONSTANT
    family = MONOMIAL
  []
  [stress_xx]
    block = 'RV_housing RV_slab'
    order = CONSTANT
    family = MONOMIAL
  []
  [stress_yy]
    block = 'RV_housing RV_slab'
    order = CONSTANT
    family = MONOMIAL
  []
  [stress_zz]
    block = 'RV_housing RV_slab'
    order = CONSTANT
    family = MONOMIAL
  []
  [strain_xx]
    block = 'RV_housing RV_slab'
    order = CONSTANT
    family = MONOMIAL
  []
  [strain_yy]
    block = 'RV_housing RV_slab'
    order = CONSTANT
    family = MONOMIAL
  []
  [strain_zz]
    block = 'RV_housing RV_slab'
    order = CONSTANT
    family = MONOMIAL
  []
  [vonmises]
    block = 'RV_housing RV_slab'
    order = CONSTANT
    family = MONOMIAL
  []
  [maxP]
    block = 'RV_housing RV_slab'
    order = CONSTANT
    family = MONOMIAL
  []
  [midP]
    block = 'RV_housing RV_slab'
    order = CONSTANT
    family = MONOMIAL
  []
  [minP]
    block = 'RV_housing RV_slab'
    order = CONSTANT
    family = MONOMIAL
  []
  [vonM]
    block = 'RV_housing RV_slab'
    order = CONSTANT
    family = MONOMIAL
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
  [reaction_x]
    block = 'isolators'
  []
  [reaction_y]
    block = 'isolators'
  []
  [reaction_z]
    block = 'isolators'
  []
  [reaction_xx]
    block = 'isolators'
  []
  [reaction_yy]
    block = 'isolators'
  []
  [reaction_zz]
    block = 'isolators'
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

  use_displaced_mesh = true
  # dynamic simulation using consistent mass/inertia matrix
  # dynamic_nodal_translational_inertia = true

  velocities = 'vel_x vel_y vel_z'
  accelerations = 'accel_x accel_y accel_z'
  rotational_velocities = 'rot_vel_x rot_vel_y rot_vel_z'
  rotational_accelerations = 'rot_accel_x rot_accel_y rot_accel_z'

  # beta = 0.25 # Newmark time integration parameter
  # gamma = 0.5 # Newmark time integration parameter

  # beta = 0.4225
  # gamma = 0.8
  # alpha = -0.3

  beta = 0.25
  gamma = 0.5
  # alpha = -0.1

  [block_16]
    block = 'rigid_beams extra_rigid_beams'
    area = 130.06
    Iy = 24166.729
    Iz = 24166.729
    y_orientation = '0.0 0.0 1.0'

    # nodal_mass = 1e-10
    # boundary = 'bottom_isolator'
  []
[]

[Kernels]
  [DynamicTensorMechanics]
    # zeta = 0.00006366
    # alpha = -0.1
    displacements = 'disp_x disp_y disp_z'
    block = 'RV_housing RV_slab'
    # static_initialization = true
    # extra_vector_tags = 'ref'
  []
  [inertia_x]
    type = InertialForce
    block = 'RV_housing RV_slab'
    variable = disp_x
    eta = 7.854
    # extra_vector_tags = 'ref'
  []
  [inertia_y]
    type = InertialForce
    block = 'RV_housing RV_slab'
    variable = disp_y
    eta = 7.854
    # extra_vector_tags = 'ref'
  []
  [inertia_z]
    type = InertialForce
    block = 'RV_housing RV_slab'
    variable = disp_z
    eta = 7.854
    # extra_vector_tags = 'ref'
  []
  # [gravity]
  #   type = Gravity
  #   variable = disp_z
  #   value = -9.81
  #   block = 'RV_slab RV_housing'
  #   # extra_vector_tags = 'ref'
  # []
  [lr_disp_x]
    block = 'isolators'
    type = StressDivergenceIsolator
    displacements = 'disp_x disp_y disp_z'
    rotations = 'rot_x rot_y rot_z'
    component = 0
    variable = disp_x
    save_in = reaction_x
    # extra_vector_tags = 'ref'
  []
  [lr_disp_y]
    block = 'isolators'
    type = StressDivergenceIsolator
    displacements = 'disp_x disp_y disp_z'
    rotations = 'rot_x rot_y rot_z'
    component = 1
    variable = disp_y
    save_in = reaction_y
    # extra_vector_tags = 'ref'
  []
  [lr_disp_z]
    block = 'isolators'
    type = StressDivergenceIsolator
    displacements = 'disp_x disp_y disp_z'
    rotations = 'rot_x rot_y rot_z'
    component = 2
    variable = disp_z
    save_in = reaction_z
    # extra_vector_tags = 'ref'
  []
  [lr_rot_x]
    block = 'isolators'
    type = StressDivergenceIsolator
    displacements = 'disp_x disp_y disp_z'
    rotations = 'rot_x rot_y rot_z'
    component = 3
    variable = rot_x
    save_in = reaction_xx
    # extra_vector_tags = 'ref'
  []
  [lr_rot_y]
    block = 'isolators'
    type = StressDivergenceIsolator
    displacements = 'disp_x disp_y disp_z'
    rotations = 'rot_x rot_y rot_z'
    component = 4
    variable = rot_y
    save_in = reaction_yy
    # extra_vector_tags = 'ref'
  []
  [lr_rot_z]
    block = 'isolators'
    type = StressDivergenceIsolator
    displacements = 'disp_x disp_y disp_z'
    rotations = 'rot_x rot_y rot_z'
    component = 5
    variable = rot_z
    save_in = reaction_zz
    # extra_vector_tags = 'ref'
  []
[]

[NodalKernels]
  [force_z]
    type = UserForcingFunctionNodalKernel
    variable = disp_z
    boundary = 'rv_slab_top'
    function = force_z
  []
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
    type = TestNewmarkTI
    displacement = disp_x
    variable = accel_x
    first = false
  []
  [vel_x]
    # block = 'RV_housing RV_slab'
    type = TestNewmarkTI
    displacement = disp_x
    variable = vel_x
  []
  [accel_y]
    # block = 'RV_housing RV_slab'
    type = TestNewmarkTI
    displacement = disp_y
    variable = accel_y
    first = false
  []
  [vel_y]
    # block = 'RV_housing RV_slab'
    type = TestNewmarkTI
    displacement = disp_y
    variable = vel_y
  []
  [accel_z]
    # block = 'RV_housing RV_slab'
    type = TestNewmarkTI
    displacement = disp_z
    variable = accel_z
    first = false
  []
  [vel_z]
    # block = 'RV_housing RV_slab'
    type = TestNewmarkTI
    displacement = disp_z
    variable = vel_z
  []
  [stress_xy]
    block = 'RV_housing RV_slab'
    type = RankTwoAux
    rank_two_tensor = stress
    variable = stress_xy
    index_i = 1
    index_j = 0
  []
  [stress_yz]
    block = 'RV_housing RV_slab'
    type = RankTwoAux
    rank_two_tensor = stress
    variable = stress_yz
    index_i = 2
    index_j = 1
  []
  [stress_zx]
    block = 'RV_housing RV_slab'
    type = RankTwoAux
    rank_two_tensor = stress
    variable = stress_zx
    index_i = 0
    index_j = 2
  []
  [strain_xy]
    block = 'RV_housing RV_slab'
    type = RankTwoAux
    rank_two_tensor = total_strain
    variable = stress_xy
    index_i = 1
    index_j = 0
  []
  [strain_yz]
    block = 'RV_housing RV_slab'
    type = RankTwoAux
    rank_two_tensor = total_strain
    variable = strain_yz
    index_i = 2
    index_j = 1
  []
  [strain_zx]
    block = 'RV_housing RV_slab'
    type = RankTwoAux
    rank_two_tensor = total_strain
    variable = strain_zx
    index_i = 0
    index_j = 2
  []
  [stress_xx]
    block = 'RV_housing RV_slab'
    type = RankTwoAux
    rank_two_tensor = stress
    variable = stress_xx
    index_i = 0
    index_j = 0
  []
  [stress_yy]
    block = 'RV_housing RV_slab'
    type = RankTwoAux
    rank_two_tensor = stress
    variable = stress_yy
    index_i = 1
    index_j = 1
  []
  [stress_zz]
    block = 'RV_housing RV_slab'
    type = RankTwoAux
    rank_two_tensor = stress
    variable = stress_zz
    index_i = 2
    index_j = 2
  []
  [strain_xx]
    block = 'RV_housing RV_slab'
    type = RankTwoAux
    rank_two_tensor = total_strain
    variable = strain_xx
    index_i = 0
    index_j = 0
  []
  [strain_yy]
    block = 'RV_housing RV_slab'
    type = RankTwoAux
    rank_two_tensor = total_strain
    variable = strain_yy
    index_i = 1
    index_j = 1
  []
  [strain_zz]
    block = 'RV_housing RV_slab'
    type = RankTwoAux
    rank_two_tensor = total_strain
    variable = strain_zz
    index_i = 2
    index_j = 2
  []
  [vonmises]
    block = 'RV_housing RV_slab'
    type = RankTwoScalarAux
    rank_two_tensor = stress
    variable = vonmises
    scalar_type = VonMisesStress
    execute_on = timestep_end
  []
  [maxP]
    block = 'RV_housing RV_slab'
    type = RankTwoScalarAux
    rank_two_tensor = stress
    variable = maxP
    scalar_type = MaxPrincipal
    execute_on = timestep_end
  []
  [minP]
    block = 'RV_housing RV_slab'
    type = RankTwoScalarAux
    rank_two_tensor = stress
    variable = minP
    scalar_type = MinPrincipal
    execute_on = timestep_end
  []
  [midP]
    block = 'RV_housing RV_slab'
    type = RankTwoScalarAux
    rank_two_tensor = stress
    variable = midP
    scalar_type = MidPrincipal
    execute_on = timestep_end
  []
  [rot_accel_x]
    block = 'isolators rigid_beams extra_rigid_beams'
    type = TestNewmarkTI
    displacement = rot_x
    variable = rot_accel_x
    first = false
  []
  [rot_vel_x]
    block = 'isolators rigid_beams extra_rigid_beams'
    type = TestNewmarkTI
    displacement = rot_x
    variable = rot_vel_x
  []
  [rot_accel_y]
    block = 'isolators rigid_beams extra_rigid_beams'
    type = TestNewmarkTI
    displacement = rot_y
    variable = rot_accel_y
    first = false
  []
  [rot_vel_y]
    block = 'isolators rigid_beams extra_rigid_beams'
    type = TestNewmarkTI
    displacement = rot_y
    variable = rot_vel_y
  []
  [rot_accel_z]
    block = 'isolators rigid_beams extra_rigid_beams'
    type = TestNewmarkTI
    displacement = rot_z
    variable = rot_accel_z
    first = false
  []
  [rot_vel_z]
    block = 'isolators rigid_beams extra_rigid_beams'
    type = TestNewmarkTI
    displacement = rot_z
    variable = rot_vel_z
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
    x = '-1000 0 1e-6 1000'
    y =     '0 0    1    1'
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
  #   y = '0 0.05 -0.1 0.15 -0.2 0.25 -0.25'
  #   scale_factor = 1e-7
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
    prop_values = 2.4e-6 #kg/m3
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
    # prop_values = 7.85e-6 #kg/m3
    # prop_values = 0.00001095551 #(4*0.0008902575/9.81)/33.134
    prop_values = 1.1e-5 #GNs2/m4 #includes RV and internals (300T)
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
    r_eff = 2.2352
    r_contact = 0.2
    uy = 0.001
    unit = 4
    beta = 0.25
    gamma = 0.5
    # beta = 0.3025
    # gamma = 0.6
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
  [x_motion]
    type = PresetAcceleration
    boundary = 100
    acceleration = accel_x
    velocity = vel_x
    variable = disp_x
    beta = 0.25
    function = x_excitation
  []
  # [x_motion]
  #   type = FunctionDirichletBC
  #   boundary = 'rv_slab_top'
  #   variable = disp_x
  #   function = sawtooth
  # []
  # [fix_x]
  #   type = DirichletBC
  #   variable = disp_x
  #   boundary = 100
  #   preset = true
  #   value = 0.0
  # []
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
  # solve_type = 'NEWTON'
  nl_rel_tol = 1e-6 # 1e-12 # 1e-10
  nl_abs_tol = 1e-16  # 1e-12 # 1e-8
  # start_time = 0.225
  dt = 0.005 # 0.0025
  start_time = -1.0
  end_time = 5.0
  timestep_tolerance = 1e-6
  automatic_scaling = true
  [TimeIntegrator]
    type = NewmarkBeta
    beta = 0.25
    gamma = 0.5
  []
[]

# [Controls]
#   [inertia_switch]
#     type = TimePeriod
#     start_time = 0.0
#     end_time = 0.15
#     disable_objects = '*/inertia_x */inertia_y */inertia_z
#                        */vel_x */vel_y */vel_z
#                        */rot_vel_x */rot_vel_y */rot_vel_z
#                        */accel_x */accel_y */accel_z
#                        */rot_accel_x */rot_accel_y */rot_accel_z'
#     set_sync_times = true
#     execute_on = 'timestep_begin timestep_end'
#   []
# []

[Postprocessors]
  [Cit_Bot_DispX]
    type = PointValue
    point = '-3.5 0.0 0' # '-15.0 -2.85 8.375'
    variable = disp_x
  []
  [Cit_Bot_AccX]
    type = PointValue
    point = '-3.5 0.0 0' # '-15.0 -2.85 8.375'
    variable = accel_x
  []
  [Iso1_Bot_DispX]
    type = PointValue
    point = '-3.5 0.0 7.325' # '-15.0 -2.85 8.375'
    variable = disp_x
  []
  [Iso1_Top_DispX]
    type = PointValue
    point = '-3.5 0.0 7.625' # '-15.0 -3.010056 8.675'
    variable = disp_x
  []
  [Iso1_Bot_AccX]
    type = PointValue
    point = '-3.5 0.0 7.325' # '-15.0 -2.85 8.375'
    variable = accel_x
  []
  [Iso1_Top_AccX]
    type = PointValue
    point = '-3.5 0.0 7.625' # '-15.0 -3.010056 8.675'
    variable = accel_x
  []
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
  [Iso1_Fb_Y]
    type = PointValue
    point = '-3.5 0.0 7.5'
    variable = Fb_y
  []
  [Iso1_Fb_Z]
    type = PointValue
    point = '-3.5 0.0 7.5'
    variable = Fb_z
  []
  [Iso1_Defb_X]
    type = PointValue
    point = '-3.5 0.0 7.5'
    variable = Defb_x
  []
  [Iso1_Defb_Y]
    type = PointValue
    point = '-3.5 0.0 7.5'
    variable = Defb_y
  []
  [Iso1_Defb_Z]
    type = PointValue
    point = '-3.5 0.0 7.5'
    variable = Defb_z
  []
  # [Iso1_Bot_ReacY]
  #   type = PointValue
  #   point = '-3.5 0.0 7.325' # '-15.0 -2.85 8.375'
  #   variable = reaction_y
  # []
  # [Iso1_Top_ReacY]
  #   type = PointValue
  #   point = '-3.5 0.0 7.625' # '-15.0 -3.010056 8.675'
  #   variable = reaction_y
  # []
  # [Iso1_Bot_ReacX]
  #   type = PointValue
  #   point = '-3.5 0.0 7.325' # '-15.0 -2.85 8.375'
  #   variable = reaction_x
  # []
  # [Iso1_Top_ReacX]
  #   type = PointValue
  #   point = '-3.5 0.0 7.625' # '-15.0 -2.85 8.375'
  #   variable = reaction_x
  # []
  # [Iso2_Bot_ReacX]
  #   type = PointValue
  #   point = '3.5 0.0 7.325' # '-15.0 -2.85 8.375'
  #   variable = reaction_x
  # []
  # [Iso2_Top_ReacX]
  #   type = PointValue
  #   point = '3.5 0.0 7.625' # '-15.0 -2.85 8.375'
  #   variable = reaction_x
  # []
  # [Iso2_Bot_ReacZ]
  #   type = PointValue
  #   point = '3.5 0.0 7.325' # '-15.0 -2.85 8.375'
  #   variable = reaction_z
  # []
  # [Iso2_Top_ReacZ]
  #   type = PointValue
  #   point = '3.5 0.0 7.625' # '-15.0 -2.85 8.375'
  #   variable = reaction_z
  # []
[]

# [VectorPostprocessors]
#   [accel_hist_x]
#     type = ResponseHistoryBuilder
#     variables = 'accel_x'
#     nodes = '58376 28136 28733 28661 28445 59602 30904 59034 31371 42014 42032 42030 42044 42058 25533 25249 24963 24676'
#     # nodes = '31192 58401 59068 60306 58376 28136 28733 28661 28445 42014 42030 42044 42058 25536 25250 24959 24676'
#     outputs = out1
#     block = 'roof ext_buttresses ext_walls int_buttresses SG_bases int_wall int_slab RV_housing small_walls basemat RV SGs RV_slab' # 99 98 97 96 95 94'
#   []
#   [accel_spec_x]
#     type = ResponseSpectraCalculator
#     vectorpostprocessor = accel_hist_x
#     regularize_dt = 0.001
#     damping_ratio = 0.05
#     start_frequency = 0.1
#     end_frequency = 1000
#     outputs = out1
#   []
#
#   [accel_hist_y]
#     type = ResponseHistoryBuilder
#     variables = 'accel_y'
#     nodes = '58376 28136 28733 28661 28445 59602 30904 59034 31371 42014 42032 42030 42044 42058 25533 25249 24963 24676'
#     # nodes = '31192 58401 59068 60306 58376 28136 28733 28661 28445 42014 42030 42044 42058 25536 25250 24959 24676'
#     outputs = out1
#     block = 'roof ext_buttresses ext_walls int_buttresses SG_bases int_wall int_slab RV_housing small_walls basemat RV SGs RV_slab' # 99 98 97 96 95 94'
#   []
#   [accel_spec_y]
#     type = ResponseSpectraCalculator
#     vectorpostprocessor = accel_hist_y
#     regularize_dt = 0.001
#     damping_ratio = 0.05
#     start_frequency = 0.1
#     end_frequency = 1000
#     outputs = out1
#   []
#
#   [accel_hist_z]
#     type = ResponseHistoryBuilder
#     variables = 'accel_z'
#     nodes = '58376 28136 28733 28661 28445 59602 30904 59034 31371 42014 42032 42030 42044 42058 25533 25249 24963 24676'
#     # nodes = '31192 58401 59068 60306 58376 28136 28733 28661 28445 42014 42030 42044 42058 25536 25250 24959 24676'
#     outputs = out1
#     block = 'roof ext_buttresses ext_walls int_buttresses SG_bases int_wall int_slab RV_housing small_walls basemat RV SGs RV_slab' # 99 98 97 96 95 94'
#   []
#   [accel_spec_z]
#     type = ResponseSpectraCalculator
#     vectorpostprocessor = accel_hist_z
#     regularize_dt = 0.001
#     damping_ratio = 0.05
#     start_frequency = 0.1
#     end_frequency = 1000
#     outputs = out1
#   []
# []

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
