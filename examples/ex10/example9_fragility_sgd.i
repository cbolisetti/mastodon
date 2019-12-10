# Fragility calculation for the GNF in Yu et al 2018
# Demands from LS-DYNA simulations are converted for MOOSE csv format and
# used here.
[Mesh]
  # dummy mesh
  type = GeneratedMesh
  dim = 3
[]

[Variables]
  # dummy variables
  [./u]
  [../]
[]

[Problem]
  solve = false
  kernel_coverage_check = false
[]

[Executioner]
  type = Transient
  num_steps = 1
[]

[VectorPostprocessors]
  [./fragility_mcc]
    type = Fragility
    master_file = 'inl_moose_nosi/master' # name of the master file in this case is 'master.i'
    hazard_multiapp = 'run_hazard'
    probabilistic_multiapp = 'sub'
    num_gms = 30
    demand_variable_names = 'accel_x_1712'
    frequencies = '3 10 20' # 20 frequencies between 3-10
    demand_calc_type = average # spectral accelerations are averaged
    damping_ratio = 0.05
    dt = 0.005
    median_capacity = 14.5
    beta_capacity = 0.46
    num_samples = 1
    num_bins = 6
    im_values = '0.33 0.65 0.98 1.30 1.63 1.95'
    median_fragility_limits = '1.0 8.0'
    beta_fragility_limits = '0.01 1.0'
    execute_on = TIMESTEP_END
  [../]
[]

[Outputs]
  csv = true
  execute_on = 'final'
[]
