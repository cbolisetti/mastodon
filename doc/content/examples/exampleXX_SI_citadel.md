# Example XX: Component seismic isolation of a molten salt reactor vessel with fluid-structure interaction

!alert note title=Units of this example
Length in m, time in sec, force in GN, and pressure in GPa

## Description

This example illustrates the seismic analysis of an idealized, cylindrical reactor vessel that is filled with fluid and is head supported over seismic isolators. Molten salt reactors, which use molten salt as a coolant, are being considered by several advanced reactor developers in the United States. Seismic isolation of molten salt reactors drastically reduces the seismic loads on these reactors and improves economics and safety of the power plant [ref](ref). MASTODON can perform fluid-structure interaction (FSI) analyses of seismically isolated reactor vessels and can calculate hydrodynamic pressures in the fluid and on the vessel walls, stresses and strains on the vessel, wave heights at the surface of the vessel (which are accurate only for smaller wave heights with the acoustic FSI implementation in MASTODON), foundation forces and moments, etc. This example demonstrates one such analysis for an idealized reactor vessel.

## Modeling in MASTODON

### Description of the model

In a real nuclear power plant, reactor vessels are typically either head supported or base supported. This example involves a head-supported cylindrical reactor vessel, with the reactor 'head' being a circular steel plate with a thickness of 0.15 m and a diameter of 7.5 m. The reactor vessel is suspended from the reactor head, which is supported by four Friction Pendulum^TM^ seismic isolators. The reactor vessel itself is 6 m tall with a diameter of 5 m and a thickness of 5 cm. The reactor vessel is assumed to be partially filled with the fluid surface being 1 m below the top of the reactor vessel. For the demonstration of this example, the fluid inside the reactor vessel is assumed to be water, with a density of 1000 kg/m^3^ and a bulk modulus of about 2.1 GPa.

The reactor vessel, reactor head, and the fluid are meshed using HEX8 elements and the mesh is built using Cubit. The isolators are modeled using two-noded line elements, which are attached to the reactor head through very stiff beams that simulate an almost rigid connection between the line element and the solid elements of the reactor head. The finite-element mesh of the model is shown in [fig:ex_model.png]. FSI is simulated using an [acoustic formulation](ref), for which, a continuous mesh is required between the reactor vessel and the fluid. In the formulation used in MASTODON (i.e., the [FSI module of MOOSE](ref)), FSI is simulated using `InterfaceKernels`, which require a continuous mesh to be present at the interface.

!media media/examples/ex_model.png
       style=width:50%;margin-left:150px;float:center;
       id=fig:ex10a_input
       caption=Finite-element mesh of the reactor vessel assembly of this example.

### Material properties

The reactor vessel and the reactor head are modeled with linear elastic material models with the properties of stainless steel (Young's modulus of 200 GPa, Poisson's ratio of 0.27, and density of 7850 kg/m^3^). The isolators are designed for a sliding period of 1 sec, using the expression $T = \sqrt(W/r_eff)$, where $W$ is the total weight supported by each isolator, and $r_eff$ is the radius of curvature of the isolator (see [ref] for a description the design of FP isolation systems). In this example, the total weight supported by the isolators is ~XX~ tons. For a sliding period of 1 sec, using the expression above, $r_eff$ is calculated as 0.25 m. The Friction Pendulum isolators are modeled using the `ComputeFPIsolatorElasticity` and the `ComputeIsolatorDeformation` objects as shown below in the input file.

!listing examples/ex??/RV_isolators.i start=elasticity end=???

The acoustic fluid elements need only one materials block that provides the inverse of speed of sound (`inv_co_sq`). Here, this is specified using the `GenericConstantMaterial` object and the `inv_co_sq` value is calculated as 4.65e-7 m^-2^s^-2^. In addition to this materials block, the acoustic fluid is defined by the `AcousticInertia` and the `Diffusion` kernels. Further description of modeling FSI is described in the [FSI module](ref) and examples [XX](ref) and [AA](ref).

## Boundary conditions

The reactor vessel and reactor head are supported by the seismic isolators. In a real nuclear power plant, these isolators would rest on a reinforced concrete citadel as shown in example [XX](ref), which demonstrates the seismic analysis of the whole nuclear power plant that contains this reactor vessel. The earthquake ground motion in this example is therefore applied to the base of the seismic isolators. For demonstration, ~a simple GM is applied here~ in the X direction using the `PresetAcceleration` BC. The displacements and rotations in all other directions are constrained at the base of the isolators. The free surface of the fluid is modeled using the `FluidFreeSurfaceBC` BC. In addition to these BCs, the nodes where the rigid beams connect to the solid elements of the reactor head are constrained to have zero rotations. This is needed in order to avoid zeroes in the Jacobian since the nodes of solid elements have zero rotational stiffness.

!listing examples/ex??/RV_isolators.i block=BCs

In addition to the BCs listed above, the

## Results

The results of the simulation above are presented in this section. The following results are presented:

 - Acceleration histories in the X direction above and below one of the seismic isolators.
 - Pressure histories at the surface, mid-level, and the base of the fluid on the center line of the fluid volume.
 - Wave heights at the fluid surface at three points on the y=0 line: center point, and two points at the flui-structure interface.


These results are all calculated using `PointValue` postprocessors that query and store AuxVariable values at the requested points.
