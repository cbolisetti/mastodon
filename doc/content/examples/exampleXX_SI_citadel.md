# Example XX: Component seismic isolation of a molten salt reactor vessel with fluid-structure interaction

## Description

This example illustrates the seismic analysis of an idealized, cylindrical reactor vessel that is filled with fluid and is head supported over seismic isolators. Molten salt reactors, which use molten salt as a coolant, are being considered by several advanced reactor developers in the United States. Seismic isolation of molten salt reactors drastically reduces the seismic loads on these reactors and improves economics and safety of the power plant [ref](ref). MASTODON can perform fluid-structure interaction (FSI) analyses of seismically isolated reactor vessels and can calculate hydrodynamic pressures in the fluid and on the vessel walls, stresses and strains on the vessel, wave heights at the surface of the vessel (which are accurate only for smaller wave heights with the acoustic FSI implementation in MASTODON), foundation forces and moments, etc. This example demonstrates one such analysis for an idealized reactor vessel.  

## Modeling in MASTODON

### Description of the model

In a real nuclear power plant, reactor vessels are typically either head supported or base supported. This example involves a head-supported cylindrical reactor vessel, with the reactor 'head' being a circular steel plate with a thickness of 0.15 m and a diameter of 7.5 m. The reactor vessel is suspended from the reactor head, which is supported by four Friction Pendulum^TM^ seismic isolators. The reactor vessel itself is 6 m tall with a diameter of 5 m and a thickness of 5 cm. The reactor vessel is assumed to be partially filled with the fluid surface being 1 m below the top of the reactor vessel. For the demonstration of this example, the fluid inside the reactor vessel is assumed to be water, with a density of 1000 kg/m^3^ and a bulk modulus of about 2.1 GPa.

The reactor vessel, reactor head and the fluid are meshed using HEX8 elements and the mesh is built using Cubit. The fluid 

!media media/examples/MASTODON_ex10_model.png
       style=width:50%;margin-left:150px;float:center;
       id=fig:ex10a_input
       caption=Finite-element mesh of the reactor vessel, isolators, and the citadel of this example.


## Results
