

Author: Jonatan Gustafsson
Date: 2021-04-02


###########################################################################

How to simulate system with MPC controllers

###########################################################################

1. Open "Controllers/run_controlledSystem.m"

2. Select MPC controller by modifying "controller_object"

3. Modify the selected MPC controller if desired
(integral action on/off, sample rate "init_param.T_s", prediction horizon 
"init_param.N" and weight matrices "Q", "R", "S")

4. Set the system inputs "M_e_input", "n_e_input" and "time_vector" that 
indicates how "M_e_input" and "n_e_input" vary with time. Some examples
follow with the run file

5. Run file to simulate system with MPC controller with given inputs

###########################################################################

Folder Content

###########################################################################

--------- CasADi ---------

Contains the tool CasADi (https://web.casadi.org/) that is used to solve 
optimization problems, as well as some basic CasADi tests.

--------- Controllers ---------

The main folder which contains the reference generator, MPC implementations
and the controlled system. The MPC implementations are considered below,
where each design has separate files for with and without preview.

*L.m & L_preview.m*
Linearization based MPC with linearization about a single linearization 
point corresponding to 80 kW mechanical power at 1200 rpm engine speed

*SL_current.m & SLpreview_current.m*
Successive linearization based MPC with linearization about current 
operating point (x(t), u(t))

*SL_ref.m & SLpreview_ref.m*
Successive linearization based MPC with linearization about reference 
values (x_ref and u_ref)

--------- Figures ---------

Contains results in pdf formats.

--------- Lookup_tables ---------

Contains everything related to constructing the lookup tables based on an
engine map approach. Also contains lookup table data.

--------- MATLAB_model ---------

Contains a verfied MATLAB model representation of the diesel engine model 
created by Wahlstrom and Eriksson. The MATLAB model neglects the actuator
dynamics.

--------- Misc ---------

Miscellaneous.

--------- Simulation_data ---------

Contains WHTC simulation results using the examined MPC controllers.

--------- WahlstromErikssonTCDI_EGR_VGT ---------

Contains the original Simulink model of the examined diesel engine, its
parameter data set and a run example. The model is downloaded from 
https://www.vehicular.isy.liu.se/Software/TCDI_EGR_VGT/

--------- WHTC ---------

Contains everything related to the World Harmonized Transient Cycle:
plotting, de-normalization and data.
