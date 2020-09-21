-----------------------------------------------------------------------------
    Copyright 2010, 2011, Johan Wahlström, Lars Eriksson

    This package is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as
    published by the Free Software Foundation, version 3 of the License.

    This package is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
-----------------------------------------------------------------------------

This package is a Simulink-implementation of a diesel engine model with
EGR and VGT. The intention of this implementation is to give
researchers and engineers access to a validated model that can be used
for simulation, development, and verification of new control
systems. Researchers working on methodology development can use this
model as a challenging reference case since it describes a highly
nonlinear system with non-minimum phase behavior.

The package consist of four files: 

1) readme.txt: this file 

2) engine.mdl: Simulink-implementation of the model 

3) runSimEngine.m: Matlab-script that simulates engine.mdl for two steps
in VGT position. This Matlab-script illustrates how the implementation
can be simulated and gives examples on two step responses that have
non-minimum phase behavior, overshoot, and sign reversal.

4) parameterData.mat: Model parameters

If you use this model in research or otherwise, cite the publication that
develops and documents the model. The paper to cite is Wahlström Eriksson (2011).


References

Johan Wahlström and Lars Eriksson.
Modeling diesel engines with a variable-geometry turbocharger and exhaust gas recirculation by optimization of model parameters for capturing non-linear system dynamics.
Proceedings of the Institution of Mechanical Engineers, Part D, Journal of Automobile Engineering, Volume 225, Issue 7, July 2011.
http://dx.doi.org/10.1177/0954407011398177







