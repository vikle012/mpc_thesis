% runSimEngine - Script to run the TCDI_EGR_VGT model
%
% Simulates TCDI_EGR_VGT.mdl at two different steps in VGT-position. The first
% step shows a non-minimum phase behavior and that the DC-gain is negative
% in the channel VGT to compressor mass flow W_c. The second step shows an
% overshoot and that the DC-gain is positive in the channel VGT to
% compressor mass flow W_c, i.e. there is a sign reversal in this channel.
% 
% The inputs have the following ranges:
% Engine speed, n_e:       500-2000 rpm
% Fuel injection, u_delta: 1-250    mg/cycle
% EGR-valve, u_egr:        0-100    %
% VGT-position, u_vgt:     20-100   %


%    Copyright 2010, 2011, Johan Wahlström, Lars Eriksson
%
%    This package is free software: you can redistribute it and/or modify
%    it under the terms of the GNU Lesser General Public License as
%    published by the Free Software Foundation, version 3 of the License.
%
%    This package is distributed in the hope that it will be useful,
%    but WITHOUT ANY WARRANTY; without even the implied warranty of
%    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
%    GNU Lesser General Public License for more details.
%
%    You should have received a copy of the GNU Lesser General Public License
%    along with this program.  If not, see <http://www.gnu.org/licenses/>.


clear

load parameterData

control.u_egract=1;
% 1: with EGR-actuator dynamics
% 0: without EGR-actuator dynamics

control.u_vgtact=1;
% 1: with VGT-actuator dynamics
% 0: without VGT-actuator dynamics


for step=1:2
    switch step
      case 1
        %set initial values for the inputs
        %First column: time vector
        %Second column: data vector
        simU.n_e=[0 1500]; 
        simU.u_delta=[0 110];
        simU.u_egr=[0 80];        
        simU.u_vgt=[0 75];
        

        %simulate the engine for the initial values
        sim('TCDI_EGR_VGT',20)

        %set values for the input step
        %First column: time vector
        %Second column: data vector
        simU.u_vgt=[[0 1]' [75 65]'];
      case 2
        %set initial values for the inputs
        %First column: time vector
        %Second column: data vector
        simU.n_e=[0 1500]; 
        simU.u_delta=[0 110];
        simU.u_egr=[0 80];
        simU.u_vgt=[0 30];        
        
        %simulate the engine for the initial values
        %until it has reach a stationary point
        sim('TCDI_EGR_VGT',20)

        %set values for the input step
        %First column: time vector
        %Second column: data vector
        simU.u_vgt=[[0 1]' [30 25]'];
    end

    %use final values from the initial simulation as initial state in the
    %simualtion of the step
    model.x_r_Init=simx_r(end);
    model.T_1_Init=simT_1(end);
    model.uInit_egr=simu_egr(end);
    model.uInit_vgt=simu_vgt(end);
    opt=simset('InitialState',xFinal);
    
    %simulate the step
    sim('TCDI_EGR_VGT',8,opt)
    
    %collect variables from the simulation in a struct-format
    
    %temperature [K]
    simEngine.T_c=simT_c;
    simEngine.T_e=simT_e;
    simEngine.T_em=simT_em;

    %pressure [Pa]
    simEngine.p_im=simp_im;
    simEngine.p_em=simp_em;

    %oxygen mass fraction [-]
    simEngine.X_Oim=simX_Oim;
    simEngine.X_Oem=simX_Oem;

    %massflow [kg/s]
    simEngine.W_c=simW_c;
    simEngine.W_t=simW_t;
    simEngine.W_ei=simW_ei;
    simEngine.W_f=simW_f;
    simEngine.W_egr=simW_egr;

    %efficiency [-]
    simEngine.eta_tm=simeta_tm;
    simEngine.eta_c=simeta_c;
    simEngine.BSR=simBSR;

    %Power [W]
    simEngine.P_c=simP_c;
    simEngine.P_tm=simP_tm;

    %torque [Nm]
    simEngine.M_e=simM_e;

    %speed [rpm]
    simEngine.n_t=simn_t;
    simEngine.n_e=simn_e;

    %control signals
    simEngine.u_delta=simu_delta;
    simEngine.u_egr=simu_egr;
    simEngine.u_vgt=simu_vgt;

    %output
    simEngine.lambda=simlambda;
    simEngine.x_egr=simx_egr;

    %time [s]
    simEngine.time=simTime;

    %plot some interesting variables
    
    figure(step)
    clf
    subplot(221)
    plot(simEngine.time,simEngine.u_vgt)
    ylabel('u_{vgt} [%]')
    oldaxis=axis;
    axis([0 8 oldaxis(3)-1 oldaxis(4)+1]) 

    subplot(222)
    plot(simEngine.time,simEngine.W_c)
    ylabel('W_{c} [kg/s]')

    subplot(223)
    plot(simEngine.time,simEngine.p_im)
    ylabel('p_{im} [Pa]')
    xlabel('Time [s]')

    subplot(224)
    plot(simEngine.time,simEngine.p_em)
    ylabel('p_{em} [Pa]')
    xlabel('Time [s]')
    
end
