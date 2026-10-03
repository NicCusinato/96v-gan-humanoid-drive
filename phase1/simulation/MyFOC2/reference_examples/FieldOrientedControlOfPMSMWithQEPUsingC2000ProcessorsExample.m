%% Field-Oriented Control of PMSM with Quadrature Encoder Using C2000 Processors
%
% This example implements the field-oriented control (FOC) technique to control 
% the speed of a three-phase permanent magnet synchronous motor (PMSM). The FOC algorithm requires rotor position feedback, which is obtained by a quadrature encoder sensor.
% For details about FOC, see <docid:mcb_gs#mw_2d6dd8ca-938f-44d3-8334-f5e42bf4b73b
% Field-Oriented Control (FOC)>.
%
% A closed-loop FOC algorithm is used to regulate the speed and torque of a three-phase PMSM. 
% This example uses C28x peripheral blocks from the C2000(TM) Microcontroller Blockset and MCB library blocks from Motor Control Blockset(TM).
% 
% This example uses the quadrature encoder sensor to measure the rotor position. The quadrature encoder sensor consists of a disk with two tracks or channels that are coded 90 electrical degrees out of phase. 
% This creates two pulses (A and B) that have a phase difference of 90 degrees and an index pulse (I). 
% Therefore, the controller uses the phase relationship between A and B channels and the transition of channel states to determine the direction of rotation of the motor.
% 
% <<../mcb_quad_enc.png>>
% 
%
% Copyright 2021-2023 The MathWorks, Inc.

%% Required Hardware
% This example supports these hardware configurations. Use the target model name (highlighted in bold) to open the model for the corresponding hardware configuration, from the MATLAB(R) command prompt.
% 
% * F28035 control card + DRV8312-C2-KIT inverter: <matlab:openExample('c2b/FieldOrientedControlOfPMSMWithQEPUsingC2000ProcessorsExample','supportingFile','mcb_pmsm_foc_qep_f28035') mcb_pmsm_foc_qep_f28035>
%
% * F28335 control card + DRV8312-C2-KIT inverter: <matlab:openExample('c2b/FieldOrientedControlOfPMSMWithQEPUsingC2000ProcessorsExample','supportingFile','mcb_pmsm_foc_qep_f28335') mcb_pmsm_foc_qep_f28335>
%
% * LAUNCHXL-F280049C controller + BOOSTXL-DRV8305 inverter: <matlab:openExample('c2b/FieldOrientedControlOfPMSMWithQEPUsingC2000ProcessorsExample','supportingFile','mcb_pmsm_foc_qep_f280049C') mcb_pmsm_foc_qep_f280049C>
% 
% * Three-phase PMSM with optional QEP sensors attached to connector J4 of
% the DRV8312-C2-KIT inverter board or connector J12 of LAUNCHXL-F280049C.
% 
% * For F28069M control card, LAUNCHXL-F28069M controller and LAUNCHXL-F28379D controller, refer to <docid:mcb_gs#mw_4df1139b-472a-4ec8-91ee-0f0170cd4c2c MCB>.
%
% For connections related to the preceding hardware configuration, see
% <docid:c2b_ug#mw_afa3a9f7-301c-42dd-a6bd-9bf5dbdbaae1 Hardware Connections>.
%% Available Models
% The example includes these models:
%
% * <matlab:openExample('c2b/FieldOrientedControlOfPMSMWithQEPUsingC2000ProcessorsExample','supportingFile','mcb_pmsm_foc_qep_f28035') mcb_pmsm_foc_qep_f28035>
%
% * <matlab:openExample('c2b/FieldOrientedControlOfPMSMWithQEPUsingC2000ProcessorsExample','supportingFile','mcb_pmsm_foc_qep_f28335') mcb_pmsm_foc_qep_f28335>
%
% * <matlab:openExample('c2b/FieldOrientedControlOfPMSMWithQEPUsingC2000ProcessorsExample','supportingFile','mcb_pmsm_foc_qep_f280049C') mcb_pmsm_foc_qep_f280049C>
%
% *Note:* For F28069M control card, LAUNCHXL-F28069M controller and LAUNCHXL-F28379D controller, refer to <docid:mcb_gs#mw_4df1139b-472a-4ec8-91ee-0f0170cd4c2c MCB>.
% 
% You can use these models for both simulation and code generation. Open the <matlab:openExample('c2b/FieldOrientedControlOfPMSMWithQEPUsingC2000ProcessorsExample','supportingFile','mcb_pmsm_foc_qep_f28035') mcb_pmsm_foc_qep_f28035> model.
%%
open_system('mcb_pmsm_foc_qep_f28035.slx');
%%
%
% You may need to change the model parameters to fit your specific motor. Match motor voltage and power characteristics to the controller.
% A conventional voltage-source inverter drives motor. The controller algorithm generates six pulse width modulation (PWM) signals using a vector PWM technique for six power switching devices. 
% The inverter measures the current of the two motor inputs (ia and ib) input currents of the motor (ia and ib) using two analog-to-digital converters (ADCs) and sends the measurements to the processor.
%
%% Peripheral Block Configurations
% Set the peripheral block configurations for this model. Double-click on the blocks to open block parameter configurations. You can use the same parameter values if you want to run this example for other hardware boards.
% 
% * *ePWM Block configuration*
% 
% <<../foc_qep.png>>
% 
% * *ADC Block configuration*
% 
% The algorithm in this example uses an asynchronous scheduling. The pulse width modulation (PWM) block triggers the ADC conversion. 
% At the end of conversion, the ADC posts an interrupt that triggers the main FOC algorithm. For more information, refer <docid:c2b_ug#mw_ce3bbb8e-3411-4ab3-826d-d5dcc239ead4 ADC Interrupt Based Scheduling>.
%
% <<../foc_qep5.png>>
%
%% Configure the Model
% *1.* Open the <matlab:open_system('mcb_pmsm_foc_qep_f28035') mcb_pmsm_foc_qep_f28035> model. This model is configured for TI Piccolo F2803x hardware.
% 
% *2.* To run the model on other TI C2000 processors, first press *Ctrl+E* to open the Configuration Parameters dialog box. Then, select the required hardware board by navigating to *Hardware Implementation* > *Hardware board*.
%
% *3.* The following screenshots show the scheduler configurations performed in the model. You can use the same parameter values if you want to run this example for other hardware boards.
%
% <<../foc_qep2.png>>
%
% *Note:*
%
% * Sampling rate of the ADC block should be same as the base rate of the model determined by the PWM period of the ePWM block.
% 
% * Base rate trigger selection should be same as the interrupt triggered
% by the ADC module. For more information, see Model Configuration Parameters for Texas Instruments C2000(TM) Processors.
%
% * Ensure that the *Default parameter behavior* (Configuration Parameters > Code Generation > Optimization) is set to *Inlined*.
% 
% *4.* Ensure that the baud rate is set to 1.5e6 bits/sec.
%
% <<../foc_qep3.png>>
%
%% Required MathWorks(R) Products
% 
% *To simulate model:*
% 
% For the models: *mcb_pmsm_foc_qep_f28035*, *mcb_pmsm_foc_qep_f28335* and
% *mcb_pmsm_foc_qep_f280049C*
% 
% * Motor Control Blockset(TM)
% * Fixed-Point Designer(TM)
% 
% *To generate code and deploy model:*
% 
% For the models: *mcb_pmsm_foc_qep_f28035*, *mcb_pmsm_foc_qep_f28335* and
% *mcb_pmsm_foc_qep_f280049C*
% 
% * Motor Control Blockset(TM)
% * Embedded Coder(R)
% * C2000(TM) Microcontroller Blockset
% * Fixed-Point Designer(TM)
% 
%% Prerequisites
% 
% * If you obtain the motor parameters from the datasheet or other sources, update the motor parameters and inverter parameters in the model initialization script
% associated with the Simulink(R) models. For instructions, see
% <docid:mcb_gs#mw_dd32d1fd-68d8-4cfd-8dea-ef7f7ed008c0 Estimate Control Gains from Motor Parameters>.
% 
%% Simulate Model
% 
% This example supports simulation. Follow these steps to simulate the
% model.
% 
% *1.* Open a model included with this example.
% 
% *2.* Click *Run* on the *Simulation* tab to simulate the model.
% 
% *3.* Click *Data Inspector* on the *Simulation* tab to view and analyze the simulation
% results.
% 
%% Generate Code and Deploy Model to Target Hardware
% This section instructs you to generate code and run the FOC algorithm on
% the target hardware.
% 
% This example uses a host and a target model. The host model is a user interface to the controller hardware board.  You can run the host model on the host computer. The prerequisite to use the host model is to deploy the target model to the controller hardware board. 
% The host model uses serial communication to command the target Simulink(R) model and run the motor in a closed-loop control.
% 
% *Note:* For F28335 processor you need to use external FTDI for serial communication.
%
% *1.* Simulate the target model and observe the simulation results.
% 
% *2.* Complete the hardware connections.
% 
% *3.* The model automatically computes the ADC (or current) offset values. To disable this functionality (enabled by default), update the value 0 to the variable inverter.ADCOffsetCalibEnable in the model initialization script.
%  
% Alternatively, you can compute the ADC offset values and update it manually in the model initialization scripts. For instructions, see <docid:c2b_ug#mw_91be046f-02bb-4256-b09b-a4c9b86191a3 Open-loop example>.
% 
% *4.* Compute the quadrature encoder index offset value and update it in the model initialization scripts associated with the target model. For instructions, see <docid:c2b_ug#mw_6c933bed-960e-4698-b1f3-cba2e0f5b9a3 Quadrature Encoder Offset Calibration for PMSM Motor>.
% 
% *5.* Open the target model for the hardware configuration that you want to use. If you want to change the default hardware configuration settings for the model, see <docid:mcb_gs#mw_3286e9a5-4b65-4b84-9133-a92130b252bc Model Configuration
% Parameters for Sensors>.
%
% *6.* Click *Build, Deploy & Start* on the *Hardware* tab to deploy the target model to the hardware.
% 
% *7.* Click the *host model* hyperlink in the target model to open the associated host model. Open the <matlab:openExample('c2b/FieldOrientedControlOfPMSMWithQEPUsingC2000ProcessorsExample','supportingFile','mcb_pmsm_foc_host_model') mcb_pmsm_foc_host_model> host model.
%%
open_system('mcb_pmsm_foc_host_model.slx'); 
%%
% For details about the serial communication between the host and target
% models, see <docid:mcb_gs#mw_7d703f4b-0b29-4ec7-a42b-0b300f580ccc Communication between Host and Target>.
%  
% *8.* Set the parameter *Port* of the following blocks in the
% <matlab:mcb_pmsm_foc_host_model mcb_pmsm_foc_host_model> model to match the COM
% port of the host computer:
%
% * mcb_pmsm_foc_host_model > Host Serial Setup.
% * mcb_pmsm_foc_host_model > Serial Communication > Host Serial Receive.
% * mcb_pmsm_foc_host_model > Serial Communication > SCI_TX > Host Serial
% Transmit.
%
% *9.* Update the Reference Speed value in the host model.
% 
% *10.* Click *Run* on the *Simulation* tab to run the host model.
% 
% *11.* Change the position of the motor switch to Start, to start running the motor.
%  
% *12.* Observe the debug signals from serial communication block, in the Time Scope of host model.
%
%% More About:
%
% * <docid:mcb_gs#mw_7d703f4b-0b29-4ec7-a42b-0b300f580ccc Host communication Target>
%
% * <docid:mcb_gs#mw_46abe140-e03c-44a1-97a4-b88041db6a90 Estimate Motor Parameters by Using Motor Control Blockset Parameter Estimation Tool>
%
% * For F28069M control card, LAUNCHXL-F28069M controller and LAUNCHXL-F28379D controller, refer to <docid:mcb_gs#mw_4df1139b-472a-4ec8-91ee-0f0170cd4c2c MCB>
%
% * <docid:c2b_ug#mw_6c933bed-960e-4698-b1f3-cba2e0f5b9a3 Quadrature Encoder Offset Calibration for PMSM Motor>
