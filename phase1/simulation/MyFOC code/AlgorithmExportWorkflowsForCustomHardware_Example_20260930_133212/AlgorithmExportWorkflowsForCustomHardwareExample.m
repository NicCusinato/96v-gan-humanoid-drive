%% Algorithm-Export Workflows for Custom Hardware
% 
% This example enables you to use any custom motor-control hardware
% (hardware not used in the Motor Control Blockset(TM) examples) to run a
% three-phase permanent magnet synchronous motor (PMSM) using
% field-oriented control (FOC). Using the algorithm export workflows, which
% involve generating code for the control algorithm by using Simulink(R)
% and Embedded Coder(R) and then integrating it with either manually
% written or externally generated hardware driver code. This example
% explains the algorithm export workflows along with the intermediate
% steps.
% 
% The example uses the following hardware as a reference, but you can use
% any motor-control hardware:
% 
% * Controller: STMicroelectronics(R) STM32F302R8
% 
% * Inverter: STMicroelectronics(R) X-NUCLEO-IHM07M1
% 
% * Motor: BLY171D (includes quadrature encoder sensor)
% 
% You can use this example to customize the control algorithm and integrate
% it with the drivers for your motor-control hardware. In this example, we
% use the <https://www.st.com/en/development-tools/stm32cubemx.html |STM32 Cube MX|> software to configure and generate code for the hardware
% drivers. This example supports any three-phase PMSM.
% 
% Implementing the FOC algorithm needs real-time rotor position feedback. This example uses a quadrature encoder sensor to measure the rotor position. 
% For details about FOC, see <docid:mcb_gs#mw_2d6dd8ca-938f-44d3-8334-f5e42bf4b73b Field-Oriented
% Control (FOC)>.
% 
% The example includes three workflows.
% 
% *1.* *Open-loop control and ADC offset calibration* &mdash; This workflow uses an algorithm that runs a PMSM using open-loop control (also known as scalar control or Volts/Hz control).
% You can use this workflow to check the integrity of the hardware connections and calculate the ADC offsets for the current sensors available on the hardware.
% 
% *2.* *Quadrature encoder offset calibration* &mdash; This workflow uses an algorithm that calculates the offset between the |d|-axis of the rotor and the index pulse position as detected by the quadrature encoder sensor.
% The control algorithm (available in the field-oriented control workflow) needs this offset to accurately compute the rotor position, which is necessary to implement FOC.
% 
% *3.* *Field-oriented control* &mdash; This workflow uses an algorithm that runs a PMSM using closed-loop field-oriented control (FOC). The workflow uses the ADC and quadrature sensor offsets as inputs.
% 
% Each workflow includes these steps to prepare, deploy, and run the algorithm on your hardware:
% 
% *1.* Generate code for the control algorithm using Embedded Coder.
% 
% *2.* Obtain C Code For Hardware Drivers.
% 
% *3.* Integrate control algorithm code with the driver code.
% 
% *4.* Deploy the integrated code to hardware.
% 
% *5.* Control the motor using a host Simulink model.
% 

% Copyright 2021 The MathWorks, Inc.

%% Open MATLAB Project
% 
% Use one of these methods to open the MATLAB(R) project:
% 
% * Click *Open Project*.
% 
% * Run the following command to access the supporting files of the
% example and open the MATLAB project.
%%
% |openExample('mcb/AlgorithmExportWorkflowsForCustomHardwareExample');|
%%
% 
% The project contains three folders, one for each workflow required to run
% the final FOC algorithm. Each folder contains these contents: 
% 
% * Data script containing motor, inverter, and target hardware details.
% 
% * Algorithm model for generating code for the control algorithm. The
% generated code will be available in the folder |[project_root]/work/code|.
% 
% * Host model for communication with the target hardware.
% 
% * C code which shows how to integrate the generated algorithm code and the
% hardware driver code (specific to STM32F302R8 & X-NUCLEO-IHM07M1).
% 
% In addition to the three folders, the project also includes an |.IOC|
% file. You can use this file with STM32 Cube MX to configure the
% peripherals of the target and generate code. The |.IOC| file available in
% the project is specific to the STM32F302R8 and X-NUCLEO-IHM07M1 hardware.
% 
%% Workflows for Custom Hardware
% 
% Follow these workflows in sequence.
% 
% *1.* <docid:mcb_gs#mw_2c7d79e0-a0f0-4a08-8b59-df4cad488302 Workflow-1: Open-Loop Control and ADC Offset Calibration>
% 
% *2.* <docid:mcb_gs#mw_d8f6d480-303f-46a9-a8b1-53aa9e8aeb9b Workflow-2: Quadrature Encoder Offset Calibration>
% 
% *3.* <docid:mcb_gs#mw_bef649cb-2f13-4863-9b70-0616ef73be95 Workflow-3: Field-Oriented Control>
% 