// PSIM Execution Script
set Vdc = 48.0; set fsw = 40000.0; set T_load = 0.00; set nm_ref = 91.6;
set K_d = 1.884956; set T_d = 0.001000;
set K_q = 2.764602; set T_q = 0.001467;
set K_w = 13.648406; set T_w = 0.007958;
set simcontrol snapshoot = 0;
set simcontrol data_saving = 1;
set graphfile "KneeRun_V48_F40000_T0.0_N91.6.txt";
run;
save "KneeRun_V48_F40000_T0.0_N91.6.txt";
