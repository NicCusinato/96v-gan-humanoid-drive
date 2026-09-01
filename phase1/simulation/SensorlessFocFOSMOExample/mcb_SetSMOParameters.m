function smo = mcb_SetSMOParameters()
% File          :   SMO encoder Parameter File
% Description   :   Parameters for Sliding Mode Observer by MCB

% Authors       :   A. Podder

% Copyright 2020 The MathWorks, Inc.
%%
% Get basic parameters
    smo.CurrentObsGain = 0.1;
    smo.SlidingSurfaceLim = 0.8;
    

end