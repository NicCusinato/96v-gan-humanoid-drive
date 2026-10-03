function result = F280049C_Hardware_Map_Build()
%F280049C_HARDWARE_MAP_BUILD Build and deploy the 49C map test safely.
%
% The project folder may contain characters that GNU Make treats specially.
% Keep source files in the project folder, but put Simulink cache and code
% generation output in a temporary folder with a simple path.

root = fileparts(mfilename('fullpath'));
modelFile = fullfile(root, 'F280049C_Hardware_Map_Tests.slx');
[~, modelName] = fileparts(modelFile);

if ~isfile(modelFile)
    error('F280049C:MissingModel', 'Model not found: %s', modelFile);
end

if bdIsLoaded(modelName)
    if strcmp(get_param(modelName, 'Dirty'), 'on')
        error('F280049C:DirtyModel', ...
            'Save or close the open model before running this build wrapper.');
    end
    bdclose(modelName);
end

load_system(modelFile);
F280049C_Hardware_Map_Test_Select('QEP_SPI');

buildRoot = fullfile(tempdir, 'F280049C_Hardware_Map_codegen');
cacheFolder = fullfile(buildRoot, 'cache');
codeGenFolder = fullfile(buildRoot, 'codegen');
if ~isfolder(cacheFolder)
    mkdir(cacheFolder);
end
if ~isfolder(codeGenFolder)
    mkdir(codeGenFolder);
end

Simulink.fileGenControl('set', ...
    'CacheFolder', cacheFolder, 'CodeGenFolder', codeGenFolder);

oldFolder = pwd;
cleanup = onCleanup(@() cd(oldFolder)); %#ok<NASGU>
cd(buildRoot);
set_param(modelName, 'SimulationCommand', 'update');
slbuild(modelName);

result = struct('model', modelFile, 'codegen_folder', codeGenFolder);
fprintf('Build and deploy completed for %s.\n', modelName);
fprintf('Generated files: %s\n', codeGenFolder);
end
