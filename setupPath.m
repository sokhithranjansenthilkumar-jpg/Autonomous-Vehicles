function root = setupPath()
%SETUPPATH Add PathWeaver source and configuration folders to MATLAB path.
root = fileparts(mfilename('fullpath'));
addpath(root);
addpath(fullfile(root, 'matlab'));
addpath(fullfile(root, 'config'));
end
