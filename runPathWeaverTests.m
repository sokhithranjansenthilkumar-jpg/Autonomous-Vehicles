function results = runPathWeaverTests()
%RUNPATHWEAVERTESTS Run the complete non-interactive test suite.
setupPath();
results=runtests(fullfile(fileparts(mfilename('fullpath')),'tests'), ...
    'IncludeSubfolders',true);
disp(table(results));
assertSuccess(results);
end
