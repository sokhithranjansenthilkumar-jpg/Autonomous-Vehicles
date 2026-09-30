function state = initialState()
%INITIALSTATE Initialise deterministic behaviour memory.
state = struct('name', "CRUISE", 'lastTransitionTimeS', 0, ...
    'clearSinceS', NaN);
end
