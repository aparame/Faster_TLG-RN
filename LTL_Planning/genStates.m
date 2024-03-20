function[states] = genStates(gridChunk, gridWidth, gridLength)

states = [];

x = gridChunk(1):gridChunk(2);
nx = length(x);
y1 = ones(1,nx);


for y = gridChunk(3):gridChunk(4)
    
    states = [states, [x;y*y1]];   % states = [[10,11,12...90;11,11,11...]...[10,11,12,...90;51,51...51]]
    
end

states = gridPath2Cell(states(1,:),states(2,:),gridWidth,gridLength);
% states'