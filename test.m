% main.m
%
% Matlab script that calls all the functions for computing the optimal cost
% and policy of the given problem.
%
% Dynamic Programming and Optimal Control
% Fall 2020
% Programming Exercise
%
% --
% ETH Zurich
% Institute for Dynamic Systems and Control
%
% --

%% Clear workspace and command window
clear all;
close all;
clc;

%% Options
% [M, N]
mapSize = [15, 20];
% Set to true to generate a random map of size mapSize, else set to false 
% to load the pre-exsisting example map
generateRandomWorld = true;

% Plotting options
global PLOT_POLICY PLOT_COST
PLOT_POLICY = true;
PLOT_COST = false;

%% Global problem parameters
% IMPORTANT: Do not add or remove any global parameter in main.m
global GAMMA R Nc P_WIND
GAMMA  = 0.2; % Shooter gamma factor
R = 2; % Shooter range
Nc = 10; % Time steps required to bring drone to base when it crashes
P_WIND = 0.1; % Gust of wind probability

% IDs of elements in the map matrix
global FREE TREE SHOOTER PICK_UP DROP_OFF BASE 
FREE = 0;
TREE = 1;
SHOOTER = 2;
PICK_UP = 3;
DROP_OFF = 4;
BASE = 5;

% Index of each action in the P and G matrices. Use this ordering
global NORTH SOUTH EAST WEST HOVER
NORTH  = 1;
SOUTH = 2;
EAST = 3;
WEST = 4;
HOVER = 5;

%% Generate map
% map(m,n) represents the cell at indices (m,n) according to the axes
% specified in the PDF.
disp('Generate map');
if generateRandomWorld
	[map] = GenerateWorld(mapSize(1), mapSize(2));
else
    % We can load a pre-generated map.
    load('exampleWorld.mat');
end
MakePlots(map);

%% Generate state space
disp('Generate state space');
% Generate a (K x 3)-matrix 'stateSpace', where each accessible cell is
% represented by two rows (with and without carrying a package).
stateSpace = [];
for m = 1 : size(map, 1)
    for n = 1 : size(map, 2)
        if map(m, n) ~= TREE
            stateSpace = [stateSpace;
                          m, n, 0;
                          m, n, 1];
        end
    end
end
% State space size
global K
K=size(stateSpace,1);

%% Set the following to true as you progress with the files
transitionProbabilitiesImplemented = true;
stageCostsImplemented = false;
valueIterationImplemented = false; 
policyIterationImplemented = false;
linearProgrammingImplemented = false;

%% Compute the terminal state index
global TERMINAL_STATE_INDEX
if transitionProbabilitiesImplemented
    % TODO: Question a)
    TERMINAL_STATE_INDEX = ComputeTerminalStateIndex(stateSpace, map);
end                  
%% Compute transition probabilities
if transitionProbabilitiesImplemented
    disp('Compute transition probabilities');
    % Compute the transition probabilities between all states in the
    % state space for all control inputs.
    % The transition probability matrix has the dimension (K x K x L), i.e.
    % the entry P(i, j, l) representes the transition probability from state i
    % to state j if control input l is applied.
    
    % TODO: Question b)
    P = ComputeTransitionProbabilities(stateSpace, map);
    disp("probability successful")
%     P_exp=load('example_P.mat');
%     x=P-P_exp.P;
%     right=find(P_exp.P~=0);
%     error=find(abs(x)>0.00001);
%     [error_m,error_n]=find(abs(x(:,:,1))>0.0001);
end

%% Compute stage costs
if stageCostsImplemented 
    disp('Compute stage costs');
    % Compute the stage costs for all states in the state space for all
    % control inputs.
    % The stage cost matrix has the dimension (K x L), i.e. the entry G(i, l)
    % represents the cost if we are in state i and apply control input l.
    
    % TODO: Question c)
    G = ComputeStageCosts(stateSpace, map);
end

%% Solve stochastic shortest path problem
% Solve the stochastic shortest path problem by Value Iteration,
% Policy Iteration, and Linear Programming
if valueIterationImplemented
    disp('Solve stochastic shortest path problem with Value Iteration');
    
    % TODO: Question d)
    [ J_opt_vi, u_opt_ind_vi ] = ValueIteration(P, G);
    
    if size(J_opt_vi,1)~=K || size(u_opt_ind_vi,1)~=K
        disp('[ERROR] the size of J and u must be K')
    end
end
if policyIterationImplemented
    disp('Solve stochastic shortest path problem with Policy Iteration');
    
    % TODO: Question d)
    [ J_opt_pi, u_opt_ind_pi ] = PolicyIteration(P, G);
    
    if size(J_opt_pi,1)~=K || size(u_opt_ind_pi,1)~=K
        disp('[ERROR] the size of J and u must be K')
    end
end
if linearProgrammingImplemented
    disp('Solve stochastic shortest path problem with Linear Programming');
    
    % TODO: Question d)
    [ J_opt_lp, u_opt_ind_lp ] = LinearProgramming(P, G);
    
    if size(J_opt_lp,1)~=K || size(u_opt_ind_lp,1)~=K
        disp('[ERROR] the size of J and u must be K')
    end
end

%% Plot results
disp('Plot results');
if valueIterationImplemented
    MakePlots(map, stateSpace, J_opt_vi, u_opt_ind_vi, 'Value iteration');
end
if policyIterationImplemented
    MakePlots(map, stateSpace, J_opt_pi, u_opt_ind_pi, 'Policy iteration');
end
if linearProgrammingImplemented
    MakePlots(map, stateSpace, J_opt_lp, u_opt_ind_lp, 'Linear programming');
end

 %% Terminated
disp('Terminated');

%% TEST
% index=indexPosCollision(stateSpace,map,EAST)
% stateSpace(index,:)

% valueIndex = ComputeValueIndex(stateSpace, map,SHOOTER)
% valueIndex = ComputeValueIndex(stateSpace, map,PICK_UP)
% stateSpace(valueIndex,:)

% y=L1(stateSpace,1,100)
%pr=shootPr(stateSpace,[57],[3 4 17 18 55 56 89 90 101 102]);

% index=stateTrans(stateSpace,40,NORTH)
% indexPick= ComputeValueIndex(stateSpace, map,PICK_UP)
% indexNow=stateTrans(stateSpace,indexPick(1),EAST)
% result=isPick(stateSpace,indexNow,WEST,indexPick)

%index=stateTrans(stateSpace,40,NORTH)
% indexDrop= ComputeValueIndex(stateSpace, map,DROP_OFF)
% index1=stateTrans(stateSpace,indexDrop(2),NORTH)
% result=isDrop(stateSpace,index1,EAST,indexDrop)

% indexDrop= ComputeValueIndex(stateSpace, map,PICK_UP)
% index1=stateTrans(stateSpace,indexDrop(2),NORTH)
% result=isDrop(stateSpace,index1,EAST,indexDrop)

% indexNow=stateTrans(stateSpace,indexDrop(2),EAST)
% result=isDrop(stateSpace,indexNow-1,WEST,indexDrop)

% index=indexPosCollision(stateSpace,map,WEST)
% result=isHit(stateSpace,index(4),SOUTH)

% indexBase=ComputeValueIndex(stateSpace, map,BASE);
% indexShoot=ComputeValueIndex(stateSpace, map,SHOOTER);
% indexPick=ComputeValueIndex(stateSpace, map,PICK_UP);
% indexDrop=ComputeValueIndex(stateSpace, map,DROP_OFF);
% p=calProb(stateSpace,NORTH,indexPick,indexShoot,indexDrop,indexBase)

function valueIndex = ComputeValueIndex(stateSpace, map,value)
    global K
    [M,N]=size(map);
    [value_m,value_n]=find(map==value);
    valueIndex=[];
    for i=1:K
        if(any(ismember([value_m,value_n],[stateSpace(i,1),stateSpace(i,2)],'rows'))==1)
                valueIndex=[valueIndex;i];
        end
    end
    
end
function index=indexPosCollision(stateSpace, map,direction)
    
    global FREE TREE SHOOTER PICK_UP DROP_OFF BASE
    global NORTH SOUTH EAST WEST HOVER
    global K TERMINAL_STATE_INDEX
    
    [M,N]=size(map);
    [tree_m,tree_n]=find(map==TREE);
    tree_pos=[tree_m tree_n];
    
    if(direction==NORTH)
        index=find(stateSpace(:,2)==N);
        for i=1:K
            if(any(ismember(tree_pos,[stateSpace(i,1),stateSpace(i,2)+1],'rows'))==1)
                index=[index;i];
            end
        end
    end
    
    if(direction==SOUTH)
        index=find(stateSpace(:,2)==1);
        for i=1:K
            if(any(ismember(tree_pos,[stateSpace(i,1),stateSpace(i,2)-1],'rows'))==1)
                index=[index;i];
            end
        end
    end

    if(direction==EAST)
        index=find(stateSpace(:,1)==M);
        for i=1:K
            if(any(ismember(tree_pos,[stateSpace(i,1)+1,stateSpace(i,2)],'rows'))==1)
                index=[index;i];
            end
        end
    end
    
    if(direction==WEST)
        index=find(stateSpace(:,1)==1);
        for i=1:K
            if(any(ismember(tree_pos,[stateSpace(i,1)-1,stateSpace(i,2)+1],'rows'))==1)
                index=[index;i];
            end
        end
    end

end


function y=L1(stateSpace,index1,index2)
y=abs(stateSpace(index1,1)-stateSpace(index2,1))+abs(stateSpace(index1,2)-stateSpace(index2,2));
end


function pr=shootPr(stateSpace,indexDrone,indexShoot)
global GAMMA R
L=length(indexShoot);
s=zeros(L/2);
for i=1:L/2
    s(i)=L1(stateSpace,indexDrone,indexShoot(2*i-1));
end
s;

p=zeros(length(L/2));
for i=1:L/2
    if(s(i)>R)
        p(i)=0;
    else
        p(i)=GAMMA/(s(i)+1);
    end
end
pr=1;
for i=1:L/2
    pr=pr*(1-p(i));
end
pr=1-pr;

end

function index=stateTrans(stateSpace,indexNow,direction)
    global FREE TREE SHOOTER PICK_UP DROP_OFF BASE
    global NORTH SOUTH EAST WEST HOVER
    global K TERMINAL_STATE_INDEX
    
    m=stateSpace(indexNow,1);
    n=stateSpace(indexNow,2);
    q=stateSpace(indexNow,3);
    
    if(direction==NORTH)
        [m,n+1];
        for i=1:K
            if(any(ismember([m,n+1,q],[stateSpace(i,1),stateSpace(i,2),stateSpace(i,3)],'rows'))==1)
                index=i;
                break;
            else
                index=0;
            end
        end
    elseif(direction==SOUTH)
        [m,n-1];
        for i=1:K
            if(any(ismember([m,n-1,q],[stateSpace(i,1),stateSpace(i,2),stateSpace(i,3)],'rows'))==1)
                index=i;
                break;
            else
                index=0;
            end
        end
    elseif(direction==WEST)
        [m-1,n];
        for i=1:K
            if(any(ismember([m-1,n,q],[stateSpace(i,1),stateSpace(i,2),stateSpace(i,3)],'rows'))==1)
                index=i;
                break;
            else
                index=0;
            end
        end        
    elseif(direction==EAST)
        [m+1,n];
        for i=1:K
            if(any(ismember([m+1,n,q],[stateSpace(i,1),stateSpace(i,2),stateSpace(i,3)],'rows'))==1)
                index=i;
                break;
            else
                index=0;
            end
        end        
    elseif(direction==HOVER)
        index=indexNow;
    end

end

function result=isPick(stateSpace,indexNow,direction,indexPick)
    m=stateSpace(indexNow,1);
    n=stateSpace(indexNow,2);
    q=stateSpace(indexNow,3);
    
    indexNew=stateTrans(stateSpace,indexNow,direction);
 
    indexPick=indexPick(1);
    if(indexNew==indexPick)
        result= 1;
    else
        result=0;
        

    end




end

function result=isDrop(stateSpace,indexNow,direction,indexDrop)
    indexNew=stateTrans(stateSpace,indexNow,direction);
 
    indexDrop=indexDrop(2);
    if(indexNew==indexDrop)
        result= 1;
    else
        result=0;
    end
end

% function result=isHit(stateSpace,indexNow,direction)
% indexNew=stateTrans(stateSpace,indexNow,direction);
% if(indexNew==0)
%     result=1;
% else
%     result=0;
% end
% end
% 
% function p=calProb(stateSpace,direction,indexPick,indexShoot,indexDrop,indexBase)
% global GAMMA R P_WIND
% global FREE TREE SHOOTER PICK_UP DROP_OFF BASE
% global NORTH SOUTH EAST WEST HOVER
% global K TERMINAL_STATE_INDEX
% 
% p=zeros(K,K);
% for i=1:K
%     %carrying no things
%     if(mod(i,2)==1)
%         indexNew=stateTrans(stateSpace,i,direction);
%         if(indexNew~=0)
% 
%             %no wind
%              P_SHOOT=shootPr(stateSpace,indexNew,indexShoot);
%             if(isPick(stateSpace,i,direction,indexPick))
%                 p(i,indexNew+1)=(1-P_WIND)*(1-P_SHOOT);
%             else
%                 p(i,indexNew)=(1-P_WIND)*(1-P_SHOOT);
%             end
%             
%             %wind
%             indexNorth=stateTrans(stateSpace,indexNew,NORTH);
%             indexSouth=stateTrans(stateSpace,indexNew,SOUTH);
%             indexWest=stateTrans(stateSpace,indexNew,WEST);
%             indexEast=stateTrans(stateSpace,indexNew,EAST);
%             indexShift=[indexNorth,indexSouth,indexWest,indexEast];
%             shiftDirection=[NORTH SOUTH WEST EAST];
%             p_wind_accu=0;
%             
%             for j=1:4
%                 
%                 %make sure the drone do not collide with boundary or trees
%                 if(isHit(stateSpace,indexNew,shiftDirection(j))==0)
%                     P_SHOOT_SHIFT=shootPr(stateSpace,indexShift(j),indexShoot);
%                     if(isPick(stateSpace,indexNew,shiftDirection(j),indexPick))
%                         p(i,indexShift(j)+1)=P_WIND*(1-P_SHOOT_SHIFT)*0.25;
%                         p_wind_accu=p_wind_accu+p(i,indexShift(j)+1);
%                     else
%                         p(i,indexShift(j))=P_WIND*(1-P_SHOOT_SHIFT)*0.25;
%                         p_wind_accu=p_wind_accu+p(i,indexShift(j));
%                     end
%                     
%                 end
%             end
%             %rest base probability
%             p(i,indexBase(1))=1-p_wind_accu-(1-P_WIND)*(1-P_SHOOT);
%             
%         end
%         
%         
%     else
%         %carrying things
%         if(i==TERMINAL_STATE_INDEX)
%             p(i,i)=1;
%         else
%             indexNew=stateTrans(stateSpace,i,direction);
%             if(indexNew~=0)
% 
%                 %no wind
%                  P_SHOOT=shootPr(stateSpace,indexNew,indexShoot);
%                 if(isDrop(stateSpace,i,direction,indexDrop))
%                     p(i,indexNew+1)=(1-P_WIND)*(1-P_SHOOT);
%                 else
%                     p(i,indexNew)=(1-P_WIND)*(1-P_SHOOT);
%                 end
% 
%                 %wind
%                 indexNorth=stateTrans(stateSpace,indexNew,NORTH);
%                 indexSouth=stateTrans(stateSpace,indexNew,SOUTH);
%                 indexWest=stateTrans(stateSpace,indexNew,WEST);
%                 indexEast=stateTrans(stateSpace,indexNew,EAST);
%                 indexShift=[indexNorth,indexSouth,indexWest,indexEast];
%                 shiftDirection=[NORTH SOUTH WEST EAST];
%                 p_wind_accu=0;
% 
%                 for j=1:4
% 
%                     %make sure the drone do not collide with boundary or trees
%                     if(isHit(stateSpace,indexNew,shiftDirection(j))==0)
%                         P_SHOOT_SHIFT=shootPr(stateSpace,indexShift(j),indexShoot);
%                         if(isDrop(stateSpace,indexNew,shiftDirection(j),indexDrop))
%                             p(i,indexShift(j))=P_WIND*(1-P_SHOOT_SHIFT)*0.25;
%                             p_wind_accu=p_wind_accu+p(i,indexShift(j)+1);
%                         else
%                             p(i,indexShift(j))=P_WIND*(1-P_SHOOT_SHIFT)*0.25;
%                             p_wind_accu=p_wind_accu+p(i,indexShift(j));
%                         end
% 
%                     end
%                 end
%                 %rest base probability
%                 p(i,indexBase(1))=1-p_wind_accu-(1-P_WIND)*(1-P_SHOOT);
%             end
%         end
%     
% end
% 
% end
% end