function G = ComputeStageCosts(stateSpace, map)
%COMPUTESTAGECOSTS Compute stage costs.
% 	Compute the stage costs for all states in the state space for all
%   control inputs.
%
%   G = ComputeStageCosts(stateSpace, map) 
%   computes the stage costs for all states in the state space for all
%   control inputs.
%
%   Input arguments:
%       stateSpace:
%           A (K x 3)-matrix, where the i-th row represents the i-th
%           element of the state space.
%
%       map:
%           A (M x N)-matrix describing the world. With
%           values: FREE TREE SHOOTER PICK_UP DROP_OFF BASE
%
%   Output arguments:
%       G:
%           A (K x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the expected stage cost if we are in state i and 
%           apply control input l.

    global GAMMA R P_WIND Nc
    global FREE TREE SHOOTER PICK_UP DROP_OFF BASE
    global NORTH SOUTH EAST WEST HOVER
    global K
    global TERMINAL_STATE_INDEX
    G=zeros(K,5);
    
    indexBase=ComputeValueIndex(stateSpace, map,BASE);
    indexShoot=ComputeValueIndex(stateSpace, map,SHOOTER);
    indexPick=ComputeValueIndex(stateSpace, map,PICK_UP);
    indexDrop=ComputeValueIndex(stateSpace, map,DROP_OFF);
    Control=[NORTH SOUTH  EAST WEST HOVER];
    for i=1:5
        
        G(:,i)=calCost(stateSpace,Control(i),indexShoot);
        
    end

    
end

function y=L1(stateSpace,index1,index2)
y=abs(stateSpace(index1,1)-stateSpace(index2,1))+abs(stateSpace(index1,2)-stateSpace(index2,2));
end

function index=stateTrans(stateSpace,indexNow,direction)
    
    global NORTH SOUTH EAST WEST HOVER
    global K 
    
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


function result=isHit(stateSpace,indexNow,direction)
indexNew=stateTrans(stateSpace,indexNow,direction);
if(indexNew==0)
    result=1;
else
    result=0;
end
end

function pr=shootPr(stateSpace,indexDrone,indexShoot)
global GAMMA R
L=length(indexShoot);
s=zeros(L/2);
for i=1:L/2
    s(i)=L1(stateSpace,indexDrone,indexShoot(2*i-1));
end


p=zeros(length(L/2));
for i=1:L/2
    if(s(i)>R)
        p(i)=0;
    else
        p(i)=GAMMA/(s(i)+1);
    end
end


% pr=1;
% for i=1:L/2
%     pr=pr*(1-p(i));
% end
% end


pr=0;
for i=1:L/2
    
    pr=p(i)+pr;

end
end

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

function p=prNotCrash(stateSpace,indexNow,direction,indexShoot)
global GAMMA R P_WIND Nc
global NORTH SOUTH EAST WEST HOVER
p=0;
indexNew=stateTrans(stateSpace,indexNow,direction);
if(indexNew~=0)
    %no wind
    P_SHOOT=shootPr(stateSpace,indexNew,indexShoot);
    p=p+(1-P_WIND)*(1-P_SHOOT);
            
    %wind
    indexNorth=stateTrans(stateSpace,indexNew,NORTH);
    indexSouth=stateTrans(stateSpace,indexNew,SOUTH);
    indexWest=stateTrans(stateSpace,indexNew,WEST);
    indexEast=stateTrans(stateSpace,indexNew,EAST);
    indexShift=[indexNorth,indexSouth,indexWest,indexEast];
    shiftDirection=[NORTH SOUTH WEST EAST];
    p_wind_accu=0;
    
    for j=1:4
        
        %make sure the drone do not collide with boundary or trees
        if(isHit(stateSpace,indexNew,shiftDirection(j))==0)
            P_SHOOT_SHIFT=shootPr(stateSpace,indexShift(j),indexShoot);
            p_wind_accu=p_wind_accu+P_WIND*(1-P_SHOOT_SHIFT)*0.25;
        end
        
    end
    p=p+p_wind_accu;

end
end

function stageCost=calCost(stateSpace,direction,indexShoot)
global NORTH SOUTH EAST WEST HOVER
global TERMINAL_STATE_INDEX
global Nc K
stageCost=ones(K,1);
for i=1:K
    if(i==TERMINAL_STATE_INDEX)
        stageCost(i)=0;
        TERMINAL_STATE_INDEX
    else
        if(isHit(stateSpace,i,direction))
            stageCost(i)=inf;
        else
            p_notcrash=prNotCrash(stateSpace,i,direction,indexShoot)
            stageCost(i,1)=1*p_notcrash+Nc*(1-p_notcrash);
        end
    end
end
end

