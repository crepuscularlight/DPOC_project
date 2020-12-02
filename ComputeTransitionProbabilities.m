function P = ComputeTransitionProbabilities(stateSpace, map)
%COMPUTETRANSITIONPROBABILITIES Compute transition probabilities.
% 	Compute the transition probabilities between all states in the state
%   space for all control inputs.
%
%   P = ComputeTransitionProbabilities(stateSpace, map) 
%   computes the transition probabilities between all states in the state 
%   space for all control inputs.
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
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.

global GAMMA R P_WIND
global FREE TREE SHOOTER PICK_UP DROP_OFF BASE
global NORTH SOUTH EAST WEST HOVER
global K TERMINAL_STATE_INDEX
[M,N]=size(map);

P=zeros(K,K,5);
%find specific spot
indexBase=ComputeValueIndex(stateSpace, map,BASE);
indexShoot=ComputeValueIndex(stateSpace, map,SHOOTER);
indexPick=ComputeValueIndex(stateSpace, map,PICK_UP);
indexDrop=ComputeValueIndex(stateSpace, map,DROP_OFF);



%choose the control north=1  P[:,:,1]
Control=[NORTH SOUTH  EAST WEST HOVER];
for i=1:5
    
    P(:,:,i)=calProb(stateSpace,Control(i),indexPick,indexShoot,indexDrop,indexBase);
        
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


function index=indexPosCollision(stateSpace, map,direction)
    
    global  TREE 
    global NORTH SOUTH EAST WEST 
    global K 
    
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

function result=isHit(stateSpace,indexNow,direction)
indexNew=stateTrans(stateSpace,indexNow,direction);
if(indexNew==0)
    result=1;
else
    result=0;
end
end

function p=calProb(stateSpace,direction,indexPick,indexShoot,indexDrop,indexBase)
global GAMMA R P_WIND
global FREE TREE SHOOTER PICK_UP DROP_OFF BASE
global NORTH SOUTH EAST WEST HOVER
global K TERMINAL_STATE_INDEX

p=zeros(K,K);
for i=1:K
    %carrying no things
    if(mod(i,2)==1)
        indexNew=stateTrans(stateSpace,i,direction);
        complement=0;
        if(indexNew~=0)

            %no wind
             P_SHOOT=shootPr(stateSpace,indexNew,indexShoot);
            if(isPick(stateSpace,i,direction,indexPick))
                p(i,indexNew+1)=(1-P_WIND)*(1-P_SHOOT);
            else
             
                p(i,indexNew)=(1-P_WIND)*(1-P_SHOOT);
                if(indexNew==indexBase(1))
                    complement=p(i,indexNew);
                end
            end
            
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
                    if(isPick(stateSpace,indexNew,shiftDirection(j),indexPick))
                        p(i,indexShift(j)+1)=P_WIND*(1-P_SHOOT_SHIFT)*0.25;
                        p_wind_accu=p_wind_accu+p(i,indexShift(j)+1);
                    else
                        p(i,indexShift(j))=P_WIND*(1-P_SHOOT_SHIFT)*0.25;
                        p_wind_accu=p_wind_accu+p(i,indexShift(j));
                        if(indexShift(j)==indexBase(1))
                            complement=complement+p(i,indexShift(j));
                        end
                    end
                    
                end
            end
            %rest base probability

            p(i,indexBase(1))=1-p_wind_accu-(1-P_WIND)*(1-P_SHOOT)+complement;

            
        end
        
        
    else
        %carrying things
        if(i==TERMINAL_STATE_INDEX)
             p(i,i)=1;
        else
            indexNew=stateTrans(stateSpace,i,direction);
            if(indexNew~=0)

                %no wind
                 P_SHOOT=shootPr(stateSpace,indexNew,indexShoot);
                
                 p(i,indexNew)=(1-P_WIND)*(1-P_SHOOT);
              

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
                        
                        p(i,indexShift(j))=P_WIND*(1-P_SHOOT_SHIFT)*0.25;
                        p_wind_accu=p_wind_accu+p(i,indexShift(j));


                    end
                end
                %rest base probability
                p_wind_accu
                
                p(i,indexBase(1))=1-p_wind_accu-(1-P_WIND)*(1-P_SHOOT);
        end
        end
end

end
end