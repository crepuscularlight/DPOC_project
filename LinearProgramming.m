function [ J_opt, u_opt_ind ] = LinearProgramming(P, G)
%LINEARPROGRAMMING Linear Programming
%   Solve a stochastic shortest path problem by Linear Programming.
%
%   [J_opt, u_opt_ind] = LinearProgramming(P, G) computes the optimal cost
%   and the optimal control input for each state of the state space.
%
%   Input arguments:
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.
%
%       G:
%           A (K x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the cost if we are in state i and apply control
%           input l.
%
%   Output arguments:
%       J_opt:
%       	A (K x 1)-matrix containing the optimal cost-to-go for each
%       	element of the state space.
%
%       u_opt_ind:
%       	A (K x 1)-matrix containing the index of the optimal control
%       	input for each element of the state space. Mapping of the
%       	terminal state is arbitrary (for example: HOVER).

global K HOVER

%% Handle terminal state
% Do yo need to do something with the teminal state before starting policy
% iteration ?
global TERMINAL_STATE_INDEX
% IMPORTANT: You can use the global variable TERMINAL_STATE_INDEX computed
% in the ComputeTerminalStateIndex.m file (see main.m)
key=false;
if key==false
    b=[];
    A=[];
    f=-ones(K,1);

    Aeq=zeros(K,K);
    Aeq(TERMINAL_STATE_INDEX,TERMINAL_STATE_INDEX)=1;
    beq=zeros(K,1);
    lb=zeros(K,1);
    ub=1000*ones(K,1);
    for i=1:5
        b=[b;G(:,i)];
        A=[A;eye(K)-P(:,:,i)];

    end
    index=find(b==inf);
    A(index,:)=0;
    b(index)=0;

    [J_opt,tmp]=linprog(f,A,b,Aeq,beq,lb,ub);

    result=zeros(K,5);
    for i=1:5
        result(:,i)=G(:,i)+P(:,:,i)*J_opt;
    end
    [tmp,u_opt_ind]=min(result,[],2);



    u_opt_ind(TERMINAL_STATE_INDEX)=5;
else
    f=-ones(K,1);
    Aeq=zeros(K,K);
    Aeq(TERMINAL_STATE_INDEX,TERMINAL_STATE_INDEX)=1;
    beq=zeros(K,1);
    lb=zeros(K,1);
    ub=100*ones(K,1);
    A=zeros(5*K,K);
    b=zeros(1,5*K);
    for i=1:K
        if (i==TERMINAL_STATE_INDEX)
            continue
        end
 
            A(5*(i-1)+1:5*(i-1)+5,:)=-squeeze(P(i,:,:))';
            A(5*(i-1)+1:5*(i-1)+5,i)=A(5*(i-1)+1:5*(i-1)+5,i)+1;
            b(1,5*(i-1)+1:5*(i-1)+5)=G(i,:);
        end
        index=find(b==inf);
        A(index,:)=0;
        b(index)=0;
        [J_opt,tmp]=linprog(f,A,b,Aeq,beq,lb,ub);
        
        result=zeros(K,5);
        for i=1:5
            result(:,i)=G(:,i)+P(:,:,i)*J_opt;
        end
        [tmp,u_opt_ind]=min(result,[],2);
end


end

function y=calApproxiate(G,P)
global K
N=20;
y=eye(K);
for i=1:N
    y=y+mpower(P,i);
end
y=y*G;
end