function [ J_opt, u_opt_ind ] = PolicyIteration(P, G)
%POLICYITERATION Policy iteration
%   Solve a stochastic shortest path problem by Policy Iteration.
%
%   [J_opt, u_opt_ind] = PolicyIteration(P, G) computes the optimal cost and
%   the optimal control input for each state of the state space.
%
%   Input arguments:
%       P:
%           A (k x k x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.
%
%       G:
%           A (k x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the cost if we are in state i and apply control
%           input l.
%
%   Output arguments:
%       J_opt:
%       	A (k x 1)-matrix containing the optimal cost-to-go for each
%       	element of the state space.
%
%       u_opt_ind:
%       	A (k x 1)-matrix containing the index of the optimal control
%       	input for each element of the state space. Mapping of the
%       	terminal state is arbitrary (for example: HOVER).

global K HOVER

%% Handle terminal state
% Do yo need to do something with the teminal state before starting policy
% iteration?
global TERMINAL_STATE_INDEX
% IMPORTANT: You can use the global variable TERMINAL_STATE_INDEX computed
% in the ComputeTerminalStateIndex.m file (see main.m)
J_opt=zeros(K,1);
J_opt_1=ones(K,1);
u_opt_ind=ones(K,1);

u_opt_ind_initial=ones(K,1);
P_t=zeros(K,K);
G_t=zeros(K,1);
I=eye(K);

num=0;
J0=ones(K,1);
G_t=min(G,[],2);
while(max(abs(J_opt-J_opt_1))>1e-4)


    
    for j=1:5
        result(:,j)=G(:,j)+P(:,:,j)*J_opt;
    end
    [tmp,u_opt_ind]=min(result,[],2);
    
    
    for i=1:K
        P_t(i,:)=P(i,:,u_opt_ind(i));
        G_t(i)=G(i,u_opt_ind(i));
    end
    J_opt_1=J_opt;
    
    
    J_opt=linsolve(G_t,P_t,J_opt);
    num=num+1;
    if(num>500)
        break
    end
    
    
end
u_opt_ind(TERMINAL_STATE_INDEX)=5;
num


end



function y=linsolve(G,P,y0)
    N=50;
    I=eye(length(y0));
    
    for i=1:N-1
        I=I+mpower(P,i);
    end
    y=I*G+mpower(P,N)*y0;
end