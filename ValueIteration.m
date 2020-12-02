function [ J_opt, u_opt_ind ] = ValueIteration(P, G)
%VALUEITERATION Value iteration
%   Solve a stochastic shortest path problem by Value Iteration.
%
%   [J_opt, u_opt_ind] = ValueIteration(P, G) computes the optimal cost and
%   the optimal control input for each state of the state space.
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
J_opt=ones(K,1);
u_opt_ind=ones(K,1);
J_opt_initial=zeros(K,1);
num=0;
 while(max(abs(J_opt-J_opt_initial))>1e-4)
     result=zeros(K,5);
     for i=1:5
         result(:,i)=G(:,i)+P(:,:,i)*J_opt_initial;
     end
     J_opt_initial=J_opt;
     [J_opt,u_opt_ind]=min(result,[],2);
     u_opt_ind(TERMINAL_STATE_INDEX)=5;
     num=num+1;
     if(num>2000)
         break
     end
 end
u_opt_ind(TERMINAL_STATE_INDEX)=5;

num
end