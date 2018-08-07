function [R,S,T] = designRST(B,A,Hr,Hs,P)
% [R,S,T] = designRST(B,A,Hr,Hs,P) computes the coefficient of an RST
% controller which is designed via the pole placement method. Note that T
% is a scalar, thus the RST controller has the same performances on
% tracking and regulation. Once the contoller is designed the command
% signal can be computed as follows:
% u(t) = T*r(t) - R.* [y(t) ... y(t-nr)] + S.*[0 u(t-1) ... u(t-ns)] where 
% u is the command signal, y is the output and r is the reference signal.
%
% A and B are vectors representing the plant z transform of the form:
% G(z^-1) = B(z^-1) / A(z^-1). For instance, A and B can be used from a
% Discrete-time OE model as such A = dtoe_object.f; B = dtoe_object.b;
% For instance A = [1 0 -2 4] represents A(z^-1) = 1 -2*z^-2 + 4*z^-3.
%
% Hr and Hs can be specified to force a fixe term in R and S respectively.
% For instance, it might be needed to have an integrator in S which can be
% done with Hs = [1 -1] (Hs(z^-1) = 1 - z^-1). To reduce the noise at high
% frequencies, Hr = [1 1] (Hr(z^-1) = 1 + z^-1) can be useful.
%
% P is the characteristic polynomial which we want the closed-loop system
% to have. R,S and T are designed to drive the pole of the system to those
% of P in order to obtain the required performances. Usually, it is not
% necessary to place all the poles, but only two dominant poles so that the
% system behaves like a pseudo second-order system. Those dominant poles
% can be computed from requirements such as rise-time, overshoot,
% settling-time. For instance:
% Ts = 0.04; % sampling time (s)
% Tr = 0.5; % rise time (s)
% OS = 0.05; % overshoot (5%)
% xi = sqrt(log(OS)^2 / (pi^2 + log(OS)^2)); % damping ratio
% wn = (2.16*xi + 0.6) / Tr; % natural frequency
% p1 = -2*exp(-xi*wn*Ts)*cos(wn*Ts*sqrt(1-xi^2));
% p2 = exp(-2*xi*wn*Ts);
% P = [1 p1 p2]; % characteristic polynomial of 2nd order system which
% satisfies the requirements

%% Incorporate Hr and Hs in A and B
A = conv(A,Hs);
B = conv(B,Hr);

%% Obtain dimensions
na = size(A,2) - 1;

% find d
d=-1;
C=B;

while C(1) == 0
    d=d+1;
    C = C(2:end);
end

nb = size(B,2) - d - 1;

%% Build the sylveter matrix
sylvester = zeros(na+nb+d);

for i = 1:(nb+d)
    sylvester(i:(i + na), i) = A';
end

for i = (nb+d+1):(na+nb+d)
    sylvester((i - (nb + d)):i, i) = B';
end

%% Build p vector
p = zeros(na+nb+d, 1);
p(1:length(P)) = P;

% Find solution
x = sylvester \ p;

%% Extract R, S and T
ns = nb + d - 1;

S = x(1:(ns+1))';
S = conv(S, Hs);

R = x((ns+2):end)';
R = conv(R, Hr);

T = sum(R);

return