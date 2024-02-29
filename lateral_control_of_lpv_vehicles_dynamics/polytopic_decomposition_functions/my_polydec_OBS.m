function c = my_polydec_OBS(p,VERTX)
% This function returns the convex decomposition c of p over 
% the set of corners VERTX:
%
%       p = c1*VERTX(:,1) + ... + cn*VERTX(:,n)
%
%       cj >=0 ,              c1 + ... + cn = 1
%
% Here a constrained LQR-based observer is used for finding an admissible 
% solution c after a finite number of iterations according to a desired 
% error tolerance (default value equal to tol=1e-5).
%
% By John J. MARTINEZ MOLINA, GRENOBLE-INP, 12th November 2022.

n=size(VERTX,2); %number of columns of VERTX

%We set: y=M*x  ; x(k+1)=A*x(k) with A=eye(n).
y=[p; 1];  
m=size(y,1);
%
M=[VERTX;ones(1,n)]; %for including the constraint: c1 + ... + cn = 1
A=eye(n); %We assume stabisability of A.
L=mydlqr(A',M',eye(n),1e-3*eye(m))'; %check eig(A-L*M)

alpha=L*y; %initialisation

error=1; %the error will be computed as norm(y-M*alpha);    
tol=1e-5; %decired tolerance 

while error>tol
    alpha = (A-L*M)*alpha + L*y;
    
    % We apply a virtual force (as a disturbance) for respecting constraints.
    % i.e. alpha = A*alpha + L*(y-M*alpha)) - Q_constraint
    % Thus, to prevent possible alpha<0, we set:
    alpha=diag(real(alpha>0))*alpha;
    
    error=norm(y-M*alpha); %criteria to stop the iterations
end
c=alpha';

function [K]=mydlqr(A,B,Q,R) %Nested function
% This function computes de control gain K, such that
% u=-K*x stabilizes the system x(k+1)=A*x+B*u
% The matrices Q and R have to be diagonal and positive definite.
% John J. Martinez, Grenoble-INP, July 2016

P=Q; %
for i=1:1000
    Pold=P;
    P=A'*P*A+Q-A'*P*B*((B'*P*B+R)\B')*P*A;
    
    K=((B'*P*B+R)\B')*P*A;
    Kold=((B'*Pold*B+R)\B')*Pold*A;
    
    error2=norm(K-Kold)/norm(K);
    if error2<1e-12, break; end
end
K=((B'*P*B+R)\B')*P*A;
end

end %ending main function