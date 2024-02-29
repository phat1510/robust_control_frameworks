function [alpha, vertx] = my_polydec(p,RANGE)
%This function returns the convex decomposition c of p over 
% the set VERTX of box corners:
%
%       p = c1*VERTX(:,1) + ... + cn*VERTX(:,n)
%
%       cj >=0 ,              c1 + ... + cn = 1
%
%
% The RANGE matrix is composed by the min and max values of entries of 
% vector p, such that RANGE(:,1) <= p <= RANGE(:,2)

%Box which contains the varying parameter p
minval=RANGE(:,1);
maxval=RANGE(:,2);

%dimension of varying parameter p
np=size(p,1); 

%p=rho;

vertx=[minval(1),maxval(1)];
c=1;
  for j=1:np
     t=(p(j)-minval(j))/(maxval(j)-minval(j));
     if t < 0 | t > 1
        error(sprintf('P(%d) is not in the specified range',j));
     end
     c=[c*(1-t) , c*t];
     
     %Building the vertex matrix:
     if j>1
     ls=size(vertx,2);
     vertx=[vertx vertx;minval(j)*ones(1,ls) maxval(j)*ones(1,ls)];
     end
  end
  alpha=c; %The obtained alpha vector such that vertx*alpha'=rho