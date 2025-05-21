% Subspaces (images and kernels) of the Jacobian of a robot and their dimensions
% Example: planar 3R robot with equal (unitary) links
% by A. De Luca, first 25 Nov 2011 
% revised 3 Dec 2018 (run twice: for regular and singular cases)
% available in the course material of Robotics 1

clear all
clc

syms q1 q2 q3 real

disp('Subspace analysis (range and null spaces): R(J), N(J), R(J^T), N(J^T)')
disp('of the robot Jacobian (a mxn matrix) in a given configuration q')
disp(' ')
disp('robot: planar 3R arm with unitary links (n=3)')
disp('task: end-effector position in the plane (m=2)')
disp(' ')
disp('direct (task) kinematics')
p=[cos(q1) + cos(q1+q2) + cos(q1+q2+q3);
   sin(q1) + sin(q1+q2) + sin(q1+q2+q3)];
q=[q1 q2 q3]';

disp('Jacobian (a 2x3 matrix)')
Jac=jacobian(p,q)


% first case: a configuration that is regular

disp('REGULAR CASE: in the configuration q=(0, pi/2, pi/2)') %(0, pi/2, pi/4)
disp(' ')
disp('a) working with the Jacobian')
J=subs(Jac,{q1,q2,q3},{0,pi/2,pi/2}) %{0,pi/2,pi/4}


% second case: a configuration that is singular

% disp('SINGULAR CASE: in the configuration q=(pi/2, 0, pi)')
% disp(' ')
% disp('a) working with the Jacobian')
% J=subs(Jac,{q1,q2,q3},{pi/2,0, pi})
% pause

rankJ=rank(J)


NullSpaceJ=null(J)
dimNullSpaceJ=size(NullSpaceJ,2);
disp('normalizing...')
if dimNullSpaceJ>1,
   NullSpaceJ(:,1)=NullSpaceJ(:,1)/norm(NullSpaceJ(:,1));
   NullSpaceJ(:,2)=NullSpaceJ(:,2)/norm(NullSpaceJ(:,2));
   NullSpaceJ
else
   NullSpaceJ=NullSpaceJ/norm(NullSpaceJ)
end


dimNullSpaceJ=size(NullSpaceJ,2)


disp('check null-space joint velocity: J*NullSpaceJ ?')

simplify(J*NullSpaceJ)


RangeJ=orth(J)
%RangeJ=simplify(orth(J))


dimRangeSpaceJ=size(RangeJ,2)


disp('b) working with the Jacobian transpose')
JT=J'


rankJT=rank(JT)


NullSpaceJT=null(JT);
dimNullSpaceJT=size(NullSpaceJT,2);
if dimNullSpaceJT>0,
   NullSpaceJT=NullSpaceJT/norm(NullSpaceJT)
else
   NullSpaceJT
end


dimNullSpaceJT=size(NullSpaceJT,2)


RangeJT=orth(JT);
RangeJT=simplify(RangeJT)
dimRangeSpaceJT=size(RangeJT,2);
disp('de-normalizing...') %assuming that the first elements of the columns are non-zero, else...
RangeJT(:,1)=RangeJT(:,1)/RangeJT(1,1);
if dimRangeSpaceJT>1
   RangeJT(:,2)=RangeJT(:,2)/RangeJT(1,2);
end
RangeJT


dimRangeSpaceJT=size(RangeJT,2)


disp('c) final check on dimensions of subspaces')

disp(' ')
disp('dim Range(J) + dim Null(J^T) ?')
 
dimRangeSpaceJ+dimNullSpaceJT
disp('..equal to the task-space dimension')


disp(' ')
disp('dim Range(J^T) + dim Null(J) ?')

dimRangeSpaceJT+dimNullSpaceJ
disp('..equal to the joint-space dimension')

% end