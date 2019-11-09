clc
clear
%% Student Information
disp('Name : Aynala Anirudh');
disp('Student Number : 292932');
disp('Name : Kulunu Samarawickrama');
disp('Student Number : 293005');
disp('Name : Le Anh Phuong');
disp('Student Number : 269540');
%% R1 Transformation matrices between each links - 0T1-1T2-2T3-3T4
disp('R1: Transformation matrices between links');
syms q1 q2 d3 q4;
T01= trotx(0)*transl([0 0 0])*transl([0 0 0.8])*trotz(q1);
T12= trotx(0)*transl([0.5 0 0])*transl([0 0 0])*trotz(q2);
T23=trotx(-pi)*transl([0.6 0 0])*transl([0 0 d3])*trotz(0);
T34= trotx(pi)*transl([0 0 0])*transl([0 0 0])*trotz(q4);
T04= simplify(T01*T12*T23*T34)

%% R2  Homogenous transformation matrix for the robot when all the joint angles are set to zero
disp('R2: Homogenous Transformation with joints variables = 0');
HTOEF0=subs(T04,[q1 q2 d3 q4],[0 0 0 0]); 
HTOEF0=double(HTOEF0)
  
%% R3 Assigning Link parameters from the DH table and defining our Robot.
disp('R3: Modelling the robot by assigning Link Parameters using robotic toolbox');
L(1) = Link([0 0.8 0 0],'modified');
L(2) = Link([0 0 0.5 0],'modified');
L(3) = Link([0 0 0.6 -pi 1],'modified');
L(4) = Link([0 0 0 pi],'modified');
L(3).qlim = [0,1]; %Defining the range for our prismatic joint
Scara = SerialLink(L,'name','Scara Robot')

%% R4 Plotting the robot in Zero Position (all the joint variables are zero)
disp('R4: Scara Robot plot at zero position');
Scara.teach([0 0 0 0],'reach',2,'tilesize',2,'scale', 0.7);
title('R4: Scara Robot plot at zero position');
waitforbuttonpress

%% R5 Applying the tool transformation to the robot
disp('R5: Applying Tool transformation');
Scara.tool=transl(-0.03,0,-0.135)
Scara.plot([0 0 0 0],'reach',2,'tilesize',2,'scale', 0.7);
title('R5: Applying Tool transformation');
waitforbuttonpress

%% R6 Applying the Base Transformation to the robot as per base dimensions given.
disp('R6: Applying Base transformation');
Scara.base=transl(0.1,0.1,0.2)
Scara.plot([0 0 0 0],'reach',2,'tilesize',2,'scale', 0.7);
title('R6: Applying Base transformation');
waitforbuttonpress

%% R7: Robot in zero position
disp('R7: SCARA at zero position');
HTOEF0=Scara.fkine([0 0 0 0])%Homogeneous transformation matrix of End frame when Joint variables are zero.
Scara.teach([0 0 0 0],'reach',2,'tilesize',2,'scale', 0.7);
title('R7: SCARA at zero position')
waitforbuttonpress

%% R8 : Robot in offset position
disp('R8: SCARA at offset position');
HTEF1=Scara.fkine([pi/2 pi/2 0 0]) %%Homogeneous transformation matrix of End frame when Joint variables are set.
Scara.teach([pi/2 pi/2 0 0],'reach',2,'tilesize',2,'scale', 0.7);
title('R8: SCARA at offset position')
waitforbuttonpress

%% R9 Applying the Inverse Kinematics
disp('R9: Inverse kinematics for red and green pieces');

green=transl(0.6,0.6,0)*trotz(45,'deg');
igreen=Scara.ikine(green,'mask',[1 1 1 0 0 1])
Scara.teach(igreen,'reach',2,'tilesize',2,'scale', 0.7);
title('R9: Scara Robot at green object position')
waitforbuttonpress
hold on
title('R9: Plotting the green object position')
trplot(green,'frame','Green', 'color','g');
waitforbuttonpress
red=transl(0.3,0.8,0.125);
ired=Scara.ikine(red,'mask',[1 1 1 0 0 1])% Obtain the joint variables to reach red piece.
Scara.teach(ired,'reach',2,'tilesize',2,'scale', 0.7);
title('R9: Scara  Robot at red object position')
waitforbuttonpress
hold on
trplot(red,'frame','Red', 'color','r');
title('R9: Plotting the  red object position')
waitforbuttonpress

%% R10 Forward Kinematics
disp('R10: Forward kinematics using fkine');
fred=Scara.fkine(ired) %Calculate the position using forward kinematics with angles obtained above.
fgreen=Scara.fkine(igreen)

%% R11 Using Symbolic Toolbox to calculate the forward Kineamtics
disp('R11: Forward kinematics using symbolic toolbox')
Tbase=[1 0 0 0.1;
       0 1 0 0.1
       0 0 1 0.2
       0 0 0 1];
Ttool=[1 0 0 -0.03;
       0 1 0 0;
       0 0 1 -0.135
       0 0 0 1];
T=simplify(Tbase*T04*Ttool)
fgreensym=subs(T,[q1 q2 d3 q4],igreen);
fgreensym=double(fgreensym)
fredsym=subs(T,[q1 q2 d3 q4],ired);
fredsym=double(fredsym)