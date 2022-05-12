%% MAE 465 Final Project
% 5/11/2022
% 
%Ethan Covington
%
%
%
%
%% Setting up intital Values
clear;clc;
L = 0.3; %m
L_inc = sqrt(2)*L;
E = 100*10^9;%Pascals
A = 1/(100^2); %m^2

%Initialize Global Stiffness Matrix;
K_Global = zeros(28);

%% Problem 1


%Determining local stiffness matrices for bottom Horizontal elements
%angle is 0
%Location assignment for Bottom Horizontal Elements node 1 is bottom/left
%and node 2 is top/right
offset =0;
for elementCounter = 1:6
   element(elementCounter).K = StiffnessPlaneTruss(E,A,L,0); %User-Defined funtion to calculate K matrix in Plan Truss
   element(elementCounter).node1 = [((elementCounter-1)*0.3) 0]; %x = numNode*0.3 y = 0
   element(elementCounter).node2 = [(((elementCounter-1)*0.3)+0.3) 0]; %x = (numNode*0.3) +0.3 y =0
   
   %Inserting K for element n into K_Global
   %Top Left
   for rowCounter = 1:2
       for columnCounter =1:2

      K_Global((rowCounter+offset), (columnCounter+offset)) =  K_Global((rowCounter+offset), (columnCounter+offset))+element(elementCounter).K(rowCounter,columnCounter);
       end
   end
   %Top Right
   for rowCounter = 1:2
       for columnCounter =3:4

      K_Global((rowCounter+offset), (columnCounter+2+offset)) = K_Global((rowCounter+offset), (columnCounter+2+offset))+ element(elementCounter).K(rowCounter,columnCounter);
       end
   end
%    %Bottom Left
   for rowCounter = 3:4
       for columnCounter =1:2

      K_Global((rowCounter+2+offset), (columnCounter+offset)) = K_Global((rowCounter+2+offset), (columnCounter+offset))+ element(elementCounter).K(rowCounter,columnCounter);
       end
   end
%    %Bottom Right
   for rowCounter = 3:4
       for columnCounter =3:4

      K_Global((rowCounter+2+offset), (columnCounter+2+offset)) = K_Global((rowCounter+2+offset), (columnCounter+2+offset))+ element(elementCounter).K(rowCounter,columnCounter);
       end
   end
    offset = offset+4;
end


%Determining local stiffness matrices for Top Horizontal elements
%angle is 0
%Location assignment for Bottom Horizontal Elements node 1 is bottom/left
%and node 2 is top/right
offset = 0;
for elementCounter = 7:12
   element(elementCounter).K = StiffnessPlaneTruss(E,A,L,0); %User-Defined funtion to calculate K matrix in Plan Truss
   element(elementCounter).node1 = [((elementCounter-1)*0.3) 0.3]; %x = numNode*0.3 y = 0
   element(elementCounter).node2 = [(((elementCounter-1)*0.3)+0.3) 0.3]; %x = (numNode*0.3) +0.3 y =0
       %Inserting K for element n into K_Global
       %Top Left
       for rowCounter = 1:2
           for columnCounter =1:2
               
               K_Global((rowCounter+offset+2), (columnCounter+offset+2)) =  K_Global((rowCounter+offset+2), (columnCounter+offset+2))+element(elementCounter).K(rowCounter,columnCounter);
           end
       end
       %Top Right
       for rowCounter = 1:2
           for columnCounter =3:4
               
               K_Global((rowCounter+offset+2), (columnCounter+2+offset+2)) = K_Global((rowCounter+offset+2), (columnCounter+2+offset+2))+ element(elementCounter).K(rowCounter,columnCounter);
           end
       end
       %    %Bottom Left
       for rowCounter = 3:4
           for columnCounter =1:2
               
               K_Global((rowCounter+2+offset+2), (columnCounter+offset+2)) = K_Global((rowCounter+2+offset+2), (columnCounter+offset+2))+ element(elementCounter).K(rowCounter,columnCounter);
           end
       end
       %    %Bottom Right
       for rowCounter = 3:4
           for columnCounter =3:4
               
               K_Global((rowCounter+2+offset+2), (columnCounter+2+offset+2)) = K_Global((rowCounter+2+offset+2), (columnCounter+2+offset+2))+ element(elementCounter).K(rowCounter,columnCounter);
           end
       end
    offset = offset+4;
    
end

%Determining local stiffness matrices for Vertical elements
%angle is 90
%Location assignment for Bottom Horizontal Elements node 1 is bottom/left
%and node 2 is top/right
offset =0;
for elementCounter = 13:19
   element(elementCounter).K = StiffnessPlaneTruss(E,A,L,90); %User-Defined funtion to calculate K matrix in Plan Truss
   element(elementCounter).node1 = [((elementCounter-1)*0.3) 0]; %x = numNode*0.3 y = 0
   element(elementCounter).node2 = [(((elementCounter-1)*0.3)+0.3) 0.3]; %x = (numNode*0.3) +0.3 y =0
   
   %Inserting K for element n into K_Global
   %Top Left
   for rowCounter = 1:2
       for columnCounter =1:2

      K_Global((rowCounter+offset), (columnCounter+offset)) =  K_Global((rowCounter+offset), (columnCounter+offset))+element(elementCounter).K(rowCounter,columnCounter);
       end
   end
   %Top Right
   for rowCounter = 1:2
       for columnCounter =3:4

      K_Global((rowCounter+offset), (columnCounter+offset)) = K_Global((rowCounter+offset), (columnCounter+offset))+ element(elementCounter).K(rowCounter,columnCounter);
       end
   end
%    %Bottom Left
   for rowCounter = 3:4
       for columnCounter =1:2

      K_Global((rowCounter+offset), (columnCounter+offset)) = K_Global((rowCounter+offset), (columnCounter+offset))+ element(elementCounter).K(rowCounter,columnCounter);
       end
   end
%    %Bottom Right
   for rowCounter = 3:4
       for columnCounter =3:4

      K_Global((rowCounter+offset), (columnCounter+offset)) = K_Global((rowCounter+offset), (columnCounter+offset))+ element(elementCounter).K(rowCounter,columnCounter);
       end
   end
    offset = offset+4;
end

%Determining local stiffness matrices for Upward(left to right) elements
%angle is 45
%Location assignment for Bottom Horizontal Elements node 1 is bottom/left
%and node 2 is top/right
offset=0;
for elementCounter = 20:25
   element(elementCounter).K = StiffnessPlaneTruss(E,A,L_inc,45); %User-Defined funtion to calculate K matrix in Plan Truss
   element(elementCounter).node1 = [((elementCounter-1)*0.3) 0.3]; %x = numNode*0.3 y = 0
   element(elementCounter).node2 = [(((elementCounter-1)*0.3)+0.3) 0]; %x = (numNode*0.3) +0.3 y =0
   
   %Inserting K for element n into K_Global
   %Top Left
   for rowCounter = 1:2
       for columnCounter =1:2

      K_Global((rowCounter+offset), (columnCounter+offset)) =  K_Global((rowCounter+offset), (columnCounter+offset))+element(elementCounter).K(rowCounter,columnCounter);
       end
   end
   %Top Right
   for rowCounter = 1:2
       for columnCounter =3:4

      K_Global((rowCounter+offset), (columnCounter+offset+4)) = K_Global((rowCounter+offset), (columnCounter+offset+4))+ element(elementCounter).K(rowCounter,columnCounter);
       end
   end
%    %Bottom Left
   for rowCounter = 3:4
       for columnCounter =1:2

      K_Global((rowCounter+offset+4), (columnCounter+offset)) = K_Global((rowCounter+offset+4), (columnCounter+offset))+ element(elementCounter).K(rowCounter,columnCounter);
       end
   end
%    %Bottom Right
   for rowCounter = 3:4
       for columnCounter =3:4

      K_Global((rowCounter+offset+4), (columnCounter+offset+4)) = K_Global((rowCounter+offset+4), (columnCounter+offset+4))+ element(elementCounter).K(rowCounter,columnCounter);
       end
   end
    offset = offset+4;
end


%Determining local stiffness matrices for Downward(left to right) elements
%angle is 135
%Location assignment for Bottom Horizontal Elements node 1 is bottom/left
%and node 2 is top/right
offset = 0;
for elementCounter = 26:31
   element(elementCounter).K = StiffnessPlaneTruss(E,A,L_inc,135); %User-Defined funtion to calculate K matrix in Plan Truss
   element(elementCounter).node1 = [((elementCounter-1)*0.3) 0.3]; %x = numNode*0.3 y = 0
   element(elementCounter).node2 = [(((elementCounter-1)*0.3)+0.3) 0]; %x = (numNode*0.3) +0.3 y =0
   
   %Inserting K for element n into K_Global
   %Top Left
   for rowCounter = 1:2
       for columnCounter =1:2

      K_Global((rowCounter+offset+2), (columnCounter+offset+2)) =  K_Global((rowCounter+offset+2), (columnCounter+offset+2))+element(elementCounter).K(rowCounter,columnCounter);
       end
   end
   %Top Right
   for rowCounter = 1:2
       for columnCounter =3:4

      K_Global((rowCounter+offset+2), (columnCounter+offset+2)) = K_Global((rowCounter+offset+2), (columnCounter+offset+2))+ element(elementCounter).K(rowCounter,columnCounter);
       end
   end
%    %Bottom Left
   for rowCounter = 3:4
       for columnCounter =1:2

      K_Global((rowCounter+offset+2), (columnCounter+offset+2)) = K_Global((rowCounter+offset+2), (columnCounter+offset+2))+ element(elementCounter).K(rowCounter,columnCounter);
       end
   end
%    %Bottom Right
   for rowCounter = 3:4
       for columnCounter =3:4

      K_Global((rowCounter+offset+2), (columnCounter+offset+2)) = K_Global((rowCounter+offset+2), (columnCounter+offset+2))+ element(elementCounter).K(rowCounter,columnCounter);
       end
   end
    offset = offset+4;
end
%% Setting Boundary Conditions

%u1 = v1 = v2 =0

K_Global(:,1) = [];
K_Global(1,:) = [];
K_Global(:,2) = [];
K_Global(:,2) = [];
K_Global(2,:) = [];
K_Global(2,:) = [];

%% Matrix for elemental Forces
F_0deg = [-cosd(0) -sind(0) cosd(0) sind(0)];
F_90deg  = [-cosd(90) -sind(90) cosd(90) sind(90)];
F_45deg  = [-cosd(45) -sind(45) cosd(45) sind(45)];
F_135deg  = [-cosd(135) -sind(135) cosd(135) sind(135)];

%% Case A
F_A = [0; 0;0; 0;0; 0;0; 0;0; 0;0; 0;0; 0;0; 0;0; 0;0; 0;0; 10000;0; 10000;0;];
%Deflection Matrix
d_A = K_Global^(-1)*F_A;
%Deflection MAtrix adjusted to include 0 value boundaries
d_A = [0;d_A(1);0;0;d_A(2:end)];
%Bottom Horizontal Section
for elementCounter = 1:6
    caseA(elementCounter).Elemental_Forces_A = ((A*E)/L)*F_0deg*[d_A(elementCounter); d_A(elementCounter+1);d_A(elementCounter+4);d_A(elementCounter+5);];
end
for elementCounter = 7:12
    caseA(elementCounter).Elemental_Forces_A = ((A*E)/L)*F_0deg*[d_A(elementCounter-4); d_A(elementCounter-3);d_A(elementCounter);d_A(elementCounter+1);];
end
for elementCounter = 13:19
    caseA(elementCounter).Elemental_Forces_A = ((A*E)/L)*F_90deg*[d_A(elementCounter-12); d_A(elementCounter-11);d_A(elementCounter-10);d_A(elementCounter-9);];
end
for elementCounter = 20:25
    caseA(elementCounter).Elemental_Forces_A = ((A*E)/L_inc)*F_45deg*[d_A(elementCounter-19); d_A(elementCounter-18);d_A(elementCounter-13);d_A(elementCounter-12);];
end
for elementCounter = 26:31
    caseA(elementCounter).Elemental_Forces_A = ((A*E)/L_inc)*F_135deg*[d_A(elementCounter-23); d_A(elementCounter-22);d_A(elementCounter-21);d_A(elementCounter-20);];
end
%% Case B
F_B = [0; 0;0; 0;0; 0;0; 0;0; 0;0; 0;0; 0;0; 0;0; 0;0; 0;0; 0;10000; 0;10000;];
%Deflection Matrix
d_B = K_Global^(-1)*F_B;
%Deflection MAtrix adjusted to include 0 value boundaries
d_B = [0;d_B(1);0;0;d_B(2:end)];
for elementCounter = 1:6
    caseB(elementCounter).Elemental_Forces_B = ((A*E)/L)*F_0deg*[d_B(elementCounter); d_B(elementCounter+1);d_B(elementCounter+4);d_B(elementCounter+5);];
end
for elementCounter = 7:12
    caseB(elementCounter).Elemental_Forces_B = ((A*E)/L)*F_0deg*[d_B(elementCounter-4); d_B(elementCounter-3);d_B(elementCounter);d_B(elementCounter+1);];
end
for elementCounter = 13:19
    caseB(elementCounter).Elemental_Forces_B = ((A*E)/L)*F_90deg*[d_B(elementCounter-12); d_B(elementCounter-11);d_B(elementCounter-10);d_B(elementCounter-9);];
end
for elementCounter = 20:25
    caseB(elementCounter).Elemental_Forces_B = ((A*E)/L_inc)*F_45deg*[d_B(elementCounter-19); d_B(elementCounter-18);d_B(elementCounter-13);d_B(elementCounter-12);];
end
for elementCounter = 26:31
    caseB(elementCounter).Elemental_Forces_B = ((A*E)/L_inc)*F_135deg*[d_B(elementCounter-23); d_B(elementCounter-22);d_B(elementCounter-21);d_B(elementCounter-20);];
end
%% Case C
F_C = [0; 0;0; 0;0; 0;0; 0;0; 0;0; 0;0; 0;0; 0;0; 0;0; 0;0; 10000;0; -10000;0;];
%Deflection Matrix
d_C = K_Global^(-1)*F_C;
%Deflection MAtrix adjusted to include 0 value boundaries
d_C = [0;d_C(1);0;0;d_C(2:end)];
for elementCounter = 1:6
    caseC(elementCounter).Elemental_Forces_C = ((A*E)/L)*F_0deg*[d_C(elementCounter); d_C(elementCounter+1);d_C(elementCounter+4);d_C(elementCounter+5);];
end
for elementCounter = 7:12
    caseC(elementCounter).Elemental_Forces_C = ((A*E)/L)*F_0deg*[d_C(elementCounter-4); d_C(elementCounter-3);d_C(elementCounter);d_C(elementCounter+1);];
end
for elementCounter = 13:19
    caseC(elementCounter).Elemental_Forces_C = ((A*E)/L)*F_90deg*[d_C(elementCounter-12); d_C(elementCounter-11);d_C(elementCounter-10);d_C(elementCounter-9);];
end
for elementCounter = 20:25
    caseC(elementCounter).Elemental_Forces_C = ((A*E)/L_inc)*F_135deg*[d_C(elementCounter-19); d_C(elementCounter-18);d_C(elementCounter-13);d_C(elementCounter-12);];
end
for elementCounter = 26:31
    caseC(elementCounter).Elemental_Forces_C = ((A*E)/L_inc)*F_45deg*[d_C(elementCounter-23); d_C(elementCounter-22);d_C(elementCounter-21);d_C(elementCounter-20);];
end

%% Problem 2

%Summing the deflections from nodes 13 and 14
u_tip_avg =0;
v_tip_avg =0;
for uCounter = 25:27
    if uCounter == 26 %used to skip v in the deflection array
        continue;
    end
u_tip_avg = u_tip_avg + d_A(uCounter);
u_tip_avg = u_tip_avg + d_B(uCounter);
u_tip_avg = u_tip_avg + d_C(uCounter);

end
u_tip_avg = u_tip_avg/6;
for vCounter = 26:28
    if vCounter == 27 %used to skip u in the deflection array
        continue;
    end
v_tip_avg = v_tip_avg + d_A(vCounter);
v_tip_avg = v_tip_avg + d_B(vCounter);
v_tip_avg = v_tip_avg + d_C(vCounter);
end
v_tip_avg = v_tip_avg/6;


% Calculatinf EA/EI and GA

EA = (20000*1.8)/(u_tip_avg);
%fprintf('EA = %.2f\n', EA)

EI=(10000*(1.8)^3)/(2* v_tip_avg);
%fprintf('EI = %.2f\n', EI)

GA=(20000*1.8)/(v_tip_avg - (20000*1.8^3)/(3*EI));
%fprintf('GA = %.2f\n\n\n', GA)

%% Problem 3 


%Initialize Global Stiffness Matrix;
K_Global_new = zeros(36);
%Determining local stiffness matrices for bottom Horizontal elements
%angle is 0
%Location assignment for Bottom Horizontal Elements node 1 is bottom/left
%and node 2 is top/right
offset =0;
for elementCounter = 1:8
   element_new(elementCounter).K = StiffnessPlaneTruss(E,A,L,0); %User-Defined funtion to calculate K matrix in Plan Truss
   element_new(elementCounter).node1 = [((elementCounter-1)*0.3) 0]; %x = numNode*0.3 y = 0
   element_new(elementCounter).node2 = [(((elementCounter-1)*0.3)+0.3) 0]; %x = (numNode*0.3) +0.3 y =0
   
   %Inserting K for element n into K_Global
   %Top Left
   for rowCounter = 1:2
       for columnCounter =1:2

      K_Global_new((rowCounter+offset), (columnCounter+offset)) =  K_Global_new((rowCounter+offset), (columnCounter+offset))+element_new(elementCounter).K(rowCounter,columnCounter);
       end
   end
   %Top Right
   for rowCounter = 1:2
       for columnCounter =3:4

      K_Global_new((rowCounter+offset), (columnCounter+2+offset)) = K_Global_new((rowCounter+offset), (columnCounter+2+offset))+ element_new(elementCounter).K(rowCounter,columnCounter);
       end
   end
%    %Bottom Left
   for rowCounter = 3:4
       for columnCounter =1:2

      K_Global_new((rowCounter+2+offset), (columnCounter+offset)) = K_Global_new((rowCounter+2+offset), (columnCounter+offset))+ element_new(elementCounter).K(rowCounter,columnCounter);
       end
   end
%    %Bottom Right
   for rowCounter = 3:4
       for columnCounter =3:4

      K_Global_new((rowCounter+2+offset), (columnCounter+2+offset)) = K_Global_new((rowCounter+2+offset), (columnCounter+2+offset))+ element_new(elementCounter).K(rowCounter,columnCounter);
       end
   end
    offset = offset+4;
end


%Determining local stiffness matrices for Top Horizontal elements
%angle is 0
%Location assignment for Bottom Horizontal Elements node 1 is bottom/left
%and node 2 is top/right
offset = 0;
for elementCounter = 9:16
   element_new(elementCounter).K = StiffnessPlaneTruss(E,A,L,0); %User-Defined funtion to calculate K matrix in Plan Truss
   element_new(elementCounter).node1 = [((elementCounter-1)*0.3) 0.3]; %x = numNode*0.3 y = 0
   element_new(elementCounter).node2 = [(((elementCounter-1)*0.3)+0.3) 0.3]; %x = (numNode*0.3) +0.3 y =0
       %Inserting K for element n into K_Global
       %Top Left
       for rowCounter = 1:2
           for columnCounter =1:2
               
               K_Global_new((rowCounter+offset+2), (columnCounter+offset+2)) =  K_Global_new((rowCounter+offset+2), (columnCounter+offset+2))+element_new(elementCounter).K(rowCounter,columnCounter);
           end
       end
       %Top Right
       for rowCounter = 1:2
           for columnCounter =3:4
               
               K_Global_new((rowCounter+offset+2), (columnCounter+2+offset+2)) = K_Global_new((rowCounter+offset+2), (columnCounter+2+offset+2))+ element_new(elementCounter).K(rowCounter,columnCounter);
           end
       end
       %    %Bottom Left
       for rowCounter = 3:4
           for columnCounter =1:2
               
               K_Global_new((rowCounter+2+offset+2), (columnCounter+offset+2)) = K_Global_new((rowCounter+2+offset+2), (columnCounter+offset+2))+ element_new(elementCounter).K(rowCounter,columnCounter);
           end
       end
       %    %Bottom Right
       for rowCounter = 3:4
           for columnCounter =3:4
               
               K_Global_new((rowCounter+2+offset+2), (columnCounter+2+offset+2)) = K_Global_new((rowCounter+2+offset+2), (columnCounter+2+offset+2))+ element_new(elementCounter).K(rowCounter,columnCounter);
           end
       end
    offset = offset+4;
    
end

%Determining local stiffness matrices for Vertical elements
%angle is 90
%Location assignment for Bottom Horizontal Elements node 1 is bottom/left
%and node 2 is top/right
offset =0;
for elementCounter = 17:25
   element_new(elementCounter).K = StiffnessPlaneTruss(E,A,L,90); %User-Defined funtion to calculate K matrix in Plan Truss
   element_new(elementCounter).node1 = [((elementCounter-1)*0.3) 0]; %x = numNode*0.3 y = 0
   element_new(elementCounter).node2 = [(((elementCounter-1)*0.3)+0.3) 0.3]; %x = (numNode*0.3) +0.3 y =0
   
   %Inserting K for element n into K_Global
   %Top Left
   for rowCounter = 1:2
       for columnCounter =1:2

      K_Global_new((rowCounter+offset), (columnCounter+offset)) =  K_Global_new((rowCounter+offset), (columnCounter+offset))+element_new(elementCounter).K(rowCounter,columnCounter);
       end
   end
   %Top Right
   for rowCounter = 1:2
       for columnCounter =3:4

      K_Global_new((rowCounter+offset), (columnCounter+offset)) = K_Global_new((rowCounter+offset), (columnCounter+offset))+ element_new(elementCounter).K(rowCounter,columnCounter);
       end
   end
%    %Bottom Left
   for rowCounter = 3:4
       for columnCounter =1:2

      K_Global_new((rowCounter+offset), (columnCounter+offset)) = K_Global_new((rowCounter+offset), (columnCounter+offset))+ element_new(elementCounter).K(rowCounter,columnCounter);
       end
   end
%    %Bottom Right
   for rowCounter = 3:4
       for columnCounter =3:4

      K_Global_new((rowCounter+offset), (columnCounter+offset)) = K_Global_new((rowCounter+offset), (columnCounter+offset))+ element_new(elementCounter).K(rowCounter,columnCounter);
       end
   end
    offset = offset+4;
end

%Determining local stiffness matrices for Upward(left to right) elements
%angle is 45
%Location assignment for Bottom Horizontal Elements node 1 is bottom/left
%and node 2 is top/right
offset=0;
for elementCounter = 26:33
   element_new(elementCounter).K = StiffnessPlaneTruss(E,A,L_inc,45); %User-Defined funtion to calculate K matrix in Plan Truss
   element_new(elementCounter).node1 = [((elementCounter-1)*0.3) 0.3]; %x = numNode*0.3 y = 0
   element_new(elementCounter).node2 = [(((elementCounter-1)*0.3)+0.3) 0]; %x = (numNode*0.3) +0.3 y =0
   
   %Inserting K for element n into K_Global
   %Top Left
   for rowCounter = 1:2
       for columnCounter =1:2

      K_Global_new((rowCounter+offset), (columnCounter+offset)) =  K_Global_new((rowCounter+offset), (columnCounter+offset))+element_new(elementCounter).K(rowCounter,columnCounter);
       end
   end
   %Top Right
   for rowCounter = 1:2
       for columnCounter =3:4

      K_Global_new((rowCounter+offset), (columnCounter+offset+4)) = K_Global_new((rowCounter+offset), (columnCounter+offset+4))+ element_new(elementCounter).K(rowCounter,columnCounter);
       end
   end
%    %Bottom Left
   for rowCounter = 3:4
       for columnCounter =1:2

      K_Global_new((rowCounter+offset+4), (columnCounter+offset)) = K_Global_new((rowCounter+offset+4), (columnCounter+offset))+ element_new(elementCounter).K(rowCounter,columnCounter);
       end
   end
%    %Bottom Right
   for rowCounter = 3:4
       for columnCounter =3:4

      K_Global_new((rowCounter+offset+4), (columnCounter+offset+4)) = K_Global_new((rowCounter+offset+4), (columnCounter+offset+4))+ element_new(elementCounter).K(rowCounter,columnCounter);
       end
   end
    offset = offset+4;
end


%Determining local stiffness matrices for Downward(left to right) elements
%angle is 135
%Location assignment for Bottom Horizontal Elements node 1 is bottom/left
%and node 2 is top/right
offset = 0;
for elementCounter = 34:41
   element_new(elementCounter).K = StiffnessPlaneTruss(E,A,L_inc,135); %User-Defined funtion to calculate K matrix in Plan Truss
   element_new(elementCounter).node1 = [((elementCounter-1)*0.3) 0.3]; %x = numNode*0.3 y = 0
   element_new(elementCounter).node2 = [(((elementCounter-1)*0.3)+0.3) 0]; %x = (numNode*0.3) +0.3 y =0
   
   %Inserting K for element n into K_Global
   %Top Left
   for rowCounter = 1:2
       for columnCounter =1:2

      K_Global_new((rowCounter+offset+2), (columnCounter+offset+2)) =  K_Global_new((rowCounter+offset+2), (columnCounter+offset+2))+element_new(elementCounter).K(rowCounter,columnCounter);
       end
   end
   %Top Right
   for rowCounter = 1:2
       for columnCounter =3:4

      K_Global_new((rowCounter+offset+2), (columnCounter+offset+2)) = K_Global_new((rowCounter+offset+2), (columnCounter+offset+2))+ element_new(elementCounter).K(rowCounter,columnCounter);
       end
   end
%    %Bottom Left
   for rowCounter = 3:4
       for columnCounter =1:2

      K_Global_new((rowCounter+offset+2), (columnCounter+offset+2)) = K_Global_new((rowCounter+offset+2), (columnCounter+offset+2))+ element_new(elementCounter).K(rowCounter,columnCounter);
       end
   end
%    %Bottom Right
   for rowCounter = 3:4
       for columnCounter =3:4

      K_Global_new((rowCounter+offset+2), (columnCounter+offset+2)) = K_Global_new((rowCounter+offset+2), (columnCounter+offset+2))+ element_new(elementCounter).K(rowCounter,columnCounter);
       end
   end
    offset = offset+4;
end
%% Setting Boundary Conditions

%u1 = v1 = v2 =0

K_Global_new(:,1) = [];
K_Global_new(1,:) = [];
K_Global_new(:,2) = [];
K_Global_new(:,2) = [];
K_Global_new(2,:) = [];
K_Global_new(2,:) = [];


%% Case A V2
F_A_new = [0; 0;0; 0;0; 0;0; 0;0; 0;0; 0;0; 0;0; 0;0; 0;0; 0;0; 0;0; 0;0; 0;0; 0;0; 10000;0; 10000;0;];
%Deflection Matrix
d_A_new = K_Global_new^(-1)*F_A_new;
%Deflection MAtrix adjusted to include 0 value boundaries
d_A_new = [0;d_A_new(1);0;0;d_A_new(2:end)];

for elementCounter = 1:8
    caseA_new(elementCounter).Elemental_Forces_A = ((A*E)/L)*F_0deg*[d_A_new(elementCounter); d_A_new(elementCounter+1);d_A_new(elementCounter+4);d_A_new(elementCounter+5);];
end
for elementCounter = 9:16
    caseA_new(elementCounter).Elemental_Forces_A = ((A*E)/L)*F_0deg*[d_A_new(elementCounter-4); d_A_new(elementCounter-3);d_A_new(elementCounter);d_A_new(elementCounter+1);];
end
for elementCounter = 17:25
    caseA_new(elementCounter).Elemental_Forces_A = ((A*E)/L)*F_90deg*[d_A_new(elementCounter-12); d_A_new(elementCounter-11);d_A_new(elementCounter-10);d_A_new(elementCounter-9);];
end
for elementCounter = 26:33
    caseA_new(elementCounter).Elemental_Forces_A = ((A*E)/L_inc)*F_135deg*[d_A_new(elementCounter-19); d_A_new(elementCounter-18);d_A_new(elementCounter-13);d_A_new(elementCounter-12);];
end
for elementCounter = 34:41
    caseA_new(elementCounter).Elemental_Forces_A = ((A*E)/L_inc)*F_45deg*[d_A_new(elementCounter-23); d_A_new(elementCounter-22);d_A_new(elementCounter-21);d_A_new(elementCounter-20);];
end
%% Case B V2
F_B_new = [0; 0;0; 0;0; 0;0; 0;0; 0;0; 0;0; 0;0; 0;0; 0;0; 0;0; 0;0; 0;0; 0;0; 0;0; 0;10000; 0;10000;];
%Deflection Matrix
d_B_new = K_Global_new^(-1)*F_B_new;
%Deflection MAtrix adjusted to include 0 value boundaries
d_B_new = [0;d_B_new(1);0;0;d_B_new(2:end)];

for elementCounter = 1:8
    caseB_new(elementCounter).Elemental_Forces_B = ((A*E)/L)*F_0deg*[d_B_new(elementCounter); d_B_new(elementCounter+1);d_B_new(elementCounter+4);d_B_new(elementCounter+5);];
end
for elementCounter = 9:16
    caseB_new(elementCounter).Elemental_Forces_B = ((A*E)/L)*F_0deg*[d_B_new(elementCounter-4); d_B_new(elementCounter-3);d_B_new(elementCounter);d_B_new(elementCounter+1);];
end
for elementCounter = 17:25
    caseB_new(elementCounter).Elemental_Forces_B = ((A*E)/L)*F_90deg*[d_B_new(elementCounter-12); d_B_new(elementCounter-11);d_B_new(elementCounter-10);d_B_new(elementCounter-9);];
end
for elementCounter = 26:33
    caseB_new(elementCounter).Elemental_Forces_B = ((A*E)/L_inc)*F_135deg*[d_B_new(elementCounter-19); d_B_new(elementCounter-18);d_B_new(elementCounter-13);d_B_new(elementCounter-12);];
end
for elementCounter = 34:41
    caseB_new(elementCounter).Elemental_Forces_B = ((A*E)/L_inc)*F_45deg*[d_B_new(elementCounter-23); d_B_new(elementCounter-22);d_B_new(elementCounter-21);d_B_new(elementCounter-20);];
end
%% Case C V2
F_C_new = [0; 0;0; 0;0; 0;0; 0;0; 0;0; 0;0; 0;0; 0;0; 0;0; 0;0; 0;0; 0;0; 0;0; 0;0; 10000;0; -10000;0;];
%Deflection Matrix
d_C_new = K_Global_new^(-1)*F_C_new;
%Deflection MAtrix adjusted to include 0 value boundaries
d_C_new = [0;d_C_new(1);0;0;d_C_new(2:end)];

for elementCounter = 1:8
    caseC_new(elementCounter).Elemental_Forces_C = ((A*E)/L)*F_0deg*[d_C_new(elementCounter); d_C_new(elementCounter+1);d_C_new(elementCounter+4);d_C_new(elementCounter+5);];
end
for elementCounter = 9:16
    caseC_new(elementCounter).Elemental_Forces_C = ((A*E)/L)*F_0deg*[d_C_new(elementCounter-4); d_C_new(elementCounter-3);d_C_new(elementCounter);d_C_new(elementCounter+1);];
end
for elementCounter = 17:25
    caseC_new(elementCounter).Elemental_Forces_C = ((A*E)/L)*F_90deg*[d_C_new(elementCounter-12); d_C_new(elementCounter-11);d_C_new(elementCounter-10);d_C_new(elementCounter-9);];
end
for elementCounter = 26:33
    caseC_new(elementCounter).Elemental_Forces_C = ((A*E)/L_inc)*F_135deg*[d_C_new(elementCounter-19); d_C_new(elementCounter-18);d_C_new(elementCounter-13);d_C_new(elementCounter-12);];
end
for elementCounter = 34:41
    caseC_new(elementCounter).Elemental_Forces_C = ((A*E)/L_inc)*F_45deg*[d_C_new(elementCounter-23); d_C_new(elementCounter-22);d_C_new(elementCounter-21);d_C_new(elementCounter-20);];
end

%Summing the deflections from nodes 17 and 18
u_tip_avg_new =0;
v_tip_avg_new =0;
for uCounter = 25:27
    if uCounter == 26 %used to skip v in the deflection array
        continue;
    end
u_tip_avg_new = u_tip_avg_new + d_A_new(uCounter);
u_tip_avg_new = u_tip_avg_new + d_B_new(uCounter);
u_tip_avg_new = u_tip_avg_new + d_C_new(uCounter);

end
u_tip_avg_new = u_tip_avg_new/6;
for vCounter = 26:28
    if vCounter == 27 %used to skip u in the deflection array
        continue;
    end
v_tip_avg_new = v_tip_avg_new + d_A_new(vCounter);
v_tip_avg_new = v_tip_avg_new + d_B_new(vCounter);
v_tip_avg_new = v_tip_avg_new + d_C_new(vCounter);
end
v_tip_avg_new = v_tip_avg_new/6;


% Calculatinf EA/EI and GA

EA_new = (20000*1.8)/(u_tip_avg_new);
%fprintf('EA = %.2f\n', EA)

EI_new=(10000*(1.8)^3)/(2* v_tip_avg_new);
%fprintf('EI = %.2f\n', EI)

GA_new=(20000*1.8)/(v_tip_avg_new - (20000*1.8^3)/(3*EI_new));
%fprintf('GA = %.2f\n\n\n', GA)
