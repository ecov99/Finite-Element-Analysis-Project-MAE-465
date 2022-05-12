%Plane Truss Calculation Function
function k = StiffnessPlaneTruss(E, A, L, angle)

%%Calculation for matrix Generation for Element
%Calculate Length of element
%Calculate Cos and Sin for K matrix

C = cosd(angle);
S = sind(angle);

%K Matrix
k = (A*E/L)*[C^2 C*S -C^2 -C*S;
             C*S S^2 -C*S -S^2;
             -C^2 -C*S C^2 C*S;
             -C*S -S^2 C*S S^2];


end
