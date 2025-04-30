
lam1 = 1;
lam2 = 1;
lam3 = 1;
lam4 = 1;

g = 9.81;
Ix = 0.0093;
Iy = 0.0092;
xC = -7;
yC = -3.5;
R = 1.;

H0 = [];
H1 = [];
H2 = [];
H3 = [];
H4 = [];

for i = 1:size(U1,1)
    
    x = X0(:,i);

    p1 = x(1);
    p2 = x(2);
    v1 = x(3);
    v2 = x(4);
    a1 = x(5);
    a2 = x(6);
    w1 = x(7);
    w2 = x(8);

    u1 = U1(i);
    u2 = U2(i);


    h0 = (p1 - xC)^2 + (p2 - yC)^2 - R^2;
 
    h1 = lam1*((p1 - xC)^2 + (p2 - yC)^2 - R^2) + v1*(2*p1 - 2*xC) + v2*(2*p2 - 2*yC);
 
 
    h2 = lam2*(lam1*((p1 - xC)^2 + (p2 - yC)^2 - R^2) + v1*(2*p1 - 2*xC) + v2*(2*p2 - 2*yC)) + v1*(2*v1 + lam1*(2*p1 - 2*xC)) + v2*(2*v2 + lam1*(2*p2 - 2*yC)) + a2*g*(2*p1 - 2*xC) - a1*g*(2*p2 - 2*yC);
 
    h3 = lam3*(lam2*(lam1*((p1 - xC)^2 + (p2 - yC)^2 - R^2) + v1*(2*p1 - 2*xC) + v2*(2*p2 - 2*yC)) + v1*(2*v1 + lam1*(2*p1 - 2*xC)) + v2*(2*v2 + lam1*(2*p2 - 2*yC)) + a2*g*(2*p1 - 2*xC) - a1*g*(2*p2 - 2*yC)) + v1*(2*a2*g + 2*lam1*v1 + lam2*(2*v1 + lam1*(2*p1 - 2*xC))) + v2*(2*lam1*v2 - 2*a1*g + lam2*(2*v2 + lam1*(2*p2 - 2*yC))) + a2*g*(4*v1 + lam1*(2*p1 - 2*xC) + lam2*(2*p1 - 2*xC)) - a1*g*(4*v2 + lam1*(2*p2 - 2*yC) + lam2*(2*p2 - 2*yC)) + g*w2*(2*p1 - 2*xC) - g*w1*(2*p2 - 2*yC);

    h4 = lam4*(lam3*(lam2*(lam1*((p1 - xC)^2 + (p2 - yC)^2 - R^2) + v1*(2*p1 - 2*xC) + v2*(2*p2 - 2*yC)) + v1*(2*v1 + lam1*(2*p1 - 2*xC)) + v2*(2*v2 + lam1*(2*p2 - 2*yC)) + a2*g*(2*p1 - 2*xC) - a1*g*(2*p2 - 2*yC)) + v1*(2*a2*g + 2*lam1*v1 + lam2*(2*v1 + lam1*(2*p1 - 2*xC))) + v2*(2*lam1*v2 - 2*a1*g + lam2*(2*v2 + lam1*(2*p2 - 2*yC))) + a2*g*(4*v1 + lam1*(2*p1 - 2*xC) + lam2*(2*p1 - 2*xC)) - a1*g*(4*v2 + lam1*(2*p2 - 2*yC) + lam2*(2*p2 - 2*yC)) + g*w2*(2*p1 - 2*xC) - g*w1*(2*p2 - 2*yC)) + w2*(2*g*v1 + g*(4*v1 + lam1*(2*p1 - 2*xC) + lam2*(2*p1 - 2*xC)) + g*lam3*(2*p1 - 2*xC)) - w1*(2*g*v2 + g*(4*v2 + lam1*(2*p2 - 2*yC) + lam2*(2*p2 - 2*yC)) + g*lam3*(2*p2 - 2*yC)) + v1*(2*g*w2 + lam3*(2*a2*g + 2*lam1*v1 + lam2*(2*v1 + lam1*(2*p1 - 2*xC))) + a2*g*(2*lam1 + 2*lam2) + 2*lam1*lam2*v1) - v2*(2*g*w1 - lam3*(2*lam1*v2 - 2*a1*g + lam2*(2*v2 + lam1*(2*p2 - 2*yC))) + a1*g*(2*lam1 + 2*lam2) - 2*lam1*lam2*v2) + a2*g*(6*a2*g + 2*lam1*v1 + lam3*(4*v1 + lam1*(2*p1 - 2*xC) + lam2*(2*p1 - 2*xC)) + v1*(2*lam1 + 2*lam2) + lam2*(2*v1 + lam1*(2*p1 - 2*xC))) - a1*g*(2*lam1*v2 - 6*a1*g + lam3*(4*v2 + lam1*(2*p2 - 2*yC) + lam2*(2*p2 - 2*yC)) + v2*(2*lam1 + 2*lam2) + lam2*(2*v2 + lam1*(2*p2 - 2*yC))) + (g*u2*(2*p1 - 2*xC))/Iy - (g*u1*(2*p2 - 2*yC))/Ix;
    
    H0 = [H0;h0];
    H1 = [H1;h1];
    H2 = [H2;h2];
    H3 = [H3;h3];
    H4 = [H4;h4];

    
end


plot(H0)
hold on
plot(H1)
hold on
plot(H2)
hold on
plot(H3)
hold on
plot(H4)

legend("h0", "h1", "h2", "h3", "h4")