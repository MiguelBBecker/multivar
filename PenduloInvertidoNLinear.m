function dx = PenduloInvertidoNLinear(v)
% Trabalho 1
% Questão 3: Pêndulo Invertido Não-Linear

% Equações:
% (M+m)*y" = u - m*L*theta"*cos(theta) + m*L*((theta')^2)*sen(theta) 
%  m*L*theta" = m*g*sen(theta) - m*y"*cos(theta)                 

% Constantes do Sistema:
m = 0.5; 
M = 1; 
L = 1; 
g = 9.81;


% 3.1 -> REPRESENTAÇÃO EM ESPAÇO DE ESTADOS
% Variáveis Simbólicas p a Representação:
syms x1 x2 x3 x4 u dx1 dx2 dx3 dx4;

x1 = v(1);
x2 = v(2);
x3 = v(3);
x4 = v(4);
u  = v(5);
 
 
dx1 = x2;
dx2 = u - (m*g*cos(x3)*sin(x3)+m*L*x4^2*sin(x3))/(M+m - m*cos(x3)^2);
dx3 = x4;
dx4 = (u*cos(x3)+m*L*x4^2*cos(x3)*sin(x3)-(M+m)*g*sin(x3))/(m*L*cos(x3)^2 - L*(M+m));
 
dx = [dx1 ; dx2 ; dx3 ; dx4];
 
end


