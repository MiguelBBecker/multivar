clear all;
clc;

% Sistema Linearizado
A = [0 1; -5*sqrt(2) -1]; % 10/1*cos(45º)
B = [0 ; 10]; % m = 0.1 
C = [1 0]; 

% Matrizes de ganho dos controladores estabilizantes (veja os 2 exemplos na Secao 3.4 da Teoria)
K = place(A,B,[-4 -4.05])      % realimentacao de estado: polo duplo de A-BK em s=-4
L = [23 ; 113.9289]   % observador: polo duplo de A-LC em s=-12 

% Sistema Aumentado para controle integral linear
Aa=[ A zeros(2,1);
    -C 0];

Ba=[B ; 0];

% Verifica a controlabilidade do Sistema Aumentado
Con = ctrb(Aa,Ba)
vsCon = svd(Con)

% Matriz de ganho do controlador integral linear
Ka=place(Aa,Ba,[-4 -4.05 -4.1]);     % polo tripo de Aa-BaKa em s=-4     

Kx = Ka(1,1:2)
Km = Ka(1,3)

% Simulacao
x0 = [0 ; 0]                          % condicao inicial do pendulo
x0obs = [0 ; 0]                       % condicao inicial do observador
