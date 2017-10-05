% Sistema Massa-Mola

clear all;
clc;

syms s

% Planta
m1=1; m2=2;
k1=1; k2=1;

A = [     0        1       0        0; 
     -(k1 + k2)/m1 0     k2/k1      0;
          0        0       0        1;
        k2/m2      0  -(k1 + k2)/m1 0];
   
B = [ 0    0;
     1/m1  0;
      0    0;
      0   1/m2];

C = [1 0 0 0;
     0 0 1 0];

D = zeros(2,2);

Dsim = zeros(4,2); % para obter o estado x na simulacoes

x0 = [0 0 0 0];    % condicao inicial do sistema
% x0 = [1 2 0.8 -0.3];

x0obs = [0 0 0 0]; % condicao inicial do observador

% Modelo Interno
beta = expand(s*(s+3/10)*(s^2+1))

coef = [0 -3/10 -1 -3/10];   % coeficientes de beta

M = [zeros(3,1) eye(3); coef];

N = [0; 0 ; 0; 1];

Am = blkdiag(M,M);
Bm = blkdiag(N,N);

% Sistema Aumentado
Aa = [ A     zeros(4,8);
      -Bm*C    Am        ];

Ba = [B ; zeros(8,2) ];

% Determinacao da controlabilidade do sistema aumentado
Con = ctrb(Aa,Ba);  % Matriz de Controlabilidade
vsCon = svd(Con)    % numero da valores singulares nao-nulos = posto(Con)

% Polo repetido desejado
pd=-1; 
% Matriz de ganho do estado aumentado xa para posicionar os polos de Aa-BaKa em pd
Ka = place(Aa,Ba,[pd pd-0.025 pd-0.05 pd-0.075 pd-0.1 pd-0.125 pd-0.15 pd-0.175 pd-0.2 pd-0.225 pd-0.250 pd-0.275]);

% Verificacao
polosMF = eig(Aa-Ba*Ka)

% Matriz de ganho para o estado da planta x 
K = Ka(:,1:4);

% Matriz de ganho para o estado do modelo interno xm
Km = Ka(:,5:12);

% Determinacao da observabilidade da planta
Obs = obsv(A,C);   % Matriz de Observabilidade
vsObs = svd(Obs)   % numero da valores singulares nao-nulos = posto(Obs)

% Polo repetido desejado para o observador
pobs = 2*pd;
% Matriz de ganho para posicionar os polos de A-LC em pobs
H = place(A',C',[pobs pobs-0.025 pobs-0.05 pobs-0.075]);
Lobs = H';

% Verificacao
polosObs = eig(A-(Lobs*C))

