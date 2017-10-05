clear all
clc
xe = [5 0 pi 0];

% Parametros do pendulo invertido
M = 1; 
m = 0.5;
L = 1; 
g = 9.81;

% Matrizes do modelo de estado
A = [0 1  0              0;
     0 0 -m*g/M          0;
     0 0  0              1;
     0 0 (m + M)*g/(M*L) 0];

B = [0; 1/M; 0; -1/(M*L)];

C = [1 0 0 0];

D = zeros(1);

Dsim = zeros(4,1); % para obter o estado x na simulacoes

x0 = [0 0.01 0.3 0]; % condicao inicial do pendulo

% Analise do sistema
autovalores = eig(A)             % polos da matriz A

sys=ss(A,B,C,D);   % define modelo de estado
 
G=zpk(tf(sys))     % funcao de transferencia

MC=ctrb(sys)       % matriz de controlabilidade
svd(MC)            % valores singulares de MC, em que posto(MC) = numero de valores singulares nao-nulos

MO=obsv(sys)       % matriz de observabilidade
svd(MO)            % valores singulares de MO, em que posto(MO) = numero de valores singulares nao-nulos

% Modelo Interno
coef = [0];   % coeficientes de beta (s + 0) k = 1

M = [coef];

N = [1];

Am = blkdiag(M); %p=1 (uma referencia e uma pertubação) então uma copia só
Bm = blkdiag(N);

% Sistema Aumentado
Aa = [ A     zeros(4,1);
      -Bm*C    Am   ];

Ba = [ B; 0];

% Determinacao da controlabilidade do sistema aumentado
Con = ctrb(Aa,Ba);  % Matriz de Controlabilidade
vsCon = svd(Con)    % numero da valores singulares nao-nulos = posto(Con)



% Polo repetido desejado
pd=-3.6; 
% Matriz de ganho do estado aumentado xa para posicionar os polos de Aa-BaKa em pd
Ka = place(Aa,Ba,[pd pd-0.025 pd-0.05 pd-0.075 pd-0.1]);

K = Ka(1:4);

Km = Ka(5);

% Matriz de transferencia do sistema aumentado em malha-fechada
E = B;
F = zeros(1,1);
Ae = Aa-Ba*Ka;
Be = [   E  zeros(4,1); %%% -Bm*F = 0
      0     Bm    ];
Ca = [C 0];
Da = [F zeros(1,1)]; %Da é uma matriz 2x2 porque existe somente 1 ref e 1 pertubação

sysa=ss(Ae,Be,Ca,Da);

Ga=zpk(tf(sysa));
GMF=Ga(2) %FT de malha fechada com relação a referência

%% Item 4 - Seguimento de referencia e pertubação com estimador de estados
%Como o valor do K foi separado o calculo do estimador é independente do
%calculo do modelo interno. (princípio da separação)

x0obs = [0 0 0 0]; % condicao inicial do observador

pobs =4*pd;
% Matriz de ganho para posicionar os polos de A-LC em pobs
H = place(A',C',[pobs pobs-0.025 pobs-0.05 pobs-0.075]); %posiciona os polos de A-LC
Lobs = H'; 

polosObs = eig(A-Lobs*C) 

%% Item 5

% Modelo Interno para rejeição de senoide
coef = [0.01 0];   % coeficientes de beta (s^2 + w^2) = s^2 + 0.01 = s^2 + 0*s + 0.01 (k=2)

M = [zeros(1,1) eye(1); coef]; %ordem da matriz k x k

N = [0; 1]; %ordem da matriz k x 1

Am2 = blkdiag(M); %p=1 (uma referencia (0) e uma pertubação(seno)) então uma copia só
Bm2 = blkdiag(N);

% Sistema Aumentado
Aa2 = [A zeros(4, 2);
    -Bm2*C Am2];
Ba2 = [B; 0; 0];

% Polo repetido desejado
pd=-3.6; 
% Matriz de ganho do estado aumentado xa para posicionar os polos de Aa-BaKa em pd
Ka2 = place(Aa2,Ba2,[pd pd-0.025 pd-0.05 pd-0.075 pd-0.1 pd-0.035]);

K2 = Ka2(1:4); 

Km2 = Ka2(5:6);






