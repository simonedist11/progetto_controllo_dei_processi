sys_c = ss(A, B, C, D);

% Tempo di campionamento
Ts = 0.1;

% Discretizzazione Zero Order Hold
sys_d = c2d(sys_c, Ts, 'zoh');

% Matrici discrete
[Ad, Bd, Cd, Dd] = ssdata(sys_d);


% Matrice raggiungibilità 4 passi
n = 150; % Numero di passi desiderato
m = size(Bd, 2); % Numero di ingressi
p = size(Ad, 1); % Dimensione dello stato

Rd = []; 

for i = n-1 : -1 : 0
    prossimo_blocco = (Ad^i) * Bd;
    Rd = [Rd, prossimo_blocco]; 
end


% Matrice di peso W
W=1/4*eye(2*n);

% Calcolo Ingresso ottimo u_star
x_0= [0.117; 0; 0; 0];          %stato iniziale
x_f=[1.55; pi/4; 0; 0];   %stato finale
u_star = inv(W)*Rd'*inv(Rd*inv(W)*Rd')*(x_f - Ad^n*x_0);

I=eye(4);
u_regime = pinv(Bd) * (I - Ad) * x_f;