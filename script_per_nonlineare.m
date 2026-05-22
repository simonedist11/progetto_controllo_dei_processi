%% 1. DEFINIZIONE DEL MODELLO
% Espandiamo il sistema per includere l'azione integrale sui disturbi (UD)
B_ext = [B, B]; 
D_ext = [D, D]; 
plant = ss(A, B_ext, C, D_ext);

% Definiamo i segnali: 2 Ingressi Manipolati (MV) e 2 Disturbi non misurati (UD)
plant = setmpcsignals(plant, 'MV', [1 2], 'UD', [3 4]);

% Tempo di campionamento
Ts = 0.05; 
plant_d = c2d(plant, Ts);

%% 2. CREAZIONE DEL CONTROLLORE MPC
p = 30; % Prediction Horizon
m = 3;  % Control Horizon
mpc_obj = mpc(plant_d, Ts, p, m);

%% 3. DEFINIZIONE DEL PUNTO NOMINALE
% U_nom deve essere 4x1: [Ingresso_pitch, Ingresso_yaw, Disturbo_pitch, Disturbo_yaw]
U_nom = [0.8308, 0, 0, 0]; 
X_nom = [1.55, pi/4, 0, 0];
Y_nom = [1.55, pi/4, 0, 0];

mpc_obj.Model.Nominal.U = U_nom;
mpc_obj.Model.Nominal.X = X_nom;
mpc_obj.Model.Nominal.Y = Y_nom;

%% 4. VINCOLI (Saturazione Motori)
mpc_obj.MV(1).Min = -12; 
mpc_obj.MV(1).Max = 12;
mpc_obj.MV(2).Min = -12; 
mpc_obj.MV(2).Max = 12;

%% 5. TUNING (Pesi)
mpc_obj.Weights.OutputVariables = [5, 5, 0, 0]; 
mpc_obj.Weights.ManipulatedVariablesRate = [2, 2]; 
mpc_obj.Weights.ManipulatedVariables = [0, 0];

%% 6. INIZIALIZZAZIONE DELLO STATO
mstate = mpcstate(mpc_obj);

mstate.Plant = [1.55; pi/4; 0; 0];
mstate.Disturbance = zeros(size(mstate.Disturbance));
mstate.LastMove = [0.8303; 0];