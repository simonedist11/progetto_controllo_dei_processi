%% 1. DEFINIZIONE DEL MODELLO
B_ext = [B, B]; 
D_ext = [D, D]; 
plant = ss(A, B_ext, C, D_ext);

% 2 Ingressi Manipolati (MV) e 2 Disturbi non misurati (UD)
plant = setmpcsignals(plant, 'MV', [1 2], 'UD', [3 4]);

Ts = 0.05; 
plant_d = c2d(plant, Ts);

%% 2. CREAZIONE DEL CONTROLLORE MPC
p = 30;  % Prediction Horizon
m = 3;  % Control Horizon
mpc_obj = mpc(plant_d, Ts, p, m);

%% 3. DEFINIZIONE DEL PUNTO NOMINALE (punto di linearizzazione)
% Poiché le matrici A e B sono state calcolate nel punto di ARRIVO,
% il punto nominale dell'MPC deve essere quello di arrivo.
U_nom = [0.8308, 0, 0, 0]; 
X_nom = [1.55, pi/4, 0, 0];
Y_nom = [1.55, pi/4, 0, 0];

mpc_obj.Model.Nominal.U = U_nom;
mpc_obj.Model.Nominal.X = X_nom;
mpc_obj.Model.Nominal.Y = Y_nom;

%% 4. VINCOLI (Saturazione Motori)
mpc_obj.MV(1).Min = -12; mpc_obj.MV(1).Max = 12;
mpc_obj.MV(2).Min = -12; mpc_obj.MV(2).Max = 12;

%% 5. TUNING (Pesi per il tracciamento)
mpc_obj.Weights.OutputVariables = [10, 10, 0, 0]; 
mpc_obj.Weights.ManipulatedVariablesRate = [0.1, 0.1]; % Più veloce a reagire
mpc_obj.Weights.ManipulatedVariables = [0, 0];

%% 6. INIZIALIZZAZIONE DELLO STATO SUL PUNTO DI PARTENZA
mstate = mpcstate(mpc_obj);

mstate.Plant = [0.117; 0; 0; 0]; 
mstate.Disturbance = zeros(size(mstate.Disturbance));

% All'istante zero i motori sono spenti
mstate.LastMove = [0; 0]; 
