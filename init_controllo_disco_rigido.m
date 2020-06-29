%% INIZIALIZZO LE VARIABILI

b = 0.002;
J = 0.001;
%% DICHIARO LE MATRICI DEL SISTEMA

A = [-b/J, 0;1, 0];
B = [1/J; 0];
C = [0, 1];
D = 0;
%% APRO IL MODELLO SIMULINK

mdl = 'controllo_disco_rigido';
open_system(mdl)
%% DEFINISCO IL MODELLO IN SPAZIO DI STATO ED IN FUNZIONE DI TRASFERIMENTO
sys_ss = ss(A,B,C,D);
sys_tf = tf(sys_ss);

% %%
% io(1) = linio('controllo_disco_rigido/u',1,'openinput');
% io(2) = linio('controllo_disco_rigido/Integrator1',1,'openoutput');
% 
% % sys = ss('ControlloDisco/controllo_disco_rigido');
% 
% linsys_ss = linearize(mdl,io);
% %%
% bode(linsys_ss)
% linsys_tf = tf(linsys_ss);