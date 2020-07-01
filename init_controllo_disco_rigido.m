clc
clear
close all

%% INIZIALIZZO LE VARIABILI

b = 0.002;
J = 0.001;
%% DICHIARO LE MATRICI DEL SISTEMA

A = [-b/J, 0;1, 0];
B = [1/J; 0];
C = [0, 1];
D = 0;
%% APRO IL MODELLO SIMULINK

op_mdl = 'controllo_disco_rigido_openloop';
open_system(op_mdl)
%% DEFINISCO IL MODELLO IN SPAZIO DI STATO ED IN FUNZIONE DI TRASFERIMENTO
sys_ss = ss(A,B,C,D);
sys_tf = tf(sys_ss);