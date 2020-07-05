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
%% APRO I MODELLI SIMULINK

op_mdl = 'controllo_disco_rigido_openloop';
open_system(op_mdl)

cl_mdl = 'controllo_disco_rigido_closedloop';
open_system(cl_mdl)

%% APRO GLI ALTRI SCRIPT

open p1_controllo_disco_rigido
open p2_controllo_disco_rigido

%% DEFINISCO IL MODELLO IN SPAZIO DI STATO ED IN FUNZIONE DI TRASFERIMENTO
sys_ss = ss(A,B,C,D);
sys_tf = tf(sys_ss);