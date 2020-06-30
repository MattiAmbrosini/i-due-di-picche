%% ANALISI SISTEMA SENZA CONTROLLORE

figure
bode(sys_tf)
grid on
% hold on
% bode(1, 1, '*')

[Gm,Pm,Wcg,Wcp] = margin(sys_tf)

%%