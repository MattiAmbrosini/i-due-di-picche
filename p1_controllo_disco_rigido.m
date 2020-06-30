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

%% RISPOSTA A SCALINO DEL SISTEMA
t = 0:0.01:10;
[ys, ts] = step(sys_tf, t);

figure
plot(ts, ys)
grid on
title("Step response")

%% 