%% RISPOSTA A SCALINO DEL SISTEMA
t = 0:0.01:10;
[ys, ts] = step(sys_tf, t);

MdlStep = sim(op_mdl);

figure
plot(MdlStep.tout, MdlStep.stepResponse)
grid on
title("Risposta allo scalino - Modello")

figure
plot(ts, ys)
grid on
title("Risposta allo scalino - FdT")

%% CALCOLO VALORI INIZIALE, FINALE E TEMPO DI ASSESTAMENTO
lamba = eig(A);
Ta = 5/abs(real(lamba(2))); %Il primo è zero
disp("Tempo di assestamento")
disp(Ta)

syms G num den
num = poly2sym(cell2mat(sys_tf.num));
den = poly2sym(cell2mat(sys_tf.den));
G = num/den;

VF = limit(G,0)
VI = limit(G,inf)