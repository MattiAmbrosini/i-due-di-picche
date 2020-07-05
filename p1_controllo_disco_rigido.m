%% RISPOSTA A SCALINO DEL SISTEMA
t = 0:0.01:10;
[ys, ts] = step(sys_tf, t);

MdlStep = sim(op_mdl);
figure
plot(MdlStep.tout, MdlStep.y)
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

ValoreIniziale = limit(G,inf)
ValoreFinale = limit(G,0)

%% STIMA DELLA BANDA DEL SISTEMA

[m, ph, w] = bode(sys_tf);
m = 20*log10(m);
mag = m(:);
[~,~,~,Wcp] = margin(sys_tf);
disp(Wcp)

bw = [w(1) 0];
for i = 1:1:size(mag,1)
    if mag(i,1) < 0
        bw(2) = w(i,1);
        break
    end
end
disp(bw)

figure, semilogx(w(1:i), mag(1:i)), grid on