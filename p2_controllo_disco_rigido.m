%% ANALISI SISTEMA SENZA CONTROLLORE
z = zero(sys_tf);
p = pole(sys_tf);
disp("Zeri")
disp(z)
disp("Poli")
disp(p)

disp("Margini di guadagno e di fase del sistema")
[Gm,Pm,Wcg,Wcp] = margin(sys_tf);
disp(Gm)
disp(Pm)

[m,ph,w] = bode(sys_tf);
m = 20*log10(m);
mag = m(:);

figure
bode(sys_tf);
grid on
title("Diagramma di Bode della FdT ad anello aperto")

%% CALCOLO E PLOTTO I LIMITI DA IMPORRE AL PROBLEMA 

%Inserisco i dati del problema
wc = 10; wd = 0.5:0.5:5; wn = 50:50:500;
d = 1; n = 1;
Ed = 20*log10(10); En = 0;

PHm = 65;
disp("Margine di fase richiesto")
disp(PHm)
disp("Margine di fase")
disp(Pm)

%Grafico le soglie
figure
semilogx(w,mag)
hold on
semilogx(wc,0,'g*'),semilogx(wd,Ed, 'ko'),semilogx(wn,En, 'r>')
ylabel('dB'),xlabel('pulsazione'),grid;
title('Diagramma di Bode - Modulo')

%% CALCOLO LE CARATTERISTICHE DEL REGOLATORE

