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

%% NOME DEI MODELLI SIMULINK
op_mdl = 'controllo_disco_rigido_openloop';
cld_mdl = 'controllo_disco_rigido_disturbi';
cln_mdl = 'controllo_disco_rigido_rumore';
fn_mdl = 'controllo_disco_rigido_final';

%% DEFINISCO IL MODELLO IN SPAZIO DI STATO ED IN FUNZIONE DI TRASFERIMENTO
sys_ss = ss(A,B,C,D);

disp("Funzione di trasferimento del sistema")
sys_tf = tf(sys_ss)

%% CALCOLO EQUILIBRIO E STABILITA'
ubar = 1;
xbar=-inv(A)*B*ubar;
ybar=C*xbar+D*ubar;

disp("Verifico che non esiste una posizione di equilibrio")
disp("Stato")
disp(xbar)
disp("Uscita")
disp(ybar)

%% RISPOSTA A SCALINO DEL SISTEMA
t = 0:0.01:10;
[ys, ts] = step(sys_tf, t);

MdlStep = sim(op_mdl);
figure, plot(MdlStep.tout, MdlStep.y), grid on
title("Risposta allo scalino - Modello")

figure, plot(ts, ys), grid on
title("Risposta allo scalino - FdT")

%% CALCOLO VALORI INIZIALE, FINALE E TEMPO DI ASSESTAMENTO
lambda = eig(A);
disp("Autovalori di A")
disp(lambda)

Ta = 5/abs(real(lambda(2))); %Il primo è zero
disp("Tempo di assestamento")
disp(Ta)

syms G num den
num = poly2sym(cell2mat(sys_tf.num));
den = poly2sym(cell2mat(sys_tf.den));
G = num/den;

disp("Risposta allo scalino")
disp("Valore iniziale")
Vi= limit(G,inf);
disp(Vi)
disp("Valore finale")
Vf = limit(G,0);
disp(Vf)

%% STIMA DELLA BANDA DEL SISTEMA
[m, ~, w] = bode(sys_tf);
m = 20*log10(m);
mag = m(:);

disp("Stima della banda del sistema")
bw = [w(1) 0];
for i = 1:1:size(mag,1)
    if mag(i,1) < 0
        bw(2) = w(i,1);
        break
    end
end
disp(bw)

% figure, semilogx(w(1:i), mag(1:i)), grid on
% title("Diagramma di Bode del modulo della funzione per la banda stimata")

%% ANALISI SISTEMA SENZA CONTROLLORE
z = zero(sys_tf);
p = pole(sys_tf);
disp("Zeri")
disp(z)
disp("Poli")
disp(p)

disp("Margine di guadagno del sistema")
[Gm,Pm,~,~] = margin(sys_tf);
disp(Gm)

figure, bode(sys_tf), grid on
title("Diagramma di Bode della FdT ad anello aperto")

%% CALCOLO E PLOTTO I LIMITI DA IMPORRE AL PROBLEMA 

%Inserisco i dati del problema
wc = 10; wd = 0.5:0.1:5; wn = 50:10:500;
d = 1; n = 1;
Ed = 20*log10(10); En = 0;

% Setpoint e ritardo puro
y0 = 300;
delay = 0.3;

PHm = 65;
disp("Margine di fase minnimo richiesto")
disp(PHm)
disp("Margine di fase del sistema non controllato")
disp(Pm)

%Grafico le soglie
figure
semilogx(w,mag)
hold on
semilogx(wc,0,'g*'),semilogx(wd,Ed, 'ko'),semilogx(wn,En, 'r>')
ylabel('dB'),xlabel('pulsazione'),grid;
title('Diagramma di Bode - Modulo')

%% CALCOLO LE CARATTERISTICHE DEL REGOLATORE
R1 = 100;
R2_num = [1 2];
R2_den = [1 1400];

disp("FdT del regolatore")
R = tf(R2_num*R1, R2_den)

disp("FdT d'anello")
L = series(sys_tf,R)

%% SISTEMA IN ANELLO CHIUSO RISPETTO ALLE SOGLIE
[mL, ~, wL] = bode(L);
[GmL,PmL,~,~] = margin(L);
mL = 20*log10(mL);
magL = mL(:);

figure
semilogx(wL,magL)
hold on
semilogx(wc,0,'g*'),semilogx(wd,Ed, 'ko'),semilogx(wn,En, 'r>')
ylabel('dB'),xlabel('pulsazione'),grid;
title('Diagramma di Bode di L - Modulo')

disp("Margine di fase minimo richiesto")
disp(PHm)
disp("Margine di fase del sistema in anello chiuso")
disp(PmL)

figure, bode(L), grid on
title("Diagramma di Bode della FdT ad anello chiuso")

%% VERIFICO LA STABILITA' DEL NUOVO SISTEMA
figure, nyquist(feedback(L,1)), grid on
zL = real(zero(1+L));
disp("Radici di 1+L(s)")
disp(zL)
cas = 0;
for i = 1:1:size(zL,1)
    if zL(i,1) >= 0
        disp("Sistema non as. stabile")
        cas = 1;
    end
end
if cas == 0
    disp("Sistema in anello chiuso as. stabile")
end

%% VERIFICO IL COMPORTAMENTO DEL SISTEMA IN ANELLO CHIUSO
MdlCL = sim(cld_mdl);
figure, plot(MdlCL.tout, MdlCL.y), grid on
title("Risposta del modello in anello chiuso con disturbi")

[ycl, tcl] = step(feedback(L,1), t);
figure, plot(tcl, ycl), grid on
title("Risposta allo scalino - FdT anello chiuso")

%% CALCOLO SOLUZIONE PER IL RUMORE
R1_n = 10000;
R2_n_num = [1 2];
R2_n_den = [1 1540 196000];
R2_n = tf(R2_n_num, R2_n_den);
disp("Regolatore con rumore e disturbi")
R_noise = series(R2_n, R1_n)

disp("FdT d'anello con rumore e disturbi")
L_noise = series(R_noise,sys_tf)

[mL_n, ~, wL_n] = bode(L_noise);
[GmL_n,PmL_n,~,~] = margin(L_noise);
mL_n = 20*log10(mL_n);
magL_n = mL_n(:);

figure
semilogx(wL_n,magL_n)
hold on
semilogx(wc,0,'g*'),semilogx(wd,Ed, 'ko'),semilogx(wn,En, 'r>')
ylabel('dB'),xlabel('pulsazione'),grid;
title('Diagramma di Bode di L - Modulo')

disp("Margine di fase minimo richiesto")
disp(PHm)
disp("Margine di fase del sistema in anello chiuso")
disp(PmL_n)

%% VERIFICO STABILITA' DI QUESTA SOLUZIONE
figure,nyquist(feedback(L_noise,1)),grid on
zL_n = real(zero(1+L_noise));
disp("Radici di 1+L_noise(s)")
disp(zL_n)
cas = 0;
for i = 1:1:size(zL_n,1)
    if zL_n(i,1) >= 0
        disp("Sistema non as. stabile")
        cas = 1;
    end
end
if cas == 0
    disp("Sistema in anello chiuso as. stabile")
end

%% VERIFICO IL COMPORTAMENTO DEL SISTEMA IN ANELLO CHIUSO
Mdlcl = sim(cln_mdl);
figure, plot(Mdlcl.tout, Mdlcl.yd), grid on
title("Risposta del modello in anello chiuso con rumore")

[ycl_n, tcl_n] = step(feedback(L_noise,1), t);
figure, plot(tcl_n, ycl_n), grid on
title("Risposta allo scalino - FdT anello chiuso")

%% CALCOLO SOLUZIONE PER IL RITARDO PURO
P_num = [1500 19600];
P_den = 10^4*[1 2];
P = tf(P_num, P_den);
disp("FdT d'anello con predittore di Smith")
M = series(feedback(R_noise,P),sys_tf)

bode(M), grid on
figure, nyquist(feedback(M,1)), grid on

%% VERIFICO IL COMPORTAMENTO FINALE DEL MODELLO
MdlFnl = sim(fn_mdl);
figure, plot(MdlFnl.tout, MdlFnl.yd), grid on
title("Risposta del modello in anello chiuso con ritardo")

[yfl, tfl] = step(feedback(M,1), t);
figure, plot(tfl, yfl), grid on
title("Risposta allo scalino - FdT anello chiuso")

%% APRO I MODELLI SIMULINK
open_system(op_mdl),open_system(cld_mdl),open_system(cln_mdl),open_system(fn_mdl)