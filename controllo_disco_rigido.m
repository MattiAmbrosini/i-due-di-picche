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

fn_mdl = 'controllo_disco_rigido_final';
open_system(fn_mdl)

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
% [~,~,~,Wcp] = margin(sys_tf);
% disp(Wcp)

disp("Stima della banda del sistema")
bw = [w(1) 0];
for i = 1:1:size(mag,1)
    if mag(i,1) < 0
        bw(2) = w(i,1);
        break
    end
end
disp(bw)

figure, semilogx(w(1:i), mag(1:i)), grid on
title("Diagramma di Bode del modulo della funzione per la banda stimata")

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
L = sys_tf*R

% disp("FdT in anello chiuso")
sys_cl = sys_tf/(1+L);

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
figure, nyquist(sys_cl), grid on
zL = real(zero(1+L));
disp("Radici di 1+L(s)")
disp(zL)
cas = 0;
for i = 1:1:size(zL,1)
    if zL >= 0
        disp("Sistema non as. stabile")
        cas = 1;
    end
end
if cas == 0
    disp("Sistema as. stabile")
end

%% VERIFICO IL COMPORTAMENTO DEL SISTEMA IN ANELLO CHIUSO
MdlCL = sim(cl_mdl);
figure, plot(MdlCL.tout, MdlCL.y), grid on
title("Risposta del modello in anello chiuso con disturbi")

[ycl, tcl] = step(sys_cl, t);
figure, plot(tcl, ycl), grid on
title("Risposta allo scalino - FdT anello chiuso")

%% CALCOLO SOLUZIONE PER IL RUMORE

%% CALCOLO SOLUZIONE PER IL RITARDO PURO

%% VERIFICO IL COMPORTAMENTO FINALE DEL MODELLO