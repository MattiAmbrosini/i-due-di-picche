clc
clear
close all

%% INIZIALIZZO LE VARIABILI
b = 0.002;
J = 0.001;

% Setpoint e ritardo puro
y0 = 300;
delay = 0.3;
s = tf('s');
ritardo = exp(-delay*s);

%% DICHIARO LE MATRICI DEL SISTEMA
A = [-b/J, 0;1, 0];
B = [1/J; 0];
C = [0, 1];
D = 0;

%% NOME DEI MODELLI SIMULINK
mdl_A = 'controllo_disco_rigido_openloop';
mdl_1 = 'controllo_disco_rigido_disturbi';
mdl_2 = 'controllo_disco_rigido_rumore';
mdl_3 = 'controllo_disco_rigido_ritardo';

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
t = 0:0.01:5;
[ys, ts] = step(sys_tf, t);

MdlStep = sim(mdl_A);
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

PHm = 65;
disp("Margine di fase minnimo richiesto")
disp(PHm)
disp("Margine di fase del sistema non controllato")
disp(Pm)

%% CALCOLO LE CARATTERISTICHE DEL REGOLATORE
gsys = cell2mat(sys_tf.num);
R1 = 10^(round((((Ed)+20*wd(end))/20)-1))/gsys(3);
R2_zero = [-2];
R2_pole = [-500];

R2_num = s-R2_zero;
R2_den = s-R2_pole;

disp("FdT del regolatore")
R = series(R1,(R2_num/R2_den))

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
title("Diagramma di Nyquist - L")

disp("Radici di 1+L(s)")
ztf(L)

%% VERIFICO IL COMPORTAMENTO DEL SISTEMA IN ANELLO CHIUSO
MdlCL = sim(mdl_1);
figure, plot(MdlCL.tout, MdlCL.y), grid on
title("Risposta del modello in anello chiuso con disturbi")

[ycl, tcl] = step(feedback(L,1), t);
figure, plot(tcl, ycl), grid on
title("Risposta allo scalino - FdT anello chiuso")

%% CALCOLO SOLUZIONE PER IL RUMORE
R1_n = 35000;
R2_zero_n = [-2];
R2_pole_n = [-500, -1400];

R2_num_n = s-R2_zero_n;
R2_den_n = (s-R2_pole_n(1))*(s-R2_pole_n(2));

disp("Regolatore con rumore e disturbi")
R_noise = series(R1_n,(R2_num_n/R2_den_n))

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
title("Diagramma di Nyquist - L noise")

disp("Radici di 1+L_noise(s)")
ztf(L_noise)

%% VERIFICO IL COMPORTAMENTO DEL SISTEMA IN ANELLO CHIUSO
Mdlcl = sim(mdl_2);
figure, plot(Mdlcl.tout, Mdlcl.y), grid on
title("Risposta del modello in anello chiuso con rumore")

[ycl_n, tcl_n] = step(feedback(L_noise,1), t);
figure, plot(tcl_n, ycl_n), grid on
title("Risposta allo scalino - FdT anello chiuso")

%% CALCOLO SOLUZIONE PER IL RITARDO PURO
L_delay = L_noise*ritardo;
[y_r, t_r] = step(feedback(L_delay,1), t);
figure, plot(t_r, y_r), grid on
title("Effetto del ritardo")
figure, nyquist(feedback(L_delay,1)),grid on

%predittore di Smith
M = series(series(feedback(R_noise,series(1-ritardo,sys_tf)),sys_tf),ritardo);

%soluzione con polo e k
W_pole = max(R2_pole_n);
k = (1/67.5)*gsys(3);
W = k/(s-W_pole);
N = series(W, L_delay);

%% VERIFICO LA STABILITA' DELLA SOLUZIONE
figure, nyquist(feedback(N,1)),grid on
title("Diagramma di Nyquist - N")

disp("Radici di 1+N(s)")
ztf(N)

%% VERIFICO IL COMPORTAMENTO FINALE DEL MODELLO
MdlFnl = sim(mdl_3);
figure, plot(MdlFnl.tout, MdlFnl.y_p), grid on
hold on, plot(MdlFnl.tout, MdlFnl.y), legend ('Predittore Smith','Polo e k')
title("Risposta del modello - Ritardo")

[y_p, t_p] = step(feedback(M,1),t);
[y_w, t_w] = step(feedback(N,1),t);
figure, plot(t_p, y_p), grid on, hold on, plot(t_w, y_w)
title("Risposta allo scalino - Ritardo")
legend ('Predittore Smith','Polo e k')

%% APRO I MODELLI SIMULINK
open_system(mdl_A),open_system(mdl_1),open_system(mdl_2),open_system(mdl_3)

%% FUNZIONE CALCOLO ZERI DI 1+TF(S)
function ztf(tf)
    z = real(zero(1+tf));
    disp(z)
    cas = 0;
    for i = 1:1:size(z,1)
        if z(i,1) >= 0
            disp("Sistema non as. stabile")
            cas = 1;
        end
    end
    if cas == 0
        disp("Sistema in anello chiuso as. stabile")
    end
end