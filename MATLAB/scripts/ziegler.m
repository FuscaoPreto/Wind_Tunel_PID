% Carregar e extrair dados do arquivo .log
data = readtable('realdata.txt', 'Delimiter', ',', 'FileType', 'text');
time = data.Var1 / 1e6;  % Converter o tempo para segundos
error = data.Var2;
pid = data.Var3;
pressure = data.Var4;
speed = data.Var5;
pwm = data.Var6;

% 1. Gráfico de pressão e velocidade ao longo do tempo, em subplots separados
figure;
subplot(2,1,1); % Primeiro subplot para pressão
plot(time, pressure, '-b');
xlabel('Time (s)');
ylabel('Pressure');
title('Pressure over Time');
grid on;

subplot(2,1,2); % Segundo subplot para velocidade
plot(time, speed, '-r');
xlabel('Time (s)');
ylabel('Speed');
title('Speed over Time');
grid on;

% Exportar a primeira figura em alta resolução
exportgraphics(gcf, 'pressure_speed_over_time.png', 'Resolution', 300);

% 2. Gráfico de erro, PID e integral ao longo do tempo, em subplots separados
integral = cumtrapz(time, error); % Calculo da integral do erro
figure;
subplot(3,1,1); % Primeiro subplot para erro
plot(time, error, '-k');
xlabel('Time (s)');
ylabel('Error');
title('Error over Time');
grid on;

subplot(3,1,2); % Segundo subplot para PID
plot(time, pid, '-g');
xlabel('Time (s)');
ylabel('PID');
title('PID over Time');
grid on;

subplot(3,1,3); % Terceiro subplot para integral do erro
plot(time, integral, '-m');
xlabel('Time (s)');
ylabel('Integral of Error');
title('Integral of Error over Time');
grid on;

% Exportar a segunda figura em alta resolução
exportgraphics(gcf, 'error_pid_integral_over_time.png', 'Resolution', 300);

% 3. Gráfico de PWM e velocidade ao longo do tempo, em subplots separados
figure;
subplot(2,1,1); % Primeiro subplot para PWM
plot(time, pwm, '-c');
xlabel('Time (s)');
ylabel('PWM');
title('PWM over Time');
grid on;

subplot(2,1,2); % Segundo subplot para velocidade
plot(time, speed, '-r');
xlabel('Time (s)');
ylabel('Speed');
title('Speed over Time');
grid on;

% Exportar a terceira figura em alta resolução
exportgraphics(gcf, 'pwm_speed_over_time.png', 'Resolution', 300);
