% Importar dados do arquivo .log
data = readtable('valid.log', 'Delimiter', ',', 'FileType', 'text');

% Extrair as colunas
pwm = data.Var1;                  % Coluna de entrada (PWM)
time_microseconds = data.Var2;    % Coluna de tempo em microsegundos
response_period = data.Var3;       % Coluna de saída (período entre pulsos em microsegundos)

% Converter o tempo de microsegundos para segundos
time_seconds = time_microseconds / 1e6;
response_period_seconds = response_period / 1e6; % Período em segundos

% Calcular RPM a partir do período
% A cada pulso, correspondem duas rotações (considerando que temos um pulso por metade da rotação)
rpm = (60 ./ (response_period_seconds * 2))-900; 

% Filtro de média móvel para remover outliers nos dados originais
window_size_outliers = 10; % Tamanho da janela para média móvel (ajuste conforme necessário)

% Calcular a média móvel
moving_avg = movmean(rpm, window_size_outliers, 'omitnan');

% Calcular o desvio padrão da média móvel
std_dev = movstd(rpm, window_size_outliers, 'omitnan');

% Definir limite para identificar outliers
threshold = 1; % Limite de 0.5 desvios padrão

% Identificar outliers
is_outlier = abs(rpm - moving_avg) > (threshold * std_dev);

% Substituir outliers por NaN
rpm_filtered = rpm;
rpm_filtered(is_outlier) = NaN;

% Interpolar novamente os dados filtrados
rpm_filtered = interp1(time_seconds(~isnan(rpm_filtered)), rpm_filtered(~isnan(rpm_filtered)), time_seconds, 'linear');

% Aplicar um filtro de média móvel aos dados filtrados
window_size_smoothing = 10; % Tamanho da janela para suavização (ajuste conforme necessário)
rpm_smoothed = movmean(rpm_filtered, window_size_smoothing, 'omitnan');

% Substituir NaN pela média dos dados suavizados
mean_value = mean(rpm_smoothed, 'omitnan'); % Calcular a média ignorando NaN
rpm_smoothed(isnan(rpm_smoothed)) = mean_value; % Substituir NaN pela média

% Definir a nova frequência de amostragem
Fs = 1000; % Frequência de amostragem desejada em Hz
Ts = 1 / Fs; % Período de amostragem

% Criar um vetor de tempo reamostrado
time_uniform = (time_seconds(1):Ts:time_seconds(end))';

% Interpolar os dados suavizados
rpm_uniform = interp1(time_seconds, rpm_smoothed, time_uniform, 'linear');
pwm_uniform = interp1(time_seconds, pwm, time_uniform, 'linear');

% Criar objeto iddata com os dados reamostrados
vetorrpm4 = iddata(rpm_uniform, pwm_uniform, Ts);

% Plotar os dados
figure;
plot(time_seconds, rpm, 'b-', 'DisplayName', 'RPM Originais'); % Dados originais de RPM
hold on;
plot(time_seconds, rpm_filtered, 'r-', 'DisplayName', 'RPM Filtrados'); % Dados filtrados de RPM
plot(time_seconds, rpm_smoothed, 'g-', 'DisplayName', 'RPM Suavizados'); % Dados suavizados de RPM
xlabel('Tempo (s)');
ylabel('RPM');
legend;
title('Comparação de Dados Originais, Filtrados e Suavizados de RPM');
grid on;
hold off;
