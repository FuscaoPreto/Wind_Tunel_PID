#include <Wire.h>
#include "ClosedCube_HDC1080.h"
#include <Bme280.h>

// Definições de pinos e constantes
const int analogPin = 1;      // Pino ADC
const int windowAnalog = 100;  // Tamanho da janela da média móvel
const int windowRPM = 50;     // Tamanho da janela para o cálculo da média móvel de RPM
const int tacometroPin = 15;  // Pino onde o sinal do tacômetro está conectado
const int pwmPin = 16;        // Pino de saída PWM
const int pwmRes = 10;        // Resolução do PWM
const int pwmFreq = 25000;    // Frequência do PWM
const int pinSDA = 12;
const int pinSCL = 11;
// Variáveis para os arrays de média móvel e índices
volatile int analogValues[windowAnalog];  // Array para armazenar os valores da janela de leituras analógicas
volatile int analogIndex = 0;             // Índice atual para a média móvel de leituras analógicas
volatile float analogSum = 0;             // Soma das leituras analógicas para média móvel
volatile int rpmValues[windowRPM];        // Array para armazenar os valores da janela de RPM
volatile int rpmIndex = 0;                // Índice atual para a média móvel de RPM
volatile float rpmSum = 0;                // Soma das leituras de RPM para média móvel

// Variáveis para controle de tempo e cálculo de RPM
volatile unsigned long lastPulseTime = 0;  // Tempo do último pulso do tacômetro
volatile unsigned long pulseInterval = 0;  // Intervalo entre os pulsos do tacômetro
volatile int rpm = 0;                      // Velocidade do motor em RPM
volatile float filteredRPM = 0;

// Variáveis para cálculo de velocidade do ar e pressão
volatile float airspeed = 0;
volatile double pressure = 0;
volatile double volts = 0;
volatile int offset = 0;
volatile double atmPressao = 0;
volatile double umidade = 0;
volatile double temperatura = 0;
volatile double tempk = 0;
volatile double psat = 0;
volatile double pv = 0;
volatile double pd = 0;
volatile double pha = 0;
volatile float filteredAnalog = 0;
// Variáveis para controle PID
double integral = 0, output = 0, dt = 0;
double ki = 27.6;
double kp = 138;
double kd = 43.125;
volatile double proportional = 0, previous = 0, derivative = 0, error = 0;
double targetSpeed = 8.0;
volatile double pid = 0;


// Variáveis de PWM e tempo
int pwmValue = 0;                     // Valor inicial do PWM
unsigned long lastPwmUpdateTime = 0;  // Para controlar o tempo de incremento ou decremento do PWM

//Objects
ClosedCube_HDC1080 hdc;
Bme280TwoWire bme;


void updateData() {
  temperatura = hdc.readTemperature();
  umidade = hdc.readHumidity();
  atmPressao = bme.getPressure();
  tempk = temperatura + 273.15;
  psat = 0.61078 * exp((17.27 * temperatura) / (temperatura + 237.3));
  pv = psat * (umidade / 100.0);
  pd = atmPressao - pv;
  pha = (pd / (287.058 * tempk)) + (pv / (461.495 * temperatura + 273.15));
}

void IRAM_ATTR tacometroInterrupt() {
  unsigned long currentTime = micros();


  // Calcula o intervalo entre pulsos
  if (lastPulseTime > 0) {
    pulseInterval = currentTime - lastPulseTime;       // Intervalo entre os pulsos
    rpm = 60000000.0 / (pulseInterval * 2);            // Calcula a RPM com base no intervalo
    int analogValue = analogRead(analogPin) - offset;  // Lê o valor analógico e ajusta o offset

    // Calcula a média dos valores na janela
    float meanAnalog = analogSum / windowAnalog;
    float thresholdAnalog = meanAnalog * 0.02;  // Define o limite como 1% da média atual

    // Verifica se o valor lido está dentro do limite de 10% em relação à média
    if (abs(analogValue - meanAnalog) <= thresholdAnalog) {
      // Remove o valor mais antigo da soma e atualiza o array
      analogSum -= analogValues[analogIndex];
      analogValues[analogIndex] = analogValue;
      analogSum += analogValue;

      // Atualiza o índice da janela circular
      analogIndex = (analogIndex + 1) % windowAnalog;
    }

    // Calcula a média móvel sem incluir outliers
    filteredAnalog = analogSum / windowAnalog;

    // Calcula a média dos valores na janela
    float meanRPM = rpmSum / windowRPM;
    float thresholdRPM = meanRPM * 0.2;  // Define o limite como 1% da média atual

    // Verifica se o valor lido está dentro do limite de 10% em relação à média
    if (abs(rpm - meanRPM) <= thresholdRPM) {
      // Remove o valor mais antigo da soma e atualiza o array
      rpmSum -= rpmValues[rpmIndex];
      rpmValues[rpmIndex] = rpm;
      rpmSum += rpm;

      // Atualiza o índice da janela circular
      rpmIndex = (rpmIndex + 1) % windowRPM;
    }

    // Calcula a média móvel sem incluir outliers
    filteredRPM = rpmSum / windowRPM;

    // Conversão para tensão, pressão e velocidade do ar
    volts = filteredAnalog * 3.3 / 4096;
    pressure = volts * (4000 / 3.3) - 2000;
    if (pressure < 0) pressure = 0.0001;
    airspeed = sqrt(2 * (pressure / pha));

    //if(integral>0) pressure=0.0001;
    error = targetSpeed - airspeed;
    error = error * (1750.0 / 5.2);
    proportional = error;
    dt = pulseInterval / 1000000.0;
    integral += error * dt;
    derivative = (error - previous) / dt;
    previous = error;
    pid = (kp * proportional) + (ki * integral) + (kd * derivative);
    pwmValue = pid / (1750.0 / 5.2);

    if (pwmValue < 256) {
      pwmValue = 256;
    }
    if (pwmValue > 1023) {
      pwmValue = 1023;
    }


    ledcWrite(pwmPin, pwmValue);
  }

  // Atualiza o tempo do último pulso
  lastPulseTime = currentTime;
}

void setup() {
  offset = analogRead(analogPin) - 2048;
  Serial.begin(921600);  // Inicializa a comunicação serial
  for (int i = 0; i < windowAnalog; i++) {
    analogValues[i] = 2048;  // Inicializa o array com 2048, valor base para nenhuma pressão
    analogSum += analogValues[i];
  }
  for (int i = 0; i < windowRPM; i++) {
    rpmValues[i] = 3400;  // Inicializa o array com 2048, valor base para nenhuma pressão
    rpmSum += rpmValues[i];
  }
  pinMode(tacometroPin, INPUT_PULLUP);  // Configura o pino com pull-up interno
  Wire.begin(pinSDA, pinSCL);
  hdc.begin(0x40);
  bme.begin(Bme280TwoWireAddress::Primary);

  Serial.println("Filtros iniciados");
  delay(1000);
  Serial.println("5 Segundos para iniciar");
  delay(1000);
  Serial.println("4 Segundos para iniciar");
  delay(1000);
  Serial.println("3 Segundos para iniciar");
  delay(1000);
  Serial.println("2 Segundos para iniciar");
  delay(1000);
  Serial.println("1 Segundos para iniciar");
  delay(1000);

  updateData();

  attachInterrupt(digitalPinToInterrupt(tacometroPin), tacometroInterrupt, FALLING);

  // Configuração do PWM utilizando a função ledcAttach
  ledcAttach(pwmPin, pwmFreq, pwmRes);  // Associa o pino e configura a frequência e a resolução do PWM
}

void loop() {
  Serial.print(pwmValue);
  Serial.print(", ");
  Serial.print(pid);
  Serial.print(", ");
  Serial.print(error);
  Serial.print(", ");
  Serial.print(integral);
  Serial.print(", ");
  Serial.print(derivative);
  Serial.print(", ");
  Serial.print(filteredRPM);
  Serial.print(", ");
  Serial.print(pressure);
  Serial.print(", ");
  Serial.println(airspeed);
}