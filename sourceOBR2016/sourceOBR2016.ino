//Portas dos motores DC
const int portBackRight = 4;
const int portFrontRight = 5;
const int portBackLeft = 6;
const int portFrontLeft = 7;

//Constantes para as portas e quantidade de sensores de linha
const int quantityOfSensors = 8;
const int sensorPort[quantityOfSensors] = {A7, A6, A5, A4, A3, A2, A1, A0};

//Vetor que armazena os valores dos sensores de linha
int sensorValue[quantityOfSensors];

//Limiar entre o preto e o branco
const int blackLimit = 650;

//Variaveis de error (PD)
int globalError;
int lastGlobalError;

//Constantes de proporcionalidade e derivação
const float kP = 17.8;
const float kD = 4;

//Variavel que armazena o valor da proporcional derivativa
float pd;

//Variaveis que armazenam a velocidade de cada roda (PD)
int speedLeft, speedRight;

const int initSpeedLeft = 100;
const int initSpeedRight = 100;

void setup() {


}

void loop() {
  getSensorValues();
  globalError = readLine(sensorValue, quantityOfSensors);
  pd = calculatePD(globalError, lastGlobalError);
  getSpeeds();

}

// Leitura dos valores dos sensores e armazenamento no vetor "sensorValues[]"
void getSensorValues() {
  for (int i = 0; i < quantityOfSensors; i++) {
    sensorValue[i] = analogRead(sensorPort[i]);
  }
}

//Funcao que recebe os dados dos sensores e retorna o error(PD)
long readLine(int sensors[], int sensorQuantity) {
  long numerator = 0;
  long denominator = 0;

  for (int i = 0; i < sensorQuantity; i++) {
    numerator += (i * 1000 * (long)sensors[i]);
    denominator += sensors[i];
  }

  return map((numerator / denominator) , 950 , 6000 , -7 , 7);

}

//Funcao que atualiza o ultimo error(PD)
void updateLastError(int lastError) {
  lastGlobalError = lastError;
}

//Funcao que atualiza o error(PD)
void updateError(int error) {
  globalError = error;
}

//Funcao que calcula a proporcional derivativa
float calculatePD(int error, int lastError) {
  int P = error;
  int D = error - lastError;

  lastError = error;
  updateLastError(lastError);

  return (P * kP) + (D * kD); 
}

//Atualiza as velocidades e mantem dentro de uma range
void getSpeeds() {
  
  speedLeft = initSpeedLeft + pd;
  speedRight = initSpeedRight - pd;
  speedLeft = constrain(speedLeft, 10, 254);
  speedRight = constrain(speedRight, 10, 254);

  analogWrite(portFrontLeft, speedLeft);
  analogWrite(portBackLeft, 0);
  analogWrite(portFrontRight, speedRight);
  analogWrite(portBackRight, 0);
}
