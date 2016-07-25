#include <Servo.h>

//Portas dos motores DC
const int portBackRight = 4;
const int portFrontRight = 5;
const int portBackLeft = 6;
const int portFrontLeft = 7;

//Portas dos servos motores
const int portHand = 9;
const int portArm = 10;

//Constantes para as portas e quantidade de sensores de linha
const int quantityOfSensors = 8;
const int sensorPort[quantityOfSensors] = {A7, A6, A5, A4, A3, A2, A1, A0};

//Vetor que armazena os valores dos sensores de linha
int sensorValue[quantityOfSensors];

//Armazenamento das posicoes dos servos
int handServoPosition = 180; //fechar = 180 - abrir = 105
int armServoPosition = 0; //em baixo = 0 - em cima = 180

//Criação dos objetos Servo
Servo handServo;
Servo armServo;

//Limiar entre o preto e o branco
const int blackLimit = 700;
const int whiteLimit = 100;
const int maxGreenLimit = blackLimit - 50;
const int minGreenLimit = whiteLimit + 50;

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

//Constantes que armazenam as velocidades iniciais
const int initSpeedLeft = 75;
const int initSpeedRight = 75;
const int globalInitSpeed = 100;

//Limiar para inverter o sentido das rodas
const int limiarToInvert = 10;

//Vetor que armazena os estados dos sensores de refletancia de um modo simplificado
//Padrao: 0 = Branco, 1 = preto, 2 = verde, 3 = cinza
int simpSensorValue[quantityOfSensors];

//Inteira que é setada verdadeira quando algum caso especial de linha é acionado
//Padrao: -1 Curva a direita, -2 Verde a direita, 0 Nenhum caso especial, 1 Curva a esquerda, 2 Verde a direita.
int specialCase;

//Contador que será iterado enquanto o specialCase for supostamente igual a 0
int specialCaseCounted;

float distanceFront;
void setup() {
  Serial.begin(9600);
  //Inicialização das portas dos servos
  handServo.attach(8);
  armServo.attach(9);

  //Inicialização das posições
  servoMove('i');

  //distanceFront
  pinMode(53, INPUT);
  pinMode(52, OUTPUT);
}

void loop() {
  getSensorValues();
  globalError = readLine(sensorValue, quantityOfSensors);
  pd = calculatePD(globalError, lastGlobalError);

  getSimpleSensorValue();
  getSpecialCase();
  doSpecialCase();
  lineUpdateDistances();
  if (specialCase == 0) {
    getSpeeds();
    invertSpeed();
  }
  printData();

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
  speedLeft = constrain(speedLeft, 0, 254);
  speedRight = constrain(speedRight, 0, 254);

  analogWrite(portFrontLeft, speedLeft);
  analogWrite(portBackLeft, 0);
  analogWrite(portFrontRight, speedRight);
  analogWrite(portBackRight, 0);
}

//Função que inverte o sentido de giro das rodas quando sua velocidade chega a 0
void invertSpeed() {
  if (speedLeft <= limiarToInvert && speedRight != 0) {
    analogWrite(portFrontLeft, 0);
    analogWrite(portBackLeft, initSpeedRight + (speedRight / 4));
  }

  if (speedRight <= limiarToInvert && speedLeft != 0 ) {
    analogWrite(portFrontRight, 0);
    analogWrite(portBackRight, initSpeedLeft + (speedLeft / 4));
  }

}
//Funcao de movimentos primarios
//Padrao: f - Front, b - Back, l - Left, r - Right
void movement(char movementType) {

  switch (movementType) {
    case 'f':
      analogWrite(portFrontLeft, globalInitSpeed);
      analogWrite(portFrontRight, globalInitSpeed);
      analogWrite(portBackLeft, 0);
      analogWrite(portBackRight, 0);
      break;

    case 'b':
      analogWrite(portFrontLeft, 0);
      analogWrite(portFrontRight, 0);
      analogWrite(portBackLeft, globalInitSpeed);
      analogWrite(portBackRight, globalInitSpeed);
      break;

    case 'l':
      analogWrite(portFrontLeft, 0);
      analogWrite(portFrontRight, globalInitSpeed);
      analogWrite(portBackLeft, globalInitSpeed + 50);
      analogWrite(portBackRight, 0);
      break;

    case 'r':
      analogWrite(portFrontLeft, globalInitSpeed);
      analogWrite(portFrontRight, 0);
      analogWrite(portBackLeft, 0);
      analogWrite(portBackRight, globalInitSpeed + 50);
      break;
  }
}

//Funcao que simplifica os valores dos sensores e os armazena num vetor de inteiros
void getSimpleSensorValue() {
  for (int i = 0; i < quantityOfSensors; i++) {

    if (sensorValue[i] > blackLimit) {
      simpSensorValue[i] = 1;

    } else {
      simpSensorValue[i] = 0;
    }
  }
}

//Funcao que seta o valor de specialCase para o tipo de caso especial que o robo esta tratando
void getSpecialCase() {
  //11xxxx00
  if (simpSensorValue[0] == 1 && simpSensorValue[1] == 1 && simpSensorValue[6] == 0 && simpSensorValue[7] == 0) {
    specialCase = -1;
  }

  //00xxxx11
  else if (simpSensorValue[0] == 0 && simpSensorValue[1] == 0 && simpSensorValue[6] == 1 && simpSensorValue[7] == 1) {
    specialCase = 1;
  }
  else {
    specialCase = 0;
  }
}

//Funcao que transforma um caso especial em sua solucao
void doSpecialCase() {
  switch (specialCase) {
    case -1:
      leftDegCurve();
      break;

    case 1:
      rightDegCurve();
      break;

  }
}

//(Encoders!!!)
void leftDegCurve() {
  movement('l');
}

//(Encoders!!!)
void rightDegCurve() {
  movement('r');
}

float getDistance(int trigPin, int echoPin) {
  float duration, distance;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH, 7000); // 7000 Delay Microsseconds to limit range

  distance = (duration / (float)2) / 29.1;

  return distance;
}

float lineUpdateDistances() {
  distanceFront = getDistance(52, 53);
}

void servoMove(char movementType) {

  switch (movementType) {
    //close
    case 'c':
      for (int i = 105; i < 180; i++) {
        handServoPosition = i;
      }
      break;
    //open
    case 'o':
      for (int i = 180; i > 105; i--) {
        handServoPosition = i;
      }
      break;
    //down
    case 'd':
      //Checagem feita para que o braço não gire com a mão aberta
      if (handServoPosition == 180) {
        for (int i = 180; i > 105; i--) {
          armServoPosition = i;
        }
      } else {
        Serial.println("ERROR, A MAO CORRE PERIGO DE DANO");
      }
      break;
    //up
    case 'u':
      //Checagem feita para que o braço não gire com a mão aberta
      if (handServoPosition == 180) {
        for (int i = 0; i < 180; i++) {
          armServoPosition = i;
        }
      } else {
        Serial.println("ERROR, A MAO CORRE PERIGO DE DANO");
      }
      break;

    case 'i':
      for (int i = 105; i < 180; i++) {
        handServoPosition = i;
      }

      for (int i = 0; i < 180; i++) {
        armServoPosition = i;
      }

      for (int i = 180; i > 100; i--) {
        handServoPosition = i;
      }
      break;
  }
  handServo.write(handServoPosition);
  armServo.write(armServoPosition);
}



//Função utilizada para debugging, onde envia as variaveis desejadas para a saida serial
void printData() {
  Serial.print(speedLeft);
  Serial.print(" ");
  Serial.print(globalError);
  Serial.print(" ");
  Serial.print(speedRight);
  Serial.print(" ");
  Serial.print(specialCase);
  Serial.print(" ");
  Serial.print(distanceFront);
  Serial.println();
}


