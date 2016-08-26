#include <Servo.h>


//Adquire o tempo em millisegundos de execução do sistema
unsigned long int globalTime;

//Portas dos motores DC
const int portBackRight = 4;
const int portFrontRight = 5;
const int portBackLeft = 6;
const int portFrontLeft = 7;

//Portas dos Encoders
const int portLeftEncoder = A14;
const int portRightEncoder = A15;

//Portas dos servos motores
const int portHandServo = 9;
const int portArmServo = 10;

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
const int blackLimit = 600;
const int whiteLimit = 100;
const int maxGreenLimit = 400;
const int minGreenLimit = 150;
const int maxGrayLimit = 0;
const int minGrayLimit = 0;

//Variaveis de error (PD)
int globalError;
int lastGlobalError;

//Constantes de proporcionalidade e derivação
const float kP = 21;
const float kD = 4;

//Variavel que armazena o valor da proporcional derivativa
float pd;

//Variaveis que armazenam a velocidade de cada roda (PD)
int speedLeft, speedRight;

//Constantes que armazenam as velocidades iniciais
const int initSpeedLeft = 100;
const int initSpeedRight = 100;
const int globalInitSpeed = 100;

//Limiar para inverter o sentido das rodas
const int limiarToInvert = 40;

//Vetor que armazena os estados dos sensores de refletancia de um modo simplificado
//Padrao: 0 = Branco, 1 = preto, 2 = verde, 3 = cinza
int simpSensorValue[quantityOfSensors];

//Inteira que é setada verdadeira quando algum caso especial de linha é acionado
//Padrao: -1 Curva a direita, -2 Verde a direita, 0 Nenhum caso especial, 1 Curva a esquerda, 2 Verde a direita. 3 = OBSTACULO
int specialCase;

//Refletancia do encoder
int leftEncoderRefletance;
int rightEncoderRefletance;

//Estado e ultimo estado dos encoders (Direita e Esquerda)
bool leftEncoderState;
bool lastLeftEncoderState;
bool rightEncoderState;
bool lastRightEncoderState;

//Limiar de refletancia do encoder
int encoderRefletanceLimiar = 500;

//Variavel que conta o tempo que está num estado do encoder
int encoderTimeCounter;

//Quantidade total de pulsos
long int leftEncoderPulses;
long int rightEncoderPulses;

//Pulsos necessarios para curvas:

//90Graus
const int ninetyDegPulses = 4;

//Frente, 90Graus
const int frontDegPulses = 2;

//Frente, Verde
const int frontGreenPulses = 4;

//Ultima quantidade de pulsos dos encoders
int lastLeftEncoderPulses;
int lastRightEncoderPulses;

//Direção dos motores
bool leftMotorDirection;
bool rightMotorDirection;

//Portas dos sensores de distancia
const int triggerDistanceFront = 23;
const int echoDistanceFront = 22;

const int triggerDistanceLeft = 25;
const int echoDistanceLeft = 24;

const int triggerDistanceRight = 27;
const int echoDistanceRight = 26;

//Variaveis das distancias
float distanceFront;
float distanceLeft;
float distanceRight;

//Distancia para desviar do obstaculo
const float turnDistance = 10.0;

bool rescuing = false;

void setup() {
  Serial.begin(9600);
  //Inicialização das portas dos servos
  handServo.attach(portHandServo);
  armServo.attach(portArmServo);

  //Inicialização das posições
  servoMove('i');
  //distanceFront
  pinMode(triggerDistanceFront, OUTPUT);
  pinMode(echoDistanceFront, INPUT);

  pinMode(triggerDistanceLeft, OUTPUT);
  pinMode(echoDistanceLeft, INPUT);

  pinMode(triggerDistanceLeft, OUTPUT);
  pinMode(echoDistanceLeft, INPUT);

  pinMode(portFrontLeft, OUTPUT);
  pinMode(portBackLeft, OUTPUT);
  pinMode(portFrontRight, OUTPUT);
  pinMode(portBackRight, OUTPUT);

}

void loop() {
  if (!rescuing) {
    getGlobalTime();
    getLineSensorValues();
    globalError = readLine(sensorValue, quantityOfSensors);
    pd = calculatePD(globalError, lastGlobalError);
    getSimpleSensorValue();
    getLineDistances();
    getSpecialCase();
    getEncodersRefletance();
    getEncodersState();
    getEncodersPulse();
    printData();

    if (specialCase == 0) {
      getSpeeds();
      invertSpeed();
    } else {
      doSpecialCase();
    }

  } else {
    searchHostage();

  }
}


void getGlobalTime() {
  globalTime = millis() / 1000;
}


// Leitura dos valores dos sensores e armazenamento no vetor "sensorValues[]"
void getLineSensorValues() {
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
  setMotorDirection(1, 1);
}

//Função que inverte o sentido de giro das rodas quando sua velocidade chega a 0
void invertSpeed() {
  if (speedLeft <= limiarToInvert && speedRight != 0) {
    analogWrite(portFrontLeft, 0);
    analogWrite(portBackLeft, initSpeedRight + (speedRight / 4));
    setMotorDirection(0, 1);
  }

  if (speedRight <= limiarToInvert && speedLeft != 0 ) {
    analogWrite(portFrontRight, 0);
    analogWrite(portBackRight, initSpeedLeft + (speedLeft / 4));
    setMotorDirection(1, 0);
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
      setMotorDirection(1, 1);
      break;

    case 'b':
      analogWrite(portFrontLeft, 0);
      analogWrite(portFrontRight, 0);
      analogWrite(portBackLeft, globalInitSpeed);
      analogWrite(portBackRight, globalInitSpeed);
      setMotorDirection(0, 0);
      break;

    case 'l':
      analogWrite(portFrontLeft, 0);
      analogWrite(portFrontRight, globalInitSpeed);
      analogWrite(portBackLeft, globalInitSpeed);
      analogWrite(portBackRight, 0);
      setMotorDirection(0, 1);
      break;

    case 'r':
      analogWrite(portFrontLeft, globalInitSpeed);
      analogWrite(portFrontRight, 0);
      analogWrite(portBackLeft, 0);
      analogWrite(portBackRight, globalInitSpeed);
      setMotorDirection(1, 0);
      break;
    case 's':
      analogWrite(portFrontLeft, 0);
      analogWrite(portFrontRight, 0);
      analogWrite(portBackLeft, 0);
      analogWrite(portBackRight, 0);
      setMotorDirection(0, 0);
      break;
  }
}

int getNineDeg() {
  if (simpSensorValue[0] == 1 && simpSensorValue[1] == 1 && simpSensorValue[6] == 0 && simpSensorValue[7] == 0) {
    return -1;
  }
  else if (simpSensorValue[0] == 0 && simpSensorValue[1] == 0 && simpSensorValue[6] == 1 && simpSensorValue[7] == 1) {
    return 1;
  }
  else {
    return 0;
  }

}

int getGreen() {
  for (int i = 1; i < quantityOfSensors; i++) {
    if (((sensorValue[i] >= minGreenLimit) && (sensorValue[i] <= maxGreenLimit)) && ((sensorValue[i - 1] >= minGreenLimit) && (sensorValue[i - 1] <= maxGreenLimit)) && ((sensorValue[i - 2] > blackLimit) || (sensorValue[i + 1] > blackLimit))) {
      if (i <= 3) {
        return -1;
      }
      else if (i >= 4) {
        return 1;
      }
      else
        return 0;
    }
  }

  return 0;
}



int getGray() {
  int counter;
  for (int i; i < quantityOfSensors; i++) {
    if (simpSensorValue[i] == 4) {
      counter ++;
    }
  }
  if (counter > 7) {
    counter = 0;
    return 1;

  } else {
    counter = 0;
    return 0;

  }
}

//Funcao que simplifica os valores dos sensores e os armazena num vetor de inteiros
void getSimpleSensorValue() {
  for (int i = 0; i < quantityOfSensors; i++) {

    if (sensorValue[i] > blackLimit) {
      simpSensorValue[i] = 1;

    } else if (sensorValue[i] < maxGrayLimit && sensorValue[i] > minGrayLimit) {
      simpSensorValue[i] = 4;

    } else {
      simpSensorValue[i] = 0;
    }

  }
}

//Funcao que seta o valor de specialCase para o tipo de caso especial que o robo esta tratando
void getSpecialCase() {
  //11xxxx00
  if (getNineDeg() == -1) {
    specialCase = -1;
  }
  //00xxxx11
  else if (getNineDeg() == 1) {
    specialCase = 1;
  }
  else if (getGreen() == -1) {
    specialCase = -2;

  }
  else if (getGreen() == 1) {
    specialCase = 2;

  }

  else if (distanceFront < turnDistance && distanceFront > 2 && globalTime > 20) {
    specialCase = 3;
  }

  else {
    specialCase = 0;
  }
}

//Funcao que transforma um caso especial em sua solucao
void doSpecialCase() {
  switch (specialCase) {
    case -2:
      Serial.println("VERDE DETECTADO - L");
      frontControlled(1);
      leftControlled(6);
      frontControlled(5);
      break;

    case -1:
      //leftDegCurve();
      //leftControlled(1);
      movement('l');
      break;

    case 1:
      //rightDegCurve();
      //rightControlled(1);
      movement('r');
      break;

    case 2:
      Serial.println("VERDE DETECTADO - R");
      frontControlled(1);
      rightControlled(6);
      frontControlled(5);
      break;

    case 3:
      turnObstacle();
      break;

  }
}

void frontControlled(int pulses) {
  lastLeftEncoderPulses = leftEncoderPulses;
  lastRightEncoderPulses = rightEncoderPulses;

  do {
    getEncodersRefletance();
    getEncodersState();
    getEncodersPulse();
    movement('f');
  } while (lastLeftEncoderPulses != leftEncoderPulses - pulses && lastRightEncoderPulses != rightEncoderPulses - pulses);

}

void leftControlled(int pulses) {
  lastLeftEncoderPulses = leftEncoderPulses;
  lastRightEncoderPulses = rightEncoderPulses;

  do {
    getEncodersRefletance();
    getEncodersState();
    getEncodersPulse();
    movement('l');
    printData();
  }  while ((lastLeftEncoderPulses != (leftEncoderPulses + pulses)) && (lastRightEncoderPulses != (rightEncoderPulses - pulses)));

}

void rightControlled(int pulses) {
  lastLeftEncoderPulses = leftEncoderPulses;
  lastRightEncoderPulses = rightEncoderPulses;

  do {
    getEncodersRefletance();
    getEncodersState();
    getEncodersPulse();
    movement('r');
    printData();
  }  while ((lastLeftEncoderPulses != (leftEncoderPulses - pulses)) && (lastRightEncoderPulses != (rightEncoderPulses + pulses)));

}


void backControlled(int pulses) {
  lastLeftEncoderPulses = leftEncoderPulses;
  lastRightEncoderPulses = rightEncoderPulses;

  do {
    getEncodersRefletance();
    getEncodersState();
    getEncodersPulse();
    movement('b');
  } while (lastLeftEncoderPulses != leftEncoderPulses + pulses && lastRightEncoderPulses != rightEncoderPulses + pulses);

}

//Função que adiquire a distancia, by Flalves

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

//Função que gera as distancias
float getLineDistances() {
  distanceFront = getDistance(triggerDistanceFront, echoDistanceFront);
}

void turnObstacle() {
  backControlled(3);
  movement('s');
  delay(100);
  leftControlled(9);
  movement('s');
  delay(100);
  frontControlled(14);
  movement('s');
  delay(100);
  rightControlled(8);
  movement('s');
  delay(100);
  frontControlled(24);
  movement('s');
  delay(100);
  rightControlled(9);
  movement('s');
  delay(100);
  do {
    getLineSensorValues();
    movement('f');

  } while ((sensorValue[0] < blackLimit) && (sensorValue[1] < blackLimit) && (sensorValue[2] < blackLimit) && (sensorValue[3] < blackLimit) && (sensorValue[4] < blackLimit) && (sensorValue[5] < blackLimit) && (sensorValue[6] < blackLimit) && (sensorValue[7] < blackLimit));


  /*frontControlled(15);
    movement('s');
    delay(100);
  */
  frontControlled(5);
  movement('s');
  delay(100);
  leftControlled(9);
  movement('s');
  delay(100);
  backControlled(3);
  movement('s');
  delay(100);
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

void getEncodersRefletance() {
  leftEncoderRefletance = analogRead(portLeftEncoder);
  rightEncoderRefletance = analogRead(portRightEncoder);
}

void getEncodersState() {
  if (leftEncoderRefletance > encoderRefletanceLimiar) {
    encoderTimeCounter++;
    if (encoderTimeCounter > 10) {
      leftEncoderState = 1;
      encoderTimeCounter = 0;
    }
  } else {
    encoderTimeCounter++;
    if (encoderTimeCounter > 10) {
      leftEncoderState = 0;
      encoderTimeCounter = 0;
    }
  }

  if (rightEncoderRefletance > encoderRefletanceLimiar) {
    encoderTimeCounter++;
    if (encoderTimeCounter > 10) {
      rightEncoderState = 1;
      encoderTimeCounter = 0;
    }
  } else {
    encoderTimeCounter++;
    if (encoderTimeCounter > 10) {
      rightEncoderState = 0;
      encoderTimeCounter = 0;
    }
  }
}

//Itera o pulso dos sensores (colocar a diminuição do pulso para tras.)
void getEncodersPulse() {
  if (leftEncoderState != lastLeftEncoderState) {
    if (leftMotorDirection == true) {
      leftEncoderPulses++;
    } else {
      leftEncoderPulses--;
    }
  }
  if (rightEncoderState != lastRightEncoderState) {
    if (rightMotorDirection == true) {
      rightEncoderPulses++;
    } else {
      rightEncoderPulses--;
    }
  }
  lastLeftEncoderState = leftEncoderState;
  lastRightEncoderState = rightEncoderState;

}

//Seta a direção dos motores (1 - frente, 0 - tras)
void setMotorDirection(bool directionLeft, bool directionRight) {
  leftMotorDirection = directionLeft;
  rightMotorDirection = directionRight;
}

void searchHostage() {

}

//Função utilizada para debugging, onde envia as variaveis desejadas para a saida serial
void printData() {
  /*
    Serial.print(speedLeft);
    Serial.print(" ");
    Serial.print(globalError);
    Serial.print(" ");
    Serial.print(speedRight);
    Serial.print(" Green: ");
    Serial.print(getGreen());
    Serial.print(" ");
    Serial.print("Distance Front: ");
    Serial.print(distanceFront);

  */Serial.print(leftMotorDirection);
  Serial.print(" ");
  Serial.print(leftEncoderRefletance);
  Serial.print(" ");
  Serial.print(leftEncoderState);
  Serial.print(" ");
  Serial.print(leftEncoderPulses);
  Serial.print(" ");

  Serial.print(rightEncoderPulses);
  Serial.print(" ");

  Serial.print(rightEncoderState);
  Serial.print(" ");
  Serial.print(rightEncoderRefletance);
  Serial.print(" ");
  Serial.print(rightMotorDirection);

  Serial.println();
}


