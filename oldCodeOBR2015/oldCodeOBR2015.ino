/*Criar condição para retornar frente quando encontrar preto em todos os sensores. !!!*/
/*
- seguir linha (com gap também)
- desviar de obstaculo (caixa de leite)
- encontrar vitima (bola de isopor revestida de papel aluminio) - mais de uma vítima
- encontrar base (arena preta triangular - 6cm altura)
- identificar se é seguidor, ou busca (colocar a vitima e dar um sinal de finalizado)

- verifica posição da linha na barra - atribui um valor (-7 a 7)

*/
#include <Servo.h>

Servo handServo;
Servo armServo;

unsigned long int timeElapsed = 0;

int greenPosition;
int crossLine;
bool line;
int curvePosition;

bool handClosed = true;
bool armHigh = true;

int esperado = 0;

const float batteryMultiplier = 1.0;

const int motorLF = 4;
const int motorLB = 6;
const int motorRF = 5;
const int motorRB = 7;

int pwm_inicial_D = 90;
int pwm_inicial_E = 130;

int pwmD = 0;
int pwmE = 0;

const int quantityOfSensors = 8;

const int sensors[quantityOfSensors] = {A7, A6, A5, A4, A3, A2, A1, A0};
int sensorValues[quantityOfSensors];

const int blackLimit = 650; // >= 540
const int whiteLimit = 150; // <= 55
const int minGreenValue = 100;
const int maxGreenValue = 450;
const int silverLimiar = 200; // Atualizar este valor

const float kP = 14.6;
const float kD = 4;

//Distances
float  distanceRightDown;
float  distanceRightTop;
float  distanceFrontDown;
float  distanceFrontTop;
float  distanceLeftDown;
float  distanceLeftTop;


int pwmR = 0;
int pwmL = 0;
int globalError = 0;
int lastGlobalError = 0;

bool rescuing = false;

void setup() {
  Serial.begin(9600);
  
  handServo.attach(25);
  armServo.attach(27);
  HighArm();
  Stop();
  delay(400);
  OpenHand();
  //distanceFrontTop
  pinMode(31, INPUT);
  pinMode(29, OUTPUT);
  
  //distanceFrontDown
  pinMode(23, OUTPUT);
  pinMode(22, INPUT);

  //distanceLeftDown
  pinMode(2, INPUT);
  pinMode(3, OUTPUT);

  //distanceLeftTop
  pinMode(14, INPUT);
  pinMode(15, OUTPUT);

  //distanceRightDown
  pinMode(16, INPUT);
  pinMode(17, OUTPUT);
  
  //distanceRightTop
  pinMode(18, INPUT);
  pinMode(19, OUTPUT);

  //Motors
  pinMode(motorLF, OUTPUT);
  pinMode(motorLB, OUTPUT);
  pinMode(motorRF, OUTPUT);
  pinMode(motorRB, OUTPUT);
}

void loop() {

  if (!rescuing) { // Seguir Linha e desviar de obstáculo

    // Avaliar condição da pista
    //   Avaliar condição do verde
    //   Avaliar condição da linha preta
    //   Se tiver verde, curva para o lado do verde
    //   Senão se tiver linha preta do lado guardar uma memoria apontando para curva
    //     Se não tiver linha depois, curva para o lado da memoria
    //     Senão limpa a memoria da curva 
    timeElapsed = (millis() / 1000);
    distanceFrontDown = getDistance(23, 22);
    distanceFrontTop = getDistance(29, 31);

    Serial.print("  distanceFrontDown: ");
    Serial.println(distanceFrontDown);
    Serial.print("  timeElapsed: ");
    Serial.print(timeElapsed);
    for (int i = 0; i < quantityOfSensors; i++) { // Leitura dos valores dos sensores e armazenamento no vetor "sensorValues[]"
      sensorValues[i] = analogRead(sensors[i]);
      // delay(10);
    }
    globalError = readLine(sensorValues, quantityOfSensors);
    float pid = calculatePD(globalError, lastGlobalError);

    pwmD = pwm_inicial_D - pid;
    pwmE = pwm_inicial_E + pid;
    pwmD = constrain(pwmD, 10, 255);
    pwmE = constrain(pwmE, 10, 255);

    analogWrite(motorLF, pwmE);
    analogWrite(motorLB, 0);

    analogWrite(motorRF, pwmD);
    analogWrite(motorRB, 0);


    if (pwmE < 40 && pwmD > 40) {

      analogWrite(motorLF, 0);
      analogWrite(motorLB, 120);
      analogWrite(motorRF, 80);
      analogWrite(motorRB, 0);

    } else if (pwmE > 40 && pwmD < 40) {

      analogWrite(motorLF, 120);
      analogWrite(motorLB, 0);
      analogWrite(motorRF, 0);
      analogWrite(motorRB, 80);

    }  
    
    if(minGreenValue < sensorValues[0] && sensorValues[1] && sensorValues[2] && sensorValues[3] && sensorValues[4] && sensorValues[5] && sensorValues[6] && sensorValues[7] > maxGreenValue){
    Front();
    Serial.println("TA NA HORA DO RESGATE!");
    delay(500 * batteryMultiplier);
    }
    

    //Serial.print(pwmE);
    //Serial.print('\t');
    // Serial.print(globalError);
    //Serial.print('\t');
    //Serial.println(pwmD);

    greenPosition = identifyGreen();
    Serial.print("  Green Position: ");
    Serial.print(greenPosition);

    if (greenPosition == 1) {
      SpinR();
      delay(1000 * batteryMultiplier);
      Front();
      delay(500 * batteryMultiplier);
    }
    else if (greenPosition == -1) {
      SpinL();
      delay(1000  * batteryMultiplier);
      Front();
      delay(500 * batteryMultiplier);
    }


    line = identifyLine();
    getCross();
    //  line2 = identifyLine();
    //if(line != line2){
    // line2 = identifyLine();
    //}

    identifyCurve();
    
    Serial.print("\tCurve Position: ");
    Serial.print(curvePosition);
    Serial.print("\tBool Line: ");
    Serial.print(line);
    //Serial.print(" esperado: ");
    //Serial.println(esperado);
    Serial.print("\tglobalError: ");
    Serial.print(globalError);
    Serial.print("\tcrossLine: ");
    Serial.println(crossLine);
    if (esperado > 2) {
      esperado = 0;
      curvePosition = 0;

    }

    if (curvePosition == 1) {
      Front();
      esperado++;
      Serial.println("ESPERE!!!");
      delay(200 * batteryMultiplier);
      if (line == false && esperado == 2) {
        Back();
        delay(500 * batteryMultiplier);
        SpinR();
        delay(1000 * batteryMultiplier);
        Front();
        delay(500 * batteryMultiplier);
        Serial.println("CONSIDERADO");
        curvePosition = 0;
      }
    }

    else if (curvePosition == -1) {
      Front();
      esperado++;
      Serial.println("ESPERE!!!");
      delay(250 * batteryMultiplier);
      if (line == false && esperado == 2) {
        Back();
        delay(500 * batteryMultiplier);
        SpinL();
        delay(1000 * batteryMultiplier);
        Front();
        delay(500 * batteryMultiplier);
        Serial.println("CONSIDERADO");
        curvePosition = 0;
      }

    }
    else if (crossLine == 1) {
      Front();
      Serial.println("nego é cabeça de gelo");
      delay(500 * batteryMultiplier);
      curvePosition = 0;
    }
    
  if(distanceFrontDown < 20 && distanceFrontDown != 0 && distanceFrontTop < 20 && distanceFrontTop != 0 && timeElapsed > 10){
  TurnObstacle();
  }

  } else {
    //distanceLeft
    distanceRightDown = getDistance(17, 16);
    distanceRightTop = getDistance(19, 18);
    //distanceFront
    distanceFrontDown = getDistance(23, 22);
    distanceFrontTop = getDistance(21, 20);
    //distanceRight
    distanceLeftDown = getDistance(3, 2);
    distanceLeftTop = getDistance(15, 14);

    Serial.println(distanceFrontDown);

  }

}

int identifyGreen() {
  for (int i = 1; i < quantityOfSensors; i++) {
    if (((sensorValues[i] >= minGreenValue) && (sensorValues[i] <= maxGreenValue)) && ((sensorValues[i - 1] >= minGreenValue) && (sensorValues[i - 1] <= maxGreenValue))) {
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

bool identifyLine() {
  //for (int i = 8; i < quantityOfSensors; i++) {
  if ((sensorValues[0] < whiteLimit) && (sensorValues[1] < whiteLimit) && (sensorValues[2] < whiteLimit) && (sensorValues[3] < whiteLimit) && (sensorValues[4] < whiteLimit) && (sensorValues[5] < whiteLimit) && (sensorValues[6] < whiteLimit) && (sensorValues[7] < whiteLimit) && (sensorValues[8] < whiteLimit)) {
    return false;
    //}
  } else {
    return true;
  }
}

void identifyCurve() {
  for (int i = 2; i < quantityOfSensors; i++) {
    if ((sensorValues[i] > blackLimit) && (sensorValues[i - 1] >= blackLimit) && (sensorValues[i - 2] >= blackLimit)) {
      if (i <= 3) {
        curvePosition = -1;
      }
      else if (i > 4) {
        curvePosition = 1;
      }
      //else if (crossLine == true) {
      //curvePosition = 3;
      //}
    }
  }

}


float calculatePD(int error, int lastError) {
  int P = error;
  int D = error - lastError;

  lastError = error;
  updateLastError(lastError);

  return (P * kP) + (D * kD);
}

void updateLastError(int lastError) {
  lastGlobalError = lastError;
}

void updateError(int error) {
  globalError = error;
}

long readLine(int sensors[], int sensorQuantity) {
  long numerator = 0;
  long denominator = 0;

  for (int i = 0; i < sensorQuantity; i++) {
    numerator += (i * 1000 * (long)sensors[i]);
    denominator += sensors[i];
  }

  return map((numerator / denominator) , 950 , 6000 , -7 , 7);

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


void Front() {

  analogWrite(motorLF, 130);
  analogWrite(motorLB, 0);
  analogWrite(motorRF, 90);
  analogWrite(motorRB, 0);

}

void Back() {

  analogWrite(motorLF, 0);
  analogWrite(motorLB, 130);
  analogWrite(motorRF, 0);
  analogWrite(motorRB, 90);

}

void SpinL() {

  analogWrite(motorLF, 0);
  analogWrite(motorLB, 130);
  analogWrite(motorRF, 90);
  analogWrite(motorRB, 0);

}

void SpinR() {

  analogWrite(motorLF, 130);
  analogWrite(motorLB, 0);
  analogWrite(motorRF, 0);
  analogWrite(motorRB, 90);

}
void Stop() {

  analogWrite(motorLF, 0);
  analogWrite(motorLB, 0);
  analogWrite(motorRF, 0);
  analogWrite(motorRB, 0);

}

void TurnObstacle() {

  Back();
  Serial.println("1. Back");
  delay(600 * batteryMultiplier);

  SpinR();
  Serial.println("2. Right");
  delay(1100 * batteryMultiplier);

  Front();
  Serial.println("3. Front");
  delay(2500 * batteryMultiplier);

  SpinL();
  Serial.println("4. Left");
  delay(1100 * batteryMultiplier);

  Stop();
  Serial.println("Stopped");
  delay(100);

  Front();
  Serial.println("5. Front");
  delay(3000 * batteryMultiplier);

  SpinL();
  Serial.println("6. Left");
  delay(1100 * batteryMultiplier);

  Stop();
  Serial.println("Stopped");
  delay(100);

  Front();
  Serial.println("7. Front");
  delay(2000 * batteryMultiplier);

  SpinR();
  Serial.println("8. Right");
  delay(1200 * batteryMultiplier);

  Stop();
  Serial.println("Stopped");
  delay(100);


}

void getCross() {
  //desconsiderando nego
  if (sensorValues[0] > blackLimit && sensorValues[1] > blackLimit && sensorValues[2] > blackLimit && sensorValues[3] > blackLimit && sensorValues[4] > blackLimit && sensorValues[5] > blackLimit && sensorValues[6] > blackLimit && sensorValues[7] > blackLimit) {
    crossLine = 1;
  }
  else {
    crossLine = 0;
  }
}

void DownArm() {
  armServo.write(35);
  armHigh = false;
}

void HighArm() {
  armServo.write(150);
  armHigh = true;
}

void CloseHand() {
  handServo.write(49);
  handClosed = true;
}

void OpenHand() {
  handServo.write(110);
  handClosed = false;
}
