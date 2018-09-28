#define GRANICA 850

#define L_PWM 5
#define L_DIR 4
#define R_PWM 6
#define R_DIR 9
#define PWM_MAX 165

#define R_LINE_SENSOR A0
#define L_LINE_SENSOR A1
#define BUZZER 10
#define LED 13
//A0 = 14, A5 = 18

int waga[] = { -30, -13, -5, 8, 18, 35};
//int waga[] = { -35, -18, -8, 8, 18, 35};
int sum = 0;
int gain = 1;
int error = 0;
int error_prev = 0;
int cnt = 0;
int odczyt = 0;
int border = 400;
int s_low = 500;
int s_high = 500;
int v = 35;

void pid()
{
  if (cnt != 0)
  {
  sum /=cnt;
    error = sum * gain;
    error_prev = error;
  }
  else
  {
    error = error_prev;
  }
    int left;
  int right;
  if(error>0)
  {
    left = v + (error/2);
    right = v - error; 
  }
  else
  {
    left = v + error;
    right = v - (error/2); 
  }
  rightMotor(right);
  leftMotor(left);
  Serial.print("  L=");
  Serial.print(left);
  Serial.print("  P=");
  Serial.print(right);
  Serial.print("  ");
 
}

void readSensors()
{
  sum = 0;
  cnt = 6;
  //  s_low = 500;
  // s_high = 500;
  for (int i = 14; i <= 19; i++)
  {
    odczyt = analogRead(i);
    //   if(odczyt < s_low) s_low = odczyt;
    //  if(odczyt > s_high) s_high = odczyt;
    if (odczyt < border)
    {
      sum += waga[i - 14];
      cnt--;
    }
    Serial.print(i - 14);
    Serial.print("=");
    Serial.print(odczyt);
    Serial.print(",  ");

  }
  border = (s_low + s_high) / 2;
  Serial.println("  sum=" + (String)sum + "  border=" + (String)border + "  cnt=" + (String)cnt);
}

void setup() {
  //Konfiguracja pinow od mostka H
  pinMode(L_DIR, OUTPUT);
  pinMode(R_DIR, OUTPUT);
  pinMode(L_PWM, OUTPUT);
  pinMode(R_PWM, OUTPUT);

  //Konfiguracja pozostalych elementow
  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, 0); //Wylaczenie buzzera
  pinMode(LED, OUTPUT);
  digitalWrite(LED, 0); //Wylaczenie diody

  Serial.begin(9600);
}

void loop() {
  readSensors();
  pid();
  // have v, error

  /*
    if (leftSensor() == 1 && rightSensor() == 1) { //Jesli oba czujniki widza linii
    leftMotor(40); //Jazda prosto
    rightMotor(40);
    } else if (leftSensor() == 0) { //Jesli lewy czujnik nie widzi linii
    leftMotor(40); //Jazda po łuku w prawo
    rightMotor(10);
    } else if (rightSensor() == 0) { //Jesli prawy czujnik nie widzi linii
    leftMotor(10); //Jazda po łuku w lewo
    rightMotor(40);
    }
  */
}

boolean leftSensor() {
  if (analogRead(L_LINE_SENSOR) > GRANICA) { //Jesli czujnik widzi linie, to
    return 1; //Zwroc 1
  } else { //Jesli czujnik nie jest nad linią, to
    return 0; //Zwroc 0
  }
}

boolean rightSensor() {
  if (analogRead(R_LINE_SENSOR) > GRANICA) { //Jesli czujnik widzi linie, to
    return 1; //Zwroc 1
  } else { //Jesli czujnik nie jest nad linią, to
    return 0; //Zwroc 0
  }
}

void leftMotor(int V) {
  if (V > 0) { //Jesli predkosc jest wieksza od 0 (dodatnia)
    V = map(V, 0, 100, 0, PWM_MAX);
    digitalWrite(L_DIR, 0); //Kierunek: do przodu
    analogWrite(L_PWM, V); //Ustawienie predkosci
  } else {
    V = abs(V); //Funkcja abs() zwroci wartosc V  bez znaku
    V = map(V, 0, 100, 0, PWM_MAX);
    digitalWrite(L_DIR, 1); //Kierunek: do tyłu
    analogWrite(L_PWM, V); //Ustawienie predkosci
  }
}

void rightMotor(int V) {
  if (V > 0) { //Jesli predkosc jest wieksza od 0 (dodatnia)
    V = map(V, 0, 100, 0, PWM_MAX);
    digitalWrite(R_DIR, 0); //Kierunek: do przodu
    analogWrite(R_PWM, V); //Ustawienie predkosci
  } else {
    V = abs(V); //Funkcja abs() zwroci wartosc V  bez znaku
    V = map(V, 0, 100, 0, PWM_MAX);
    digitalWrite(R_DIR, 1); //Kierunek: do tyłu
    analogWrite(R_PWM, V); //Ustawienie predkosci
  }
}

void stopMotors() {
  analogWrite(L_PWM, 0); //Wylaczenie silnika lewego
  analogWrite(R_PWM, 0); //Wylaczenie silnika prawego
}
