// Motor Pins
int in1 = 2;
int in2 = 3;
int in3 = 4;
int in4 = 7;

int en1 = 5;
int en2 = 6;
int stdby = 8;



void setup() {
  Serial.begin(9600);

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  pinMode(en1, OUTPUT);
  pinMode(en2, OUTPUT);

  pinMode(stdby, HIGH);

  analogWrite(en1, 150);
  analogWrite(en2, 150);
  digitalWrite(stdby, HIGH);

}


void moveForward() {
  digitalWrite(in1, LOW); 
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW); 
  digitalWrite(in4, HIGH);
  delay(400);
}

void loop() {
 moveForward();

}
