int STBY = 16;
int AI1 = 17;
int AI2 = 18;
int PWMA = 4;
int POW = 13;

char command;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(STBY,OUTPUT);
  pinMode(AI1,OUTPUT);
  pinMode(AI2,OUTPUT);
  pinMode(PWMA,OUTPUT);
  pinMode(POW,OUTPUT);

  digitalWrite(AI1,LOW);
  digitalWrite(AI2,LOW);
  digitalWrite(POW,HIGH);
  digitalWrite(STBY,HIGH);

}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available()>0)
  {
    command = Serial.read();
  }
  
  if(command=='s')
  {
    Serial.println("Stopping motor...");
    digitalWrite(AI1,LOW);
    digitalWrite(AI2,LOW);
    command='0';
  }

  if(command=='f')
  {
    digitalWrite(AI1,LOW);
    digitalWrite(AI2,LOW);
    delay(900);
    Serial.println("Forward");
    digitalWrite(AI1,HIGH);
    digitalWrite(AI2,LOW);
    analogWrite(PWMA,126);
    command='0';
  }

  if(command=='b')
  {
    digitalWrite(AI1,LOW);
    digitalWrite(AI2,LOW);
    delay(900);
    Serial.println("Backward");
    digitalWrite(AI1,LOW);
    digitalWrite(AI2,HIGH);
    analogWrite(PWMA,255);
    command='0';
  }

}
