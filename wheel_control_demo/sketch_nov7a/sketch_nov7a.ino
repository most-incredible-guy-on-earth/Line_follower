int vcc = 13,stby=14,pwma=16,pwmb=17,ai1=21,ai2=22,bi1=19,bi2=18;

void setup() {
  // put your setup code here, to run once:
  pinMode(vcc,OUTPUT);
  pinMode(stby,OUTPUT);
  pinMode(pwma,OUTPUT);
  pinMode(pwmb,OUTPUT);
  pinMode(ai1,OUTPUT);
  pinMode(ai2,OUTPUT);
  pinMode(bi1,OUTPUT);
  pinMode(bi2,OUTPUT);

  digitalWrite(vcc,HIGH);
  digitalWrite(stby,HIGH);
  analogWrite(pwma,0);
  analogWrite(pwmb,0);
  digitalWrite(ai1,HIGH);
  digitalWrite(ai2,LOW);
  digitalWrite(bi1,HIGH);
  digitalWrite(bi2,LOW);

}

void loop() {
  // put your main code here, to run repeatedly:

  for(int i=60;i<100;i++)
  {
    analogWrite(pwma,i);
    analogWrite(pwmb,i);
    delay(50);
  }

}
