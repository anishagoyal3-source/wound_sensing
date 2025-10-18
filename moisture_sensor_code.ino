void setup() {
  // put your setup code here, to run once:
pinMode(A0,INPUT); 
Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
int val = analogRead(A0); 
int mval = map(val, 1023, 550, 0, 100);
mval = constrain(mval, 0, 100);
Serial.println(mval);
}
