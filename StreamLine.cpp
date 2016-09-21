
//Linea de comunicacion serial
class StreamL {
private:
  void testTime();
  
public:
  //Pines del stream
  int rx, tx, sgn;
  
  int  bitRate;
  bool isBusy;
  long dtime;
  
  void init(int _rx, int _tx, int _sgn, int _bitRate, bool isMaster) {
  
    rx = _rx;
    tx = _tx;
    sgn = _sgn;
    bitRate = _bitRate;
    
    pinMode(rx, INPUT);
    pinMode(tx, OUTPUT);
    
    if (isMaster) pinMode(sgn, OUTPUT);
    else pinMode(sgn, INPUT);
    
    testTime();
  
  }
  
  bool transmitSample(char sample);
  bool receiveSample(unsigned char* sample);
  
  //bool transmitString(string S);

};

void StreamL::testTime() {

  long t1 = micros();
  
  for (int i=0;i<100;i++) {
  
    digitalWrite(sgn, HIGH);
  
  }
  
  long t2 = micros();
  
  dtime = 1000000.0/(float)bitRate - (float)(t2 - t1)/100.0f;
  
  digitalWrite(sgn, LOW);  
    
    //    enviar un bit:  pasar por 'for' + cambiar pin dig + esperar dtime = 1/bitRate * 10e6 us
    // dtime = 1/bitRate*10e6 - for - cam

}


bool StreamL::transmitSample(char sample) {

  isBusy = true;
  
  bool bits[8];
  char buffer = sample;
  //1º transformar sample al array bits[]
  for (int i=0;i<8;i++) {
  
      if (buffer%2) bits[i] = 1;
    else bits[i] = 0;
    buffer /= 2;
  
  }
  
  //2º transmitir los 8 bits
  digitalWrite(sgn, HIGH);
  for (int i=0;i<8;i++) {
  
    digitalWrite(tx, bits[i]);
    delayMicroseconds(dtime);
  
  }
  
  //Pasar un bit HIGH al pin sgn
  //delay(dtime);
  digitalWrite(sgn, LOW);
  
  isBusy = false;
  /*for (int i=0; i<8; i++)
    Serial.print(bits[i]);
  Serial.println();*/
  
  return true;
  

}

bool StreamL::receiveSample(unsigned char* sample) {

  bool bits[8];
  unsigned char buffer = 0;
  char buffer_int = 1;
  for (int i=0;i<8;i++) bits[i] = 0;
  
  //Esperar a que el pin sgn se ponga en 5V
  while(digitalRead(sgn) == 0);// {delayMicroseconds(5);}
  delayMicroseconds(15);
  for (int i=0;i<8;i++) {
  
    bits[i] = digitalRead(rx);
    
    delayMicroseconds(dtime);
  
  }
  
  //Delay que depende de la bitRate (fine-tune manual)
  delayMicroseconds(12);
  
  if (digitalRead(sgn) == 1) return false;
  
  for (int i=0;i<8;i++) {
  
    buffer += bits[i] * buffer_int;
    buffer_int *= 2;
  
  }
  (*sample) = buffer;

  /*for (int i=0;i<8;i++)
    Serial.print(bits[i]);
  Serial.println();*/
  
  return true;

}


/*
//------------------------------------------------------------------
//Emisor
StreamL ArduinoProMini;
int global_bitRate = 31250;
  
float num;
unsigned char sample;
float t, freq, ampl;
void setup() {
  
  //Iniciar comunicacion con Pro Mini
  ArduinoProMini.init(28, 26, 24, global_bitRate, true);
  //ArduinoProMini.testdtime(); --> no se puede
  
  //Iniciar variables
  sample = 0;
  t = 0.0;
  freq = 0.05;
  ampl = 1.0;
  ampl/=2.0;
  Serial.begin(9600);
  
}
int res = 50;
int maximo = 120;
void loop() {
  
  //Computar oscilador
  num = sin(t) * ampl +0.5;
  sample = (char)(num * (float)maximo);
  //sample = 50;
  //sample = map( num, -2.0, 2.0,  0, 255);
  
  //Transmitir sample
  ArduinoProMini.transmitSample(sample);
  String s = "";
  for (int i=0;i<(float)sample/(float)maximo*res;i++)
    s+='0';
  Serial.println(s);
  t+=freq;
  //Delay
  delay(100);
}
*/
//---------------------------------------------
//Receptor

StreamL ArduinoMega;
int pinOut = 13;
int global_bitRate = 31250;
unsigned char sample;

void setup() {
  Serial.begin(9600);
  ArduinoMega.init(26, 24, 28, global_bitRate, false);
  pinMode(pinOut, OUTPUT);
  digitalWrite(pinOut,LOW);
  delay(2000);
  digitalWrite(pinOut,LOW);

  Serial.println(ArduinoMega.dtime);
  Serial.println("dtime");

}
int res = 50;
int maximo = 120;
float cur, cur2;
unsigned char cbuf;
void loop() {

  if (ArduinoMega.receiveSample(&sample) == false)
    Serial.println("ERROR ERROR");
  String s = "";
  /*if ((float)sample/(float)maximo*res - (float)cbuf/(float)maximo*res > 5)
  {
   //Error
    char buffer = cbuf;
    Serial.println("Before");
    for (int i=0;i<8;i++) {
  
      if (buffer%2) Serial.print(1);
    else Serial.print(0);
    buffer /= 2;
  
  }
  Serial.println();
  buffer = sample;
    Serial.println("Actual sample");
    for (int i=0;i<8;i++) {
  
      if (buffer%2) Serial.print(1);
    else Serial.print(0);
    buffer /= 2;
  
  }
     

    for(;;);
   }*/
  for (int i=0;i<(float)sample/(float)maximo*res;i++)
  s+='0';
  Serial.println(s);
  
  //Serial.println((int)sample);
  analogWrite(pinOut, sample);
  cbuf = sample;

}
