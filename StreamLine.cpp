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
  bool receiveSample(char* sample);
  
  //bool transmitString(string S);

};

void StreamL::testTime() {

  long t1 = micros();
  
  for (int i=0;i<100;i++) {
  
    digitalWrite(sgn, HIGH);
  
  }
  
  long t2 = micros();
  
  dtime = 1.0f/(float)bitRate - (float)(t2 - t1)/100.0f;
  
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
  for (int i=7;i>=0;i++) {
  
    digitalWrite(tx, bits[i]);
    delayMicroseconds(dtime);
  
  }
  
  //Pasar un bit HIGH al pin sgn
  //delay(dtime);
  digitalWrite(sgn, LOW);
  
  isBusy = false;
  
  return true;
  

}

bool StreamL::receiveSample(char* sample) {

  bool bits[8];
  char buffer = 0;
  char buffer_int = 1;
  for (int i=0;i<8;i++) bits[i] = 0;
  
  //Esperar a que el pin sgn se ponga en 5V
  while(digitalRead(sgn) == 0) {delayMicroseconds(5);}
  delayMicroseconds(10);
  
  for (int i=0;i<8;i++) {
  
    bits[i] = digitalRead(rx);
    
    delayMicroseconds(dtime);
  
  }
  
  delayMicroseconds(15);
  
  if (digitalRead(sgn) == 1) return false;
  
  for (int i=0;i<8;i++) {
  
    buffer += bits[i] * buffer_int;
    buffer_int *= 2;
  
  }
  
  (*sample) = buffer;
  
  return true;

}

/*bool StreamL::transmitString(string S) {

  

}*/

/*
//------------------------------------------------------------------
//Variables globales
Stream ArduinoProMini;
int global_bitRate = 3000;
  
  
char sample;
float t, freq, ampl;


void setup() {
  
  //Iniciar comunicacion con Pro Mini
  ArduinoProMini.init(15, 16, 17, global_bitRate, true);
  //ArduinoProMini.testdtime(); --> no se puede
  
  //Iniciar variables
  sample = 0;
  t = 0.0;
  freq = 0.1;
  ampl = 1.0;
  
}


void loop() {
  
  //Computar oscilador
  float num = sin(t*freq) * ampl;
  sample = map( num, -2.0, 2.0,  0, 255);
  
  //Transmitir sample
  ArduinoProMini.transmitSample(sample);
  
  //Delay
  delay(20);

}
*/
//---------------------------------------------
//Codigo en el Pro Mini

StreamL ArduinoMega;
int pinOut = 13;
int global_bitRate = 3000;

void setup() {
  
  ArduinoMega.init(1, 2, 3, global_bitRate, false);
  pinMode(pinOut, OUTPUT);

}

void loop() {

  char sample;
  ArduinoMega.receiveSample(&sample);
  
  analogWrite(pinOut, sample);

}
