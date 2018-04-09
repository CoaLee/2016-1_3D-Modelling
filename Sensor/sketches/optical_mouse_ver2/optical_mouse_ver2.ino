#define DebugMonitor false

enum MICE {M1, M2};

#define M1_PS2DATA   5
#define M1_PS2CLOCK  6
#define M2_PS2DATA   12
#define M2_PS2CLOCK  13

long M1_x, M1_y;
char M1_stat, M1_dx, M1_dy;

long M2_x, M2_y;
char M2_stat, M2_dx, M2_dy;

unsigned long millisStart;
byte PS2ReadByte = 0;

void PS2GoHi(int pin){
  pinMode(pin, INPUT);
  digitalWrite(pin, HIGH);
}

void PS2GoLo(int pin){
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}

void PS2Write(int miceNo, unsigned char data){
  unsigned char parity=1;
  int ps2data, ps2clock;

  switch (miceNo) {
    case M1 :
      ps2data = M1_PS2DATA;
      ps2clock = M1_PS2CLOCK;
      break;
    case M2 :
      ps2data = M2_PS2DATA;
      ps2clock = M2_PS2CLOCK;
      break;
    default :
      ps2data = M1_PS2DATA;
      ps2clock = M1_PS2CLOCK;
      break;    
  }
  
  PS2GoHi(ps2data);
  PS2GoHi(ps2clock);
  delayMicroseconds(300);
  PS2GoLo(ps2clock);
  delayMicroseconds(300);
  PS2GoLo(ps2data);
  delayMicroseconds(10);
  PS2GoHi(ps2clock);

  while(digitalRead(ps2clock)==HIGH);

  for(int i=0; i<8; i++){
    if(data&0x01) PS2GoHi(ps2data);
    else PS2GoLo(ps2data);
    while(digitalRead(ps2clock)==LOW);
    while(digitalRead(ps2clock)==HIGH);
    parity^=(data&0x01);
    data=data>>1;
  }

  if(parity) PS2GoHi(ps2data);
  else PS2GoLo(ps2data);

  while(digitalRead(ps2clock)==LOW);
  while(digitalRead(ps2clock)==HIGH);

  PS2GoHi(ps2data);
  delayMicroseconds(50);

  while(digitalRead(ps2clock)==HIGH);
  while((digitalRead(ps2clock)==LOW)||(digitalRead(ps2data)==LOW));

  PS2GoLo(ps2clock);
}

unsigned char PS2Read(int miceNo){
  unsigned char data=0, bit=1;
  int ps2data, ps2clock;

  switch (miceNo) {
    case M1 :
      ps2data = M1_PS2DATA;
      ps2clock = M1_PS2CLOCK;
      break;
    case M2 :
      ps2data = M2_PS2DATA;
      ps2clock = M2_PS2CLOCK;
      break;
    default :
      ps2data = M1_PS2DATA;
      ps2clock = M1_PS2CLOCK;
      break;    
  }
  
  PS2GoHi(ps2clock);
  PS2GoHi(ps2data);
  delayMicroseconds(50);
  while(digitalRead(ps2clock)==HIGH);

  delayMicroseconds(5);
  while(digitalRead(ps2clock)==LOW);

  for(int i=0; i<8; i++){
    while(digitalRead(ps2clock)==HIGH);
    if(digitalRead(ps2data)==HIGH) data|=bit;
    while(digitalRead(ps2clock)==LOW);
    bit=bit<<1;
  }

  while(digitalRead(ps2clock)==HIGH);
  while(digitalRead(ps2clock)==LOW);
  while(digitalRead(ps2clock)==HIGH);
  while(digitalRead(ps2clock)==LOW);

  PS2GoLo(ps2clock);

  return data;
}

void PS2MouseInit(int miceNo){
  PS2Write(miceNo, 0xFF);
  for(int i=0; i<3; i++) PS2Read(miceNo);
  PS2Write(miceNo, 0xF0);
  PS2Read(miceNo);
  delayMicroseconds(100);
}

void PS2MousePos(int miceNo, char &stat, char &x, char &y){
  PS2Write(miceNo, 0xEB);
  PS2Read(miceNo);
  stat=PS2Read(miceNo);
  x=PS2Read(miceNo);
  y=PS2Read(miceNo);
}


void setup(){
  PS2GoHi(M1_PS2CLOCK);
  PS2GoHi(M1_PS2DATA);
  PS2GoHi(M2_PS2CLOCK);
  PS2GoHi(M2_PS2DATA);

  Serial.begin(38400);
  while(!Serial); 
  if(DebugMonitor) Serial.println("Setup");

  PS2MouseInit(M1);
  if(DebugMonitor) Serial.println("Mouse1 Ready");
  PS2MouseInit(M2);
  if(DebugMonitor) Serial.println("Mouse2 Ready");
  
  millisStart=millis();
  M1_x = 0;
  M1_y = 0;
  M2_x = 0; 
  M2_y = 0;
}

void loop(){
  if(millis() < millisStart){ millisStart = millis(); }
  
  if(millis() - millisStart > 100){
    PS2MousePos(M1, M1_stat, M1_dx, M1_dy);
    PS2MousePos(M2, M2_stat, M2_dx, M2_dy);
    
    millisStart = millis();

    if(DebugMonitor) {
      Serial.print("\tM1 :\t"); Serial.print(M1_stat, BIN);
      Serial.print("\tdelta X="); Serial.print(M1_dx, DEC); Serial.print("\tdelta Y="); Serial.print(M1_dy, DEC);
      
      Serial.print("\tM2 :\t"); Serial.print(M2_stat, BIN);
      Serial.print("\tdelta X="); Serial.print(M2_dx, DEC); Serial.print("\tdelta Y="); Serial.print(M2_dy, DEC);
      
      Serial.print("\n");
    } else {
      if (Serial.available()) {
        if (Serial.read() == 's') {
          Serial.write(0xff) ;
          /* M1_y equals M2_x */
          sendChar(M1_dx);
          sendChar(M1_dy);
          sendChar(M2_dx);
          sendChar(M2_dy);
        }
      }
    }
  }
}

#define MASK_7bit B01111111
void sendChar(char c){
  if(c >= 0) Serial.write(3);
  else Serial.write(1);

  if(c >= 0) Serial.write(c);
  else {
    Serial.write(~(c-1) & MASK_7bit);
  }
}
