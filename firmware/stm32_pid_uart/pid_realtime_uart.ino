//////////////////// MOTOR 1 ////////////////////
#define ENCA1 PB3
#define ENCB1 PA15
#define PWM1  PA3
#define DIR1  PA2

//////////////////// MOTOR 2 ////////////////////
#define ENCA2 PB10
#define ENCB2 PB11
#define PWM2  PA1
#define DIR2  PA0

//////////////////// MOTOR 3 ////////////////////
#define ENCA3 PA4
#define ENCB3 PA5
#define PWM3  PA10
#define DIR3  PA9

//////////////////// MOTOR 4 ////////////////////
#define ENCA4 PB8
#define ENCB4 PB9
#define PWM4  PA8
#define DIR4  PB15

//////////////////////////////////////////////////
// UART2 to Raspberry Pi (PB6=TX, PB7=RX)
// Serial  = USB debug to laptop
// Serial2 = UART to Pi
//////////////////////////////////////////////////
HardwareSerial Serial2(PB7, PB6);

volatile long posi1=0, posi2=0, posi3=0, posi4=0;
volatile int  lastEncoded1=0, lastEncoded2=0;
volatile int  lastEncoded3=0, lastEncoded4=0;

float eintegral1=0, eintegral2=0, eintegral3=0, eintegral4=0;

float PPR          = 1980;
float WHEEL_RADIUS = 0.03367;  // update after calibration
float TRACK_WIDTH  = 0.74761;    // update after calibration

float target1=0, target2=0, target3=0, target4=0;

float kp1=4.0, ki1=0.5, kff1=2.4;
float kp2=4.0, ki2=0.5, kff2=2.4;
float kp3=4.0, ki3=0.5, kff3=2.4;
float kp4=4.0, ki4=0.5, kff4=2.4;
const float ICLAMP = 40.0;

// Odometry
float odom_x=0, odom_y=0, odom_theta=0;
long prevOdom1=0, prevOdom2=0, prevOdom3=0, prevOdom4=0;

// Watchdog: stop motors if Pi goes silent
unsigned long lastTargetMs = 0;
const unsigned long TIMEOUT_MS = 500;
bool rtActive = false;

// RPM state
static float _rpm1=0, _rpm2=0, _rpm3=0, _rpm4=0;
static long  _prevP1=0, _prevP2=0, _prevP3=0, _prevP4=0;

void setup() {
  Serial.begin(115200);    // USB debug -- connect to laptop to monitor
  Serial2.begin(115200);   // UART to Pi on PB6/PB7

  pinMode(ENCA1,INPUT_PULLUP); pinMode(ENCB1,INPUT_PULLUP);
  pinMode(ENCA2,INPUT_PULLUP); pinMode(ENCB2,INPUT_PULLUP);
  pinMode(ENCA3,INPUT_PULLUP); pinMode(ENCB3,INPUT_PULLUP);
  pinMode(ENCA4,INPUT_PULLUP); pinMode(ENCB4,INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCA1),readEncoder1,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB1),readEncoder1,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCA2),readEncoder2,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB2),readEncoder2,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCA3),readEncoder3,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB3),readEncoder3,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCA4),readEncoder4,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB4),readEncoder4,CHANGE);

  pinMode(PWM1,OUTPUT); pinMode(DIR1,OUTPUT);
  pinMode(PWM2,OUTPUT); pinMode(DIR2,OUTPUT);
  pinMode(PWM3,OUTPUT); pinMode(DIR3,OUTPUT);
  pinMode(PWM4,OUTPUT); pinMode(DIR4,OUTPUT);

  Serial.println("READY - debug on USB, Pi on Serial2 PB6/PB7");
  Serial2.println("READY");
}

void loop() {
  // Read commands from Pi on Serial2
  static char buf[64];
  static uint8_t blen = 0;

  while (Serial2.available()) {
    char c = Serial2.read();
    if (c == '\n') {
      buf[blen] = '\0';
      if (strncmp(buf, "TARGET,", 7) == 0) {
        parseTarget(buf);
      } else if (blen == 1 && buf[0] == 'r') {
        resetOdom();
      }
      blen = 0;
    } else if (c != '\r' && blen < 63) {
      buf[blen++] = c;
    }
  }

  static unsigned long lastTime = 0;
  if (millis() - lastTime < 20) return;
  lastTime = millis();
  float dt = 0.02;

  // Watchdog: Pi went silent -> stop motors
  if (rtActive && (millis() - lastTargetMs) > TIMEOUT_MS) {
    target1=target2=target3=target4=0;
    eintegral1=eintegral2=eintegral3=eintegral4=0;
    rtActive = false;
    Serial.println("WATCHDOG: Pi silent, motors stopped");
  }

  // Odometry update
  long s1,s2,s3,s4;
  noInterrupts(); s1=posi1;s2=posi2;s3=posi3;s4=posi4; interrupts();
  float tpm = PPR / (2.0*PI*WHEEL_RADIUS);
  // Physical test confirmed: M2/M3 are the RIGHT side, M1/M4 are the LEFT side
  // despite their pin names -- swap dL/dR accordingly
  float dL = ((s1-prevOdom1 + s4-prevOdom4)/2.0) / tpm;  // M1,M4 = physical LEFT
  float dR = ((s2-prevOdom2 + s3-prevOdom3)/2.0) / tpm;  // M2,M3 = physical RIGHT
  prevOdom1=s1; prevOdom2=s2; prevOdom3=s3; prevOdom4=s4;
  float dc   = (dL+dR)/2.0;
  // Negated: physically turning right increases theta in our setup
  float dth  = -((dR-dL)/TRACK_WIDTH);
  float tmid = odom_theta + dth/2.0;
  odom_x     += dc*cos(tmid);
  odom_y     += dc*sin(tmid);
  odom_theta += dth;
  odom_theta  = atan2(sin(odom_theta),cos(odom_theta));

  // PID
  runPID(dt);

  // Send odometry to Pi on Serial2
  // Format: ODOM,x,y,theta,rpm1,rpm2,rpm3,rpm4
  Serial2.print(F("ODOM,"));
  Serial2.print(odom_x,4);    Serial2.print(',');
  Serial2.print(odom_y,4);    Serial2.print(',');
  Serial2.print(odom_theta,5);Serial2.print(',');
  Serial2.print(_rpm1,2);     Serial2.print(',');
  Serial2.print(_rpm2,2);     Serial2.print(',');
  Serial2.print(_rpm3,2);     Serial2.print(',');
  Serial2.println(_rpm4,2);

  // Mirror odometry to USB debug port so you can monitor on laptop too
  Serial.print(F("ODOM,"));
  Serial.print(odom_x,4);    Serial.print(',');
  Serial.print(odom_y,4);    Serial.print(',');
  Serial.print(odom_theta*180.0/PI,2); Serial.print(F("deg | RPM "));
  Serial.print(_rpm1,1); Serial.print(' ');
  Serial.print(_rpm2,1); Serial.print(' ');
  Serial.print(_rpm3,1); Serial.print(' ');
  Serial.println(_rpm4,1);
}

void parseTarget(char* line) {
  char tmp[64]; strncpy(tmp,line,63); tmp[63]='\0';
  strtok(tmp,",");
  char* t1=strtok(NULL,","); if(!t1) return;
  char* t2=strtok(NULL,","); if(!t2) return;
  char* t3=strtok(NULL,","); if(!t3) return;
  char* t4=strtok(NULL,","); if(!t4) return;
  target1=atof(t1); target2=atof(t2);
  target3=atof(t3); target4=atof(t4);
  lastTargetMs=millis();
  rtActive=true;
}

void resetOdom() {
  odom_x=odom_y=odom_theta=0;
  noInterrupts();
  prevOdom1=posi1; prevOdom2=posi2;
  prevOdom3=posi3; prevOdom4=posi4;
  interrupts();
  Serial2.println(F("ODOM RESET"));
  Serial.println(F("Odometry reset"));
}

void runPID(float dt) {
  auto calcRPM = [&](volatile long& posi, long& prev, float& filt) {
    long snap; noInterrupts(); snap=posi; interrupts();
    float raw = ((snap-prev)/PPR)*(60.0/dt);
    prev = snap;
    filt = 0.8*filt + 0.2*raw;
  };
  calcRPM(posi1,_prevP1,_rpm1);
  calcRPM(posi2,_prevP2,_rpm2);
  calcRPM(posi3,_prevP3,_rpm3);
  calcRPM(posi4,_prevP4,_rpm4);

  auto pidStep = [&](float tgt, float rpm, float& eint,
                     float kp, float ki, float kff,
                     int pwm, int dir) {
    float u = kff*tgt + kp*(tgt-rpm) + ki*eint;
    float pwr = fabs(u); if(pwr>255) pwr=255;
    if(fabs(tgt-rpm)>15) pwr=max(pwr,150.0f);
    setMotor(pwm, dir, (u>=0)?1:-1, pwr);
    if(pwr<255){ eint+=(tgt-rpm)*dt; eint=constrain(eint,-ICLAMP,ICLAMP); }
  };

  pidStep(target1,_rpm1,eintegral1,kp1,ki1,kff1,PWM1,DIR1);
  pidStep(target2,_rpm2,eintegral2,kp2,ki2,kff2,PWM2,DIR2);
  pidStep(target3,_rpm3,eintegral3,kp3,ki3,kff3,PWM3,DIR3);
  pidStep(target4,_rpm4,eintegral4,kp4,ki4,kff4,PWM4,DIR4);
}

void setMotor(int pwmPin, int dirPin, int dir, float pwmVal) {
  analogWrite(pwmPin, (int)pwmVal);
  digitalWrite(dirPin, dir==1 ? HIGH : LOW);
}

#define ENC_ISR(N,A,B,pos,last) \
void readEncoder##N() { \
  int e=(digitalRead(A)<<1)|digitalRead(B); \
  int s=(last<<2)|e; \
  if(s==0b1101||s==0b0100||s==0b0010||s==0b1011) pos++; \
  if(s==0b1110||s==0b0111||s==0b0001||s==0b1000) pos--; \
  last=e; \
}

ENC_ISR(1,ENCA1,ENCB1,posi1,lastEncoded1)
ENC_ISR(2,ENCA2,ENCB2,posi2,lastEncoded2)
ENC_ISR(3,ENCA3,ENCB3,posi3,lastEncoded3)
ENC_ISR(4,ENCA4,ENCB4,posi4,lastEncoded4)
