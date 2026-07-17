//////////////////// MOTOR 1 ////////////////////
#define ENCA1 PA4
#define ENCB1 PA5
#define PWM1  PA6
#define DIR1  PB0

//////////////////// MOTOR 2 ////////////////////
#define ENCA2 PB10
#define ENCB2 PA1
#define PWM2  PA9
#define DIR2  PA7

//////////////////// MOTOR 3 ////////////////////
#define ENCA3 PB7
#define ENCB3 PB6
#define PWM3  PA8
#define DIR3  PB12

//////////////////// MOTOR 4 ////////////////////
#define ENCA4 PB8
#define ENCB4 PB9
#define PWM4  PA0
#define DIR4  PB13

//////////////////////////////////////////////////
// UART2
//////////////////////////////////////////////////

HardwareSerial Serial2(PA3, PA2);

//////////////////////////////////////////////////
// CONTROL MODE
//////////////////////////////////////////////////

int controlMode = 1;

//////////////////////////////////////////////////
// ENCODER POSITIONS
//////////////////////////////////////////////////

volatile long posi1 = 0;
volatile long posi2 = 0;
volatile long posi3 = 0;
volatile long posi4 = 0;

volatile int lastEncoded1 = 0;
volatile int lastEncoded2 = 0;
volatile int lastEncoded3 = 0;
volatile int lastEncoded4 = 0;

//////////////////////////////////////////////////
// TARGETS
//////////////////////////////////////////////////

float target1 = 0;
float target2 = 0;
float target3 = 0;
float target4 = 0;

float targetPos3 = 0;
float targetPos4 = 0;

//////////////////////////////////////////////////
// PID VARIABLES
//////////////////////////////////////////////////

float eintegral1 = 0;
float eintegral2 = 0;
float eintegral3 = 0;
float eintegral4 = 0;

//////////////////////////////////////////////////
// STEERING PID VARIABLES
//////////////////////////////////////////////////

float steerIntegral3 = 0;
float steerIntegral4 = 0;

float prevError3 = 0;
float prevError4 = 0;

//////////////////////////////////////////////////
// PARAMETERS
//////////////////////////////////////////////////

float PPR = 1980;

float kp = 4.0;
float ki = 0.5;
float kff = 2.4;

//////////////////////////////////////////////////
// STEERING PID PARAMETERS
//////////////////////////////////////////////////

float steerKp = 0.08;
float steerKi = 0.0005;
float steerKd = 0.002;

//////////////////////////////////////////////////
// STEERING LIMITS
//////////////////////////////////////////////////

float MAX_STEER_COUNTS = 3000;

//////////////////////////////////////////////////
// SETUP
//////////////////////////////////////////////////

void setup()
{
    Serial.begin(115200);

    Serial2.begin(115200);

    //////////////////////////////////////////////////
    // ENCODERS
    //////////////////////////////////////////////////

    pinMode(ENCA1, INPUT_PULLUP);
    pinMode(ENCB1, INPUT_PULLUP);

    pinMode(ENCA2, INPUT_PULLUP);
    pinMode(ENCB2, INPUT_PULLUP);

    pinMode(ENCA3, INPUT_PULLUP);
    pinMode(ENCB3, INPUT_PULLUP);

    pinMode(ENCA4, INPUT_PULLUP);
    pinMode(ENCB4, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENCA1), readEncoder1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCB1), readEncoder1, CHANGE);

    attachInterrupt(digitalPinToInterrupt(ENCA2), readEncoder2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCB2), readEncoder2, CHANGE);

    attachInterrupt(digitalPinToInterrupt(ENCA3), readEncoder3, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCB3), readEncoder3, CHANGE);

    attachInterrupt(digitalPinToInterrupt(ENCA4), readEncoder4, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCB4), readEncoder4, CHANGE);

    //////////////////////////////////////////////////
    // MOTOR OUTPUTS
    //////////////////////////////////////////////////

    pinMode(PWM1, OUTPUT);
    pinMode(DIR1, OUTPUT);

    pinMode(PWM2, OUTPUT);
    pinMode(DIR2, OUTPUT);

    pinMode(PWM3, OUTPUT);
    pinMode(DIR3, OUTPUT);

    pinMode(PWM4, OUTPUT);
    pinMode(DIR4, OUTPUT);
}

//////////////////////////////////////////////////
// LOOP
//////////////////////////////////////////////////

void loop()
{
    receiveTargets();

    static unsigned long lastTime = 0;

    if (millis() - lastTime < 20)
        return;

    lastTime = millis();

    float deltaT = 0.02;

    //////////////////////////////////////////////////
    // MOTOR 1
    //////////////////////////////////////////////////

    long pos1;

    noInterrupts();
    pos1 = posi1;
    interrupts();

    static long prevPos1 = 0;

    long deltaPos1 = pos1 - prevPos1;

    prevPos1 = pos1;

    float rpm1 =
        (deltaPos1 / PPR) * (60.0 / deltaT);

    static float rpmFilt1 = 0;

    rpmFilt1 =
        0.8 * rpmFilt1 + 0.2 * rpm1;

    float e1 = target1 - rpmFilt1;

    float u1 =
        kff * target1 +
        kp * e1 +
        ki * eintegral1;

    float pwr1 = fabs(u1);

    if (pwr1 > 255)
        pwr1 = 255;

    int dir1 = (u1 >= 0) ? 1 : -1;

    if(target1 == 0)
    {
        analogWrite(PWM1, 0);
        eintegral1 = 0;
    }
    else
    {
        setMotor(PWM1, DIR1, dir1, pwr1);

        if(pwr1 < 255)
            eintegral1 += e1 * deltaT;
    }

    //////////////////////////////////////////////////
    // MOTOR 2
    //////////////////////////////////////////////////

    long pos2;

    noInterrupts();
    pos2 = posi2;
    interrupts();

    static long prevPos2 = 0;

    long deltaPos2 = pos2 - prevPos2;

    prevPos2 = pos2;

    float rpm2 =
        (deltaPos2 / PPR) * (60.0 / deltaT);

    static float rpmFilt2 = 0;

    rpmFilt2 =
        0.8 * rpmFilt2 + 0.2 * rpm2;

    float e2 = target2 - rpmFilt2;

    float u2 =
        kff * target2 +
        kp * e2 +
        ki * eintegral2;

    float pwr2 = fabs(u2);

    if (pwr2 > 255)
        pwr2 = 255;

    int dir2 = (u2 >= 0) ? 1 : -1;

    if(target2 == 0)
    {
        analogWrite(PWM2, 0);
        eintegral2 = 0;
    }
    else
    {
        setMotor(PWM2, DIR2, dir2, pwr2);

        if(pwr2 < 255)
            eintegral2 += e2 * deltaT;
    }

    //////////////////////////////////////////////////
    // MOTOR 3
    //////////////////////////////////////////////////

    long pos3;

    noInterrupts();
    pos3 = posi3;
    interrupts();

    static long prevPos3 = 0;

    long deltaPos3 = pos3 - prevPos3;

    prevPos3 = pos3;

    float rpm3 =
        (deltaPos3 / PPR) * (60.0 / deltaT);

    static float rpmFilt3 = 0;

    rpmFilt3 =
        0.8 * rpmFilt3 + 0.2 * rpm3;

    //////////////////////////////////////////////////
    // MODE 1
    // VELOCITY PID
    //////////////////////////////////////////////////

    if(controlMode == 1)
    {
        float e3 = target3 - rpmFilt3;

        float u3 =
            kff * target3 +
            kp * e3 +
            ki * eintegral3;

        float pwr3 = fabs(u3);

        if (pwr3 > 255)
            pwr3 = 255;

        int dir3 = (u3 >= 0) ? 1 : -1;

        if(target3 == 0)
        {
            analogWrite(PWM3, 0);
            eintegral3 = 0;
        }
        else
        {
            setMotor(PWM3, DIR3, dir3, pwr3);

            if(pwr3 < 255)
                eintegral3 += e3 * deltaT;
        }
    }

    //////////////////////////////////////////////////
    // MODE 2
    // POSITION PID
    //////////////////////////////////////////////////

    else if(controlMode == 2)
    {
        float error3 =
            targetPos3 - pos3;

        steerIntegral3 +=
            error3 * deltaT;

        float derivative3 =
            (error3 - prevError3) / deltaT;

        float u3 =
            steerKp * error3 +
            steerKi * steerIntegral3 +
            steerKd * derivative3;

        float pwr3 = fabs(u3);

        if(pwr3 > 255)
            pwr3 = 255;

        int dir3 = (u3 >= 0) ? 1 : -1;

        if(fabs(error3) < 10)
        {
            analogWrite(PWM3, 0);
        }
        else
        {
            setMotor(PWM3, DIR3, dir3, pwr3);
        }

        prevError3 = error3;
    }

    //////////////////////////////////////////////////
    // MOTOR 4
    //////////////////////////////////////////////////

    long pos4;

    noInterrupts();
    pos4 = posi4;
    interrupts();

    static long prevPos4 = 0;

    long deltaPos4 = pos4 - prevPos4;

    prevPos4 = pos4;

    float rpm4 =
        (deltaPos4 / PPR) * (60.0 / deltaT);

    static float rpmFilt4 = 0;

    rpmFilt4 =
        0.8 * rpmFilt4 + 0.2 * rpm4;

    //////////////////////////////////////////////////
    // MODE 1
    //////////////////////////////////////////////////

    if(controlMode == 1)
    {
        float e4 = target4 - rpmFilt4;

        float u4 =
            kff * target4 +
            kp * e4 +
            ki * eintegral4;

        float pwr4 = fabs(u4);

        if (pwr4 > 255)
            pwr4 = 255;

        int dir4 = (u4 >= 0) ? 1 : -1;

        if(target4 == 0)
        {
            analogWrite(PWM4, 0);
            eintegral4 = 0;
        }
        else
        {
            setMotor(PWM4, DIR4, dir4, pwr4);

            if(pwr4 < 255)
                eintegral4 += e4 * deltaT;
        }
    }

    //////////////////////////////////////////////////
    // MODE 2
    //////////////////////////////////////////////////

    else if(controlMode == 2)
    {
        float error4 =
            targetPos4 - pos4;

        steerIntegral4 +=
            error4 * deltaT;

        float derivative4 =
            (error4 - prevError4) / deltaT;

        float u4 =
            steerKp * error4 +
            steerKi * steerIntegral4 +
            steerKd * derivative4;

        float pwr4 = fabs(u4);

        if(pwr4 > 255)
            pwr4 = 255;

        int dir4 = (u4 >= 0) ? 1 : -1;

        if(fabs(error4) < 10)
        {
            analogWrite(PWM4, 0);
        }
        else
        {
            setMotor(PWM4, DIR4, dir4, pwr4);
        }

        prevError4 = error4;
    }

    //////////////////////////////////////////////////
    // VEHICLE SPEED
    //////////////////////////////////////////////////

    float vehicleSpeed =
        (rpmFilt1 + rpmFilt2) / 2.0;

    //////////////////////////////////////////////////
    // SEND FEEDBACK
    //////////////////////////////////////////////////

    Serial2.print(rpmFilt1);
    Serial2.print(",");

    Serial2.print(rpmFilt2);
    Serial2.print(",");

    Serial2.print(rpmFilt3);
    Serial2.print(",");

    Serial2.print(rpmFilt4);
    Serial2.print(",");

    Serial2.print(pos1);
    Serial2.print(",");

    Serial2.print(pos2);
    Serial2.print(",");

    Serial2.print(pos3);
    Serial2.print(",");

    Serial2.print(pos4);
    Serial2.print(",");

    Serial2.print(controlMode);
    Serial2.print(",");

    Serial2.println(vehicleSpeed);

    //////////////////////////////////////////////////
    // DEBUG
    //////////////////////////////////////////////////

    Serial.print("MODE:");
    Serial.print(controlMode);

    Serial.print(" RPM1:");
    Serial.print(rpmFilt1);

    Serial.print(" RPM2:");
    Serial.print(rpmFilt2);

    Serial.print(" POS3:");
    Serial.print(pos3);

    Serial.print(" POS4:");
    Serial.println(pos4);
}

//////////////////////////////////////////////////
// RECEIVE UART DATA
//////////////////////////////////////////////////

void receiveTargets()
{
    static String data = "";

    while (Serial2.available())
    {
        char c = Serial2.read();

        if (c == '\n')
        {
            //////////////////////////////////////////////////
            // MODE 1
            // 1,m1,m2,m3,m4
            //////////////////////////////////////////////////

            if (data.startsWith("1,"))
            {
                controlMode = 1;

                int c1 = data.indexOf(',');
                int c2 = data.indexOf(',', c1 + 1);
                int c3 = data.indexOf(',', c2 + 1);
                int c4 = data.indexOf(',', c3 + 1);

                if (c4 > 0)
                {
                    target1 =
                        data.substring(c1 + 1, c2).toFloat();

                    target2 =
                        data.substring(c2 + 1, c3).toFloat();

                    target3 =
                        data.substring(c3 + 1, c4).toFloat();

                    target4 =
                        data.substring(c4 + 1).toFloat();
                }
            }

            //////////////////////////////////////////////////
            // MODE 2
            // 2,steer,speed
            //////////////////////////////////////////////////

            else if (data.startsWith("2,"))
            {
                controlMode = 2;

                int c1 = data.indexOf(',');
                int c2 = data.indexOf(',', c1 + 1);

                if (c2 > 0)
                {
                    float steer =
                        data.substring(c1 + 1, c2).toFloat();

                    float speed =
                        data.substring(c2 + 1).toFloat();

                    //////////////////////////////////////////////////
                    // REAR MOTORS
                    //////////////////////////////////////////////////

                    target1 = speed;
                    target2 = speed;

                    //////////////////////////////////////////////////
                    // STEERING POSITION
                    //////////////////////////////////////////////////

                    targetPos3 =
                        steer * MAX_STEER_COUNTS;

                    targetPos4 =
                        steer * MAX_STEER_COUNTS;
                }
            }

            data = "";
        }

        else
        {
            data += c;
        }
    }
}

//////////////////////////////////////////////////
// MOTOR CONTROL
//////////////////////////////////////////////////

void setMotor(int pwmPin,
              int dirPin,
              int dir,
              float pwmVal)
{
    analogWrite(pwmPin, pwmVal);

    if(dir == 1)
        digitalWrite(dirPin, HIGH);
    else
        digitalWrite(dirPin, LOW);
}

//////////////////////////////////////////////////
// ENCODER ISR 1
//////////////////////////////////////////////////

void readEncoder1()
{
    int MSB = digitalRead(ENCA1);
    int LSB = digitalRead(ENCB1);

    int encoded = (MSB << 1) | LSB;
    int sum = (lastEncoded1 << 2) | encoded;

    if(sum == 0b1101 || sum == 0b0100 ||
       sum == 0b0010 || sum == 0b1011)
        posi1++;

    if(sum == 0b1110 || sum == 0b0111 ||
       sum == 0b0001 || sum == 0b1000)
        posi1--;

    lastEncoded1 = encoded;
}

//////////////////////////////////////////////////
// ENCODER ISR 2
//////////////////////////////////////////////////

void readEncoder2()
{
    int MSB = digitalRead(ENCA2);
    int LSB = digitalRead(ENCB2);

    int encoded = (MSB << 1) | LSB;
    int sum = (lastEncoded2 << 2) | encoded;

    if(sum == 0b1101 || sum == 0b0100 ||
       sum == 0b0010 || sum == 0b1011)
        posi2++;

    if(sum == 0b1110 || sum == 0b0111 ||
       sum == 0b0001 || sum == 0b1000)
        posi2--;

    lastEncoded2 = encoded;
}

//////////////////////////////////////////////////
// ENCODER ISR 3
//////////////////////////////////////////////////

void readEncoder3()
{
    int MSB = digitalRead(ENCA3);
    int LSB = digitalRead(ENCB3);

    int encoded = (MSB << 1) | LSB;
    int sum = (lastEncoded3 << 2) | encoded;

    if(sum == 0b1101 || sum == 0b0100 ||
       sum == 0b0010 || sum == 0b1011)
        posi3++;

    if(sum == 0b1110 || sum == 0b0111 ||
       sum == 0b0001 || sum == 0b1000)
        posi3--;

    lastEncoded3 = encoded;
}

//////////////////////////////////////////////////
// ENCODER ISR 4
//////////////////////////////////////////////////

void readEncoder4()
{
    int MSB = digitalRead(ENCA4);
    int LSB = digitalRead(ENCB4);

    int encoded = (MSB << 1) | LSB;
    int sum = (lastEncoded4 << 2) | encoded;

    if(sum == 0b1101 || sum == 0b0100 ||
       sum == 0b0010 || sum == 0b1011)
        posi4++;

    if(sum == 0b1110 || sum == 0b0111 ||
       sum == 0b0001 || sum == 0b1000)
        posi4--;

    lastEncoded4 = encoded;
}