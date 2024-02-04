#include <Arduino.h>
#include <Servo.h>
#include <math.h>
#include <VL53L0X.h>
#include <Wire.h>


VL53L0X tof;
float distance;

unsigned long now, lastcycle = 0, intervalo=250, intervalo2=300, intervalorodar=400, timeladodescer=40;

typedef struct
{
  int estado, nestado;
  unsigned long tes, tis;
} maq;

maq PFD, lado, MACRO;

void set_state(maq &m, int nestado)
{
  if (m.estado != nestado)
  {
    m.estado = nestado;
    m.tes = millis();
    m.tis = 0;
  }
}

Servo servo0, servo1, servo2, servo3, servo4, servo5, servo6, servo7;

int s0In=10, s0Out=141, s1In=165, s1Out=40, s2In=12, s2Out=137, s3In=160, s3Out=40, 
    s4Ret=175, s4Ext=95, s5Ret=2, s5Ext=85, s6Ret=169, s6Ext=95, s7Ret=4, s7Ext=85;

float p[8][3] = {
    {450, 1370, 2350},
    {480, 1445, 2430},
    {495, 1505, 2455},
    {450, 1400, 2355},
    {505, 1485, 2425},
    {560, 1550, 2500},
    {450, 1425, 2385},
    {525, 1500, 2450}
};

int anglim[8][2] = {
    {10, 141},      //Mx dentro, Mx fora
    {40, 165},      //Mx dentro, Mx fora
    {12, 137},      //Mx dentro, Mx fora
    {40, 160},      //Mx dentro, Mx fora
    {95, 175},      //Mx retraido, Mx extendido
    {2, 85},        //Mx retraido, Mx extendido
    {95, 169},      //Mx retraido, Mx extendido
    {4, 85}         //Mx retraido, Mx extendido
};

void iniservos(); // inicializa os servor nas portas certas
int anglecalc(int n, float angle); //n->servo number angle->degrees

void setup() {

  Serial.begin(115200);

  Wire.setSDA(20);
  Wire.setSCL(21);

  Wire.begin();

  servo0.attach(0, 450, 2500);
  servo1.attach(1, 470, 2500);
  servo2.attach(2, 470, 2500);
  servo3.attach(3, 450, 2500);
  servo4.attach(4, 470, 2500);
  servo5.attach(5, 470, 2500);
  servo6.attach(6, 450, 2500);
  servo7.attach(7, 480, 2500);

  if (!tof.init())
   {
      Serial.println("Failed to detect and initialize sensor!");
   }

   Serial.println("TOF Sensor Found!\n");
   tof.startContinuous();

  set_state(PFD, 0);
  set_state(lado, 0);
  set_state(MACRO, 0);
}

void loop() {

  now = millis();
  if (now - lastcycle > 0)
  {
    lastcycle = now;
    unsigned long curr_time = millis();

    PFD.tis = curr_time - PFD.tes;
    lado.tis = curr_time - lado.tes;
    MACRO.tis = curr_time - MACRO.tes;

     distance = tof.readRangeContinuousMillimeters();
   if (tof.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

      Serial.println(distance);


    

    // Pata frente direita
    if (PFD.estado == 0 && PFD.tis > 5000)
    {
      PFD.nestado = 11;
    }
    else if (PFD.estado == 1 && PFD.tis > intervalo)
    {
      PFD.nestado = 2;
    }
    else if (PFD.estado == 2 && PFD.tis > intervalo)
    {
      PFD.nestado = 1;
    }
    else if(PFD.estado == 11 && PFD.tis > intervalo){
      PFD.nestado=2;
    }

//////////////////////LADO///////////////
    if (lado.estado == 0 && MACRO.estado==1)
    {
      lado.nestado = 20;
    }
    else if (lado.estado == 1 && lado.tis > intervalo2)
    {
      lado.nestado = 2;
    }
    else if (lado.estado == 2 && lado.tis > intervalo2)
    {
      lado.nestado = 1;
    }
    else if (lado.estado == 1 && MACRO.estado==3)
    {
      lado.nestado = 30;
    }
    else if (lado.estado == 2 && MACRO.estado==3)
    {
      lado.nestado = 30;
    }
    else if(lado.estado == 11 && lado.tis>500){
      lado.nestado = 12;
    }
    else if(lado.estado == 12 && lado.tis>500){
      lado.nestado = 1;
    }
    else if(lado.estado == 20 && lado.tis>200){
      lado.nestado = 11;
    }
    else if(lado.estado == 30 && lado.tis>350)
    {
      lado.nestado=31;
    }
    else if(lado.estado == 31 && lado.tis>500)
    {
      lado.nestado=32;
    }
    else if(lado.estado == 32 && lado.tis>500)
    {
      lado.nestado=33;
    }
    else if(lado.estado == 33 && MACRO.estado==0)
    {
      lado.nestado=0;
    }


/////////////////////MACRO/////////////////
    if(MACRO.estado==0 && distance<200 && distance>100 && PFD.estado!=0){
      MACRO.nestado=1;
    }
    else if(MACRO.estado==1 && distance>750){
      MACRO.nestado=2;
    }
    else if(MACRO.estado==2 && MACRO.tis>2500){
      MACRO.nestado=3;
    }
    else if(MACRO.estado==3 && lado.estado==33 && lado.tis>200){
      MACRO.nestado=0;
    }

    set_state(PFD, PFD.nestado);
    set_state(lado, lado.nestado);
    set_state(MACRO, MACRO.nestado);



    float ini0=30, ini1=145, ini2=32, ini3=142, ini4=120+25-10, ini5=60-25+10, ini6=120+18-10, ini7=40-22+10;
    float offset0=0, offset1=0, offset2=0, offset3=0, offset4=0, offset5=0, offset6=0, offset7=0;
    float ang0=27, ang1=25, ang2=25, ang3=27, ang4=35+10-5, ang5=35+10-5, ang6=35+10-5, ang7=35+10-5;

    unsigned long timedescer=100;


///////////////////////PFD///////////////

    if(PFD.estado==0 && MACRO.estado==0){
      servo0.writeMicroseconds(anglecalc(0, ini0));
      servo4.writeMicroseconds(anglecalc(4, ini4));

      servo1.writeMicroseconds(anglecalc(1, ini1));
      servo5.writeMicroseconds(anglecalc(5, ini5));

      servo2.writeMicroseconds(anglecalc(2, ini2));
      servo6.writeMicroseconds(anglecalc(6, ini6));

      servo3.writeMicroseconds(anglecalc(3, ini3));
      servo7.writeMicroseconds(anglecalc(7, ini7));
    }
    else if(PFD.estado==11 && PFD.tis<intervalo-timedescer && MACRO.estado==0){
      servo0.writeMicroseconds(anglecalc(0,map(PFD.tis, 0,intervalo,ini0,ini0-ang0-offset0)));
      servo4.writeMicroseconds(anglecalc(4,ini4));

      servo1.writeMicroseconds(anglecalc(1,map(PFD.tis, 0,intervalo, ini1, ini1-ang1+offset1)));
      servo5.writeMicroseconds(anglecalc(5,map(PFD.tis, 0,timedescer,ini5, ini5+ang5)));

      servo2.writeMicroseconds(anglecalc(2,map(PFD.tis, 0,intervalo, ini2, ini2+ang2-offset2)));
      servo6.writeMicroseconds(anglecalc(6,ini6));

      servo3.writeMicroseconds(anglecalc(3,map(PFD.tis, 0,intervalo,ini3, ini3+ang3+offset3)));
      servo7.writeMicroseconds(anglecalc(7,map(PFD.tis, 0,timedescer,ini7, ini7+ang7)));
    }
    else if(PFD.estado==11 && PFD.tis>intervalo-timedescer && MACRO.estado==0){
      servo0.writeMicroseconds(anglecalc(0,map(PFD.tis, 0,intervalo,ini0,ini0-ang0-offset0)));
      servo4.writeMicroseconds(anglecalc(4,ini4));

      servo1.writeMicroseconds(anglecalc(1,map(PFD.tis, 0,intervalo, ini1, ini1-ang1+offset1)));
      servo5.writeMicroseconds(anglecalc(5,map(PFD.tis-(intervalo-timedescer), 0, timedescer, ini5+ang5, ini5)));

      servo2.writeMicroseconds(anglecalc(2,map(PFD.tis, 0,intervalo, ini2, ini2+ang2-offset2)));
      servo6.writeMicroseconds(anglecalc(6,ini6));

      servo3.writeMicroseconds(anglecalc(3,map(PFD.tis, 0,intervalo,ini3, ini3+ang3+offset3)));
      servo7.writeMicroseconds(anglecalc(7,map(PFD.tis-(intervalo-timedescer), 0, timedescer, ini7+ang7, ini7)));
    }
    else if(PFD.estado==1 && PFD.tis<intervalo-timedescer && MACRO.estado==0){
      servo0.writeMicroseconds(anglecalc(0,map(PFD.tis, 0,intervalo,ini0+ang0-offset0,ini0-ang0-offset0)));
      servo4.writeMicroseconds(anglecalc(4,ini4));

      servo1.writeMicroseconds(anglecalc(1,map(PFD.tis, 0,intervalo,ini1+ang1+offset1,ini1-ang1+offset1)));
      servo5.writeMicroseconds(anglecalc(5,map(PFD.tis, 0,timedescer,ini5, ini5+ang5)));

      servo2.writeMicroseconds(anglecalc(2,map(PFD.tis, 0,intervalo,ini2-ang2-offset2, ini2+ang2-offset2)));
      servo6.writeMicroseconds(anglecalc(6,ini6));

      servo3.writeMicroseconds(anglecalc(3,map(PFD.tis, 0,intervalo,ini3-ang3+offset3, ini3+ang3+offset3)));
      servo7.writeMicroseconds(anglecalc(7,map(PFD.tis, 0,timedescer,ini7, ini7+ang7)));
    }
    else if(PFD.estado==1 && PFD.tis>intervalo-timedescer && MACRO.estado==0){
      servo0.writeMicroseconds(anglecalc(0,map(PFD.tis, 0,intervalo,ini0+ang0-offset0,ini0-ang0-offset0)));
      servo4.writeMicroseconds(anglecalc(4,ini4));

      servo1.writeMicroseconds(anglecalc(1,map(PFD.tis, 0,intervalo,ini1+ang1+offset1,ini1-ang1+offset1)));
      servo5.writeMicroseconds(anglecalc(5,map(PFD.tis-(intervalo-timedescer), 0, timedescer, ini5+ang5, ini5)));

      servo2.writeMicroseconds(anglecalc(2,map(PFD.tis, 0,intervalo,ini2-ang2-offset2, ini2+ang2-offset2)));
      servo6.writeMicroseconds(anglecalc(6,ini6));

      servo3.writeMicroseconds(anglecalc(3,map(PFD.tis, 0,intervalo,ini3-ang3+offset3, ini3+ang3+offset3)));
      servo7.writeMicroseconds(anglecalc(7, map(PFD.tis-(intervalo-timedescer), 0, timedescer, ini7+ang7, ini7)));
    }
    else if(PFD.estado==2 && PFD.tis<intervalo-timedescer && MACRO.estado==0){
      servo0.writeMicroseconds(anglecalc(0,map(PFD.tis, 0,intervalo,ini0-ang0-offset0,ini0+ang0-offset0)));
      servo4.writeMicroseconds(anglecalc(4,map(PFD.tis, 0,timedescer,ini4, ini4-ang4)));

      servo1.writeMicroseconds(anglecalc(1,map(PFD.tis, 0,intervalo,ini1-ang1+offset1, ini1+ang1+offset1)));
      servo5.writeMicroseconds(anglecalc(5,ini5));

      servo2.writeMicroseconds(anglecalc(2,map(PFD.tis, 0,intervalo,ini2+ang2-offset2, ini2-ang2-offset2)));
      servo6.writeMicroseconds(anglecalc(6,map(PFD.tis, 0,timedescer,ini6, ini6-ang6)));

      servo3.writeMicroseconds(anglecalc(3,map(PFD.tis, 0,intervalo,ini3+ang3+offset3, ini3-ang3+offset3)));
      servo7.writeMicroseconds(anglecalc(7,ini7));
    }
    else if(PFD.estado==2 && PFD.tis>intervalo-timedescer && MACRO.estado==0){
      servo0.writeMicroseconds(anglecalc(0,map(PFD.tis, 0,intervalo,ini0-ang0-offset0,ini0+ang0-offset0)));
      servo4.writeMicroseconds(anglecalc(4,map(PFD.tis-(intervalo-timedescer), 0, timedescer, ini4-ang4, ini4)));

      servo1.writeMicroseconds(anglecalc(1,map(PFD.tis, 0,intervalo,ini1-ang1+offset1, ini1+ang1+offset1)));
      servo5.writeMicroseconds(anglecalc(5,ini5));

      servo2.writeMicroseconds(anglecalc(2,map(PFD.tis, 0,intervalo,ini2+ang2-offset2, ini2-ang2-offset2)));
      servo6.writeMicroseconds(anglecalc(6,map(PFD.tis-(intervalo-timedescer), 0, timedescer, ini6-ang6, ini6)));

      servo3.writeMicroseconds(anglecalc(3,map(PFD.tis, 0,intervalo, ini3+ang3+offset3, ini3-ang3+offset3)));
      servo7.writeMicroseconds(anglecalc(7,ini7));


    }

    ////////////////////////LADO////////////////
    float lado0=136, lado1=40, lado2=131, lado3=40, lado4=150, lado5=30, lado6=150, lado7=15;
    float ang=30, angcot=30;

    if(lado.estado==20  && MACRO.estado!=0){
      servo0.writeMicroseconds(anglecalc(0,ini0));
      servo4.writeMicroseconds(anglecalc(4,lado4));

      servo1.writeMicroseconds(anglecalc(1,ini1));
      servo5.writeMicroseconds(anglecalc(5,lado5));

      servo2.writeMicroseconds(anglecalc(2,ini2));
      servo6.writeMicroseconds(anglecalc(6,lado6));

      servo3.writeMicroseconds(anglecalc(3,ini3));
      servo7.writeMicroseconds(anglecalc(7, lado7));
    }
    else if(lado.estado==1 && lado.tis<(intervalo2-timeladodescer)&& MACRO.estado!=0){
      servo0.writeMicroseconds(anglecalc(0,map(lado.tis, 0,intervalo2, lado0, lado0-ang)));
      servo1.writeMicroseconds(anglecalc(1,map(lado.tis, 0,intervalo2, lado1, lado1+ang)));
      servo3.writeMicroseconds(anglecalc(3,map(lado.tis, 0,intervalo2, lado3+ang, lado3)));
      servo2.writeMicroseconds(anglecalc(2,map(lado.tis, 0,intervalo2, lado2-ang, lado2)));

      servo4.writeMicroseconds(anglecalc(4,lado4-angcot));
      servo6.writeMicroseconds(anglecalc(6,lado6-angcot));
      servo7.writeMicroseconds(anglecalc(7,lado7));
      servo5.writeMicroseconds(anglecalc(5,lado5));
    }
    else if(lado.estado==1&& lado.tis>(intervalo2-timeladodescer)&& MACRO.estado!=0){
      servo0.writeMicroseconds(anglecalc(0,map(lado.tis, 0,intervalo2, lado0, lado0-ang)));
      servo1.writeMicroseconds(anglecalc(1,map(lado.tis, 0,intervalo2, lado1, lado1+ang)));
      servo3.writeMicroseconds(anglecalc(3,map(lado.tis, 0,intervalo2, lado3+ang, lado3)));
      servo2.writeMicroseconds(anglecalc(2,map(lado.tis, 0,intervalo2, lado2-ang, lado2)));

      servo4.writeMicroseconds(anglecalc(4,lado4));
      servo6.writeMicroseconds(anglecalc(6,lado6));
      servo7.writeMicroseconds(anglecalc(7,lado7));
      servo5.writeMicroseconds(anglecalc(5,lado5));
    }
    else if(lado.estado==2 && lado.tis<(intervalo2-timeladodescer)&& MACRO.estado!=0){
      servo0.writeMicroseconds(anglecalc(0,map(lado.tis, 0,intervalo2, lado0-ang, lado0)));
      servo1.writeMicroseconds(anglecalc(1,map(lado.tis, 0,intervalo2, lado1+ang, lado1)));
      servo3.writeMicroseconds(anglecalc(3,map(lado.tis, 0,intervalo2, lado3, lado3+ang)));
      servo2.writeMicroseconds(anglecalc(2,map(lado.tis, 0,intervalo2, lado2, lado2-ang)));

      servo4.writeMicroseconds(anglecalc(4,lado4));
      servo6.writeMicroseconds(anglecalc(6,lado6));
      servo7.writeMicroseconds(anglecalc(7,lado7+angcot));
      servo5.writeMicroseconds(anglecalc(5,lado5+angcot));
    }
    else if(lado.estado==2 && lado.tis>(intervalo2-timeladodescer)&& MACRO.estado!=0){
      servo0.writeMicroseconds(anglecalc(0,map(lado.tis, 0,intervalo2, lado0-ang, lado0)));
      servo1.writeMicroseconds(anglecalc(1,map(lado.tis, 0,intervalo2, lado1+ang, lado1)));
      servo3.writeMicroseconds(anglecalc(3,map(lado.tis, 0,intervalo2, lado3, lado3+ang)));
      servo2.writeMicroseconds(anglecalc(2,map(lado.tis, 0,intervalo2, lado2, lado2-ang)));

      servo4.writeMicroseconds(anglecalc(4,lado4));
      servo6.writeMicroseconds(anglecalc(6,lado6));
      servo7.writeMicroseconds(anglecalc(7,lado7));
      servo5.writeMicroseconds(anglecalc(5,lado5));
    }
    else if(lado.estado == 11 && lado.tis<intervalorodar-timeladodescer && MACRO.estado!=0){
      servo0.writeMicroseconds(anglecalc(0,map(lado.tis, 0, intervalorodar, ini0, lado0-ang/2)));
      servo4.writeMicroseconds(anglecalc(4,150-angcot));

      servo1.writeMicroseconds(anglecalc(1,ini1));
      servo5.writeMicroseconds(anglecalc(5,30));

      servo2.writeMicroseconds(anglecalc(2,map(lado.tis, 0, intervalorodar, ini2, lado2-ang/2)));
      servo6.writeMicroseconds(anglecalc(6,150-angcot));

      servo3.writeMicroseconds(anglecalc(3,ini3));
      servo7.writeMicroseconds(anglecalc(7, 5));
    }
    else if(lado.estado == 11 && lado.tis>intervalorodar-timeladodescer && MACRO.estado!=0){
      servo0.writeMicroseconds(anglecalc(0,map(lado.tis, 0, intervalorodar, ini0, lado0-ang/2)));
      servo4.writeMicroseconds(anglecalc(4,150));

      servo1.writeMicroseconds(anglecalc(1,ini1));
      servo5.writeMicroseconds(anglecalc(5,30));

      servo2.writeMicroseconds(anglecalc(2,map(lado.tis, 0, intervalorodar, ini2, lado2-ang/2)));
      servo6.writeMicroseconds(anglecalc(6,150));

      servo3.writeMicroseconds(anglecalc(3,ini3));
      servo7.writeMicroseconds(anglecalc(7, 5));
    }
    else if(lado.estado == 12 && lado.tis<intervalorodar-timeladodescer && MACRO.estado!=0){
      servo0.writeMicroseconds(anglecalc(0,lado0-ang/2));
      servo4.writeMicroseconds(anglecalc(4,150));

      servo1.writeMicroseconds(anglecalc(1,map(lado.tis, 0, intervalorodar, ini1, lado1+ang/2)));
      servo5.writeMicroseconds(anglecalc(5,30+angcot));

      servo2.writeMicroseconds(anglecalc(2,lado2-ang/2));
      servo6.writeMicroseconds(anglecalc(6,150));

      servo3.writeMicroseconds(anglecalc(3,map(lado.tis, 0, intervalorodar, ini3, lado3+ang/2)));
      servo7.writeMicroseconds(anglecalc(7, 5+angcot));      
    }
    else if(lado.estado == 12 && lado.tis>intervalorodar-timeladodescer&& MACRO.estado!=0){
      servo0.writeMicroseconds(anglecalc(0,lado0-ang/2));
      servo4.writeMicroseconds(anglecalc(4,150));

      servo1.writeMicroseconds(anglecalc(1,map(lado.tis, 0, intervalorodar, ini1, lado1+ang/2)));
      servo5.writeMicroseconds(anglecalc(5,30));

      servo2.writeMicroseconds(anglecalc(2,lado2-ang/2));
      servo6.writeMicroseconds(anglecalc(6,150));

      servo3.writeMicroseconds(anglecalc(3,map(lado.tis, 0, intervalorodar, ini3, lado3+ang/2)));
      servo7.writeMicroseconds(anglecalc(7, 5));       
    }
    else if(lado.estado==30){
      servo0.writeMicroseconds(anglecalc(0,lado0-ang/2));
      servo4.writeMicroseconds(anglecalc(4,lado4));

      servo1.writeMicroseconds(anglecalc(1,lado1+ang/2));
      servo5.writeMicroseconds(anglecalc(5,lado5));

      servo2.writeMicroseconds(anglecalc(2,lado2-ang/2));
      servo6.writeMicroseconds(anglecalc(6,lado6));

      servo3.writeMicroseconds(anglecalc(3,lado3+ang/2));
      servo7.writeMicroseconds(anglecalc(7, lado7));
    }
    else if(lado.estado==31 && lado.tis<intervalorodar-timeladodescer){
      servo0.writeMicroseconds(anglecalc(0,map(lado.tis, 0, intervalorodar, lado0-ang/2, ini0)));
      servo4.writeMicroseconds(anglecalc(4,lado4-angcot));

      servo1.writeMicroseconds(anglecalc(1,lado1+ang/2));
      servo5.writeMicroseconds(anglecalc(5,lado5));

      servo2.writeMicroseconds(anglecalc(2,map(lado.tis, 0, intervalorodar, lado2-ang/2, ini2)));
      servo6.writeMicroseconds(anglecalc(6,lado6-angcot));

      servo3.writeMicroseconds(anglecalc(3,lado3+ang/2));
      servo7.writeMicroseconds(anglecalc(7, lado7));
    }
    else if(lado.estado==31 && lado.tis>intervalorodar-timeladodescer){
      servo0.writeMicroseconds(anglecalc(0,map(lado.tis, 0, intervalorodar, lado0-ang/2, ini0)));
      servo4.writeMicroseconds(anglecalc(4,ini4));

      servo1.writeMicroseconds(anglecalc(1,lado1+ang/2));
      servo5.writeMicroseconds(anglecalc(5,lado5));

      servo2.writeMicroseconds(anglecalc(2,map(lado.tis, 0, intervalorodar, lado2-ang/2, ini2)));
      servo6.writeMicroseconds(anglecalc(6,ini6));

      servo3.writeMicroseconds(anglecalc(3,lado3+ang/2));
      servo7.writeMicroseconds(anglecalc(7, lado7));
    }
    else if(lado.estado==32 && lado.tis<intervalorodar-timeladodescer){
      servo0.writeMicroseconds(anglecalc(0,ini0));
      servo4.writeMicroseconds(anglecalc(4,ini4));

      servo1.writeMicroseconds(anglecalc(1,map(lado.tis, 0, intervalorodar, lado1+ang/2, ini1)));
      servo5.writeMicroseconds(anglecalc(5,lado5+angcot));

      servo2.writeMicroseconds(anglecalc(2,ini2));
      servo6.writeMicroseconds(anglecalc(6,ini6));

      servo3.writeMicroseconds(anglecalc(3,map(lado.tis, 0, intervalorodar, lado3+ang/2, ini3)));
      servo7.writeMicroseconds(anglecalc(7, lado7+angcot));
    }
    else if(lado.estado==32 && lado.tis>intervalorodar-timeladodescer){
      servo0.writeMicroseconds(anglecalc(0,ini0));
      servo4.writeMicroseconds(anglecalc(4,ini4));

      servo1.writeMicroseconds(anglecalc(1,map(lado.tis, 0, intervalorodar, lado1+ang/2, ini1)));
      servo5.writeMicroseconds(anglecalc(5,ini5));

      servo2.writeMicroseconds(anglecalc(2,ini2));
      servo6.writeMicroseconds(anglecalc(6,ini6));

      servo3.writeMicroseconds(anglecalc(3,map(lado.tis, 0, intervalorodar, lado3+ang/2, ini3)));
      servo7.writeMicroseconds(anglecalc(7, ini7));
    }
  }
  
}

int anglecalc(int n, float angle){
  if(angle < anglim[n][0]){
    angle = anglim[n][0];
  }
  if(angle > anglim[n][1]){
    angle = anglim[n][1];
  }
  if(angle>=0 && angle<=90){
    return int(round((p[n][1]-p[n][0])/90*angle+p[n][0]));
  }
  else if(angle>90 && angle<=180){
    return int(round((p[n][2]-p[n][1])/90*angle+2*p[n][1]-p[n][2]));
  }
  return -1;
}