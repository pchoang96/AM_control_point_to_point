#define clkw        0
#define c_clkw      1
#define encodPinA1  3
#define M1_p        6
#define M1_l        7
#define encodPinA2  2
#define M2_p        5
#define M2_l        4
#define DEBUG       1
/**--------------------Control signal variable-----------------------------------------------------------------------------------**/
double l_v,l_vt;
double r_v,r_vt; // pwm: pwm output. lv: mm/sec. lvt: tic/delta_t l:lert, r: right 
bool l_dir=clkw, r_dir=clkw;
/**----------------encoder parametter--------------------------------------------------**/
double l_p=0,r_p=0;
double l_d,r_d;
//  left pulses, left pre pulses, lef direction,....
/**--------------pid velocity calculation-------------------------------------------**/
volatile double  l_error=0.0,l_pre_error=0.0,l_integral=0.0,l_derivative=0.0,l_Ppart=0.0,l_Ipart=0.0,l_Dpart=0.0,l_out,l_set,l_ms,l_pre_out=0;
double const l_kP = 0.33, l_kI=3.53 ,l_kD = 0.004;
volatile double  r_error=0.0,r_pre_error=0.0,r_integral=0.0,r_derivative=0.0,r_Ppart=0.0,r_Ipart=0.0,r_Dpart=0.0,r_out,r_set,r_ms,r_pre_out=0;
double const r_kP = 0.33, r_kI=3.53,r_kD = 0.004;
/**----------------pid position calculator------------------------------------------**/
/* 
 * pid for motor:
 * period time: 20 ms
 * peek time: 400 ms
 * max speed (setting): value: 25/0.02 = 1250 pulses/s = 981.75 mm/s  <vm = vp*65*pi/wheels_encoder>
 * smallest speed (runable speed): value: 5/0.02= 250 pulses/s = 196.35mm/s
*/

double p_org[]={0.0,0.0,0.0}, p_now[]= {0.0,0.0,0.0}, p_end[]={0.0,0.0,0.0}; //{x,y,phi}
double lin_error=0.0,lin_pre_error=0.0,lin_integral=0.0,lin_derivative=0.0,lin_Ppart=0.0,lin_Ipart=0.0,lin_Dpart=0.0,lin_out,lin_set;
double const p_kP =3.33 ,p_kI=5.53,p_kD = 0.004;
volatile double ang_error=0.0,ang_pre_error=0.0,ang_integral=0.0,ang_derivative=0.0,ang_Ppart=0.0,ang_Ipart=0.0,ang_Dpart=0.0,ang_out,ang_set;
volatile bool  pid_type=0; //0 for angle, 1 for linear 
/**----------------car parameter---------------------**/
const double pi=3.1415;
const double sampletime = 0.02, inv_sampletime = 1/sampletime;
const double wheels_distance = 200, wheels_radius = 32.5, wheels_diameter=65,wheels_encoder = 160;// mm
const double wheel_ticLength = wheels_diameter*pi/wheels_encoder;//0.23mm
//const double wheel_ticAngle = 2*pi/(wheels_distance*pi/wheel_ticLength);//0.0022 rad
const bool l_motor=1,r_motor=0;
/**-----------------motion controller----------------------**/
double w=0.0, linear=0.0;
int i=0,sample;
bool wait_a_time = 0;
bool ang_pid=0,lin_pid=0;
//double PID_output;

void setup() {
  Serial.begin(9600);  
  pinMode(M1_l,OUTPUT);
  pinMode(M2_l,OUTPUT);
  pinMode(encodPinA1, INPUT_PULLUP);                  // encoder input pin
  pinMode(encodPinA2, INPUT_PULLUP);
  attachInterrupt(0, encoder_1 , FALLING);               // update encoder position
  attachInterrupt(1, encoder_2 , FALLING);
//--------------setup timer------------------------------------------ 
  TCCR1A = 0;
  TCCR1B = 0;
  TIMSK1 = 0;
  TCCR1B |= (1 << CS11) | (1 << CS10);    // prescale = 64 4us per pulse
  TCNT1 = 60535; //(12500*4)=50ms
  TIMSK1 |= (1 << TOIE1);                  // Overflow interrupt enable 
  sei();                                  // enable all interrupt
  
  p_end[0]=300;
  p_end[1]=400;
  p_end[2]=0;
}

void loop() 
{
/*
  l_set = 15;
  r_set = 15;
  l_dir = clkw;
  r_dir= clkw;
  delay(100);*/

  /* calculate_position(p_now[0],p_now[1],p_now[2],p_end[0],p_end[1],p_end[2]);
  pid_position();
  motion(lin_out,ang_out);
    l_error= l_set-abs(l_p);
  r_error= r_set-abs(r_p);
  l_out = PID_cal(l_error,l_pre_error,l_integral,l_derivative,l_Ppart,l_Ipart,l_Dpart,l_kP,l_kI,l_kD);
  r_out = PID_cal(r_error,r_pre_error,r_integral,r_derivative,r_Ppart,r_Ipart,r_Dpart,r_kP,r_kI,r_kD);
      Serial.println((String)  "Position:  " + "x: " +p_now[0]+ "  y: " +p_now[1]+ "  phi: " +p_now[2]);
  Serial.println((String)  "lin_error: "+ lin_error + "  ang_error: " + ang_error);
    Serial.println((String) "l_set: "+ l_set + " l_dir: " + l_dir);
    Serial.println((String) "r_set: " +r_set + " r_dir: " + r_dir);
   Serial.println((String)  "l_p: "+l_p + " r_p: "+  r_p);
    Serial.println((String) "l_out: "+ l_out + " l_dir: " + l_dir);
  Serial.println((String) "r_out: " +r_out + " r_dir: " + r_dir);
    Serial.println((String) "");
   delay(2000);
   */

}

void encoder_1()
{
  if(!l_dir) l_p ++;
  else l_p--;
}
/*----------------------------------------------------------*/
void encoder_2()
{  
  if(!r_dir) r_p ++;
  else r_p--;
}
/**--------------------------------------------------------------**/
void pwmOut(int Lpwm, int Rpwm, bool Ldir, bool Rdir)
{
  if(Lpwm==0 && Rpwm==0)
  { 
    analogWrite(M1_p,0); digitalWrite(M1_l,0);
    analogWrite(M2_p,0); digitalWrite(M2_l,0);
  }
  else if(Ldir==clkw && Rdir==clkw)
  {
    analogWrite(M1_p,0-Rpwm); digitalWrite(M1_l,1);
    analogWrite(M2_p,0-Lpwm); digitalWrite(M2_l,1);
  }

  else if(Ldir==c_clkw && Rdir==c_clkw)
  {
    analogWrite(M1_p,Rpwm); digitalWrite(M1_l,0);
    analogWrite(M2_p,Lpwm); digitalWrite(M2_l,0) ;  
  }

  else if(Ldir==c_clkw && Rdir==clkw)
  {
    analogWrite(M1_p,0-Rpwm); digitalWrite(M1_l,1);
    analogWrite(M2_p,Lpwm); digitalWrite(M2_l,0);  
  }

  else if(Ldir==clkw && Rdir==c_clkw)
  {
    analogWrite(M1_p,Rpwm); digitalWrite(M1_l,0);
    analogWrite(M2_p,0-Lpwm); digitalWrite(M2_l,1);
  }  
}

/*------------------------------------------------------------*/
void calculate_position(double xt,double yt, double pht, double xs, double ys, double phs)
{
  l_d = l_p*wheel_ticLength;
  r_d = r_p*wheel_ticLength;
  double c_d = (l_d + r_d )/2;
  
  xt += c_d*cos(pht);
  yt += c_d*sin(pht);
  pht += atan2((r_d-l_d),wheels_distance);

  w=atan2((ys-yt),(xs-xt));
  linear=sqrt((xs-xt)*(xs-xt)+(ys-yt)*(ys-yt));
  //update position
  p_now[0]=xt;
  p_now[1]=yt;
  p_now[2]=pht;
  //update error
  lin_error=linear;
  ang_error=(w-pht)*180/pi;//
  if (ang_error>180) {Serial.println((String)  "Position:  " + "x: " +p_now[0]+ "  y: " +p_now[1]+ "  phi: " +p_now[2]*180/pi);}
}
/*------------------------------------------------------------*/
void pid_position()
{
  if(abs(ang_error)>=170 && abs(lin_error)<50) {lin_pid=1; ang_error=0;}
  else lin_pid=0;
  if(abs(ang_error)>5)
  {
    if (ang_error<0) {ang_error = -ang_error; ang_pid =1;}
    else {ang_pid = 0;}
    ang_out = PID_cal(ang_error,ang_pre_error,ang_integral,ang_derivative,ang_Ppart,ang_Ipart,ang_Dpart,p_kP,p_kI,p_kD);
    pid_type = false;
    if (DEBUG)
    {
      Serial.println((String) "ang_error: " + ang_error);
    }
  }
  else if (abs(lin_error)>10)
  { 
    lin_out = PID_cal(lin_error,lin_pre_error,lin_integral,lin_derivative,lin_Ppart,lin_Ipart,lin_Dpart,p_kP,p_kI,p_kD);
    pid_type = true;
    if (DEBUG)
    {
      Serial.println((String) "lin_error: " + lin_error);
    }
  }
 if (abs(ang_error)<5 && abs(lin_error)<10)
  { 
    pwmOut(0,0,0,0);
    p_now[0]= p_now[1]= p_now[2]=0;
    p_end[0] =0;
    p_end[1]= 0;
//    wait_a_time=1;
  }
}
/**---------------------------------------------------------------------------------------------------------------**/
//PID_cal(l_error,l_pre_error,l_integral,l_derivative,l_Ppart,l_Ipart,l_Dpart,l_kP,l_kI,l_kD);
//PID_cal(r_error,r_pre_error,r_integral,r_derivative,r_Ppart,r_Ipart,r_Dpart,r_kP,r_kI,r_kD);
//PID_cal(ang_error,ang_pre_error,ang_integral,ang_derivative,ang_Ppart,ang_Ipart,ang_Dpart,p_kP,p_kI,p_kD);
//PID_cal(lin_error,lin_pre_error,lin_integral,lin_derivative,lin_Ppart,lin_Ipart,lin_Dpart,p_kP,p_kI,p_kD);
double PID_cal(double error,double pre_error,double _integral,double _derivative,double Ppart,double Ipart,double Dpart,double Kp,double Ki, double Kd)
{
  double PID_output;
    Ppart = Kp*error;

    _integral += error * sampletime;
    Ipart = Ki*_integral;

    _derivative  = (error  - pre_error )*inv_sampletime;
    Dpart  = Kd *_derivative ;

    PID_output = Ppart  + Ipart  + Dpart  ;
    pre_error  = error ;
    
    return PID_output;
}
/**-------------------------------------------------------------**/
void motion(double lin, double phi )
{
  //linear
  if(pid_type) 
  {
     if (!lin_pid) l_v = r_v = lin;
     else l_v = r_v = -lin;
  }
  else if (!pid_type)
  {
    if (!ang_pid)
    {
      l_v = (2*lin - phi*wheels_distance)/(2.0*wheels_radius);
      r_v = (2*lin + phi*wheels_distance)/(2.0*wheels_radius);
    }
    else
    {
      r_v = (2*lin - phi*wheels_distance)/(2.0*wheels_radius);
      l_v = (2*lin + phi*wheels_distance)/(2.0*wheels_radius);
    }
  } 
 /* else //curve moving
  {
    l_v = (2*lin - phi*wheels_distance)/(2.0*wheels_radius);
    r_v = (2*lin + phi*wheels_distance)/(2.0*wheels_radius);
  }
 */
//to l_vt and r_vt
  l_vt = l_v*(wheels_encoder/(pi*wheels_diameter))*sampletime;
  r_vt = r_v*(wheels_encoder/(pi*wheels_diameter))*sampletime;

  if (l_vt>=0) l_dir = clkw;//go ahead
  else l_dir =c_clkw;      //backhead
  if (r_vt>=0) r_dir = clkw;
  else r_dir =c_clkw;
  
  l_set=abs(l_vt);
  r_set=abs(r_vt);
  if (l_set>20) l_set=20;
  else if (l_set<5 && l_set>0.5) l_set=5 ;
  if (r_set>30) r_set=30;
  else if (r_set<5 && r_set>0.5) r_set =5;
}
/*------------------------------------------------------------*/
/**--------------------------------------------------------------**/
ISR(TIMER1_OVF_vect) 
{
  if(wait_a_time)
  {
    i++;
    if (i>=10) 
    {
      i=0; wait_a_time=0;
    }
  }
  else if(!wait_a_time)
  {
    if (abs(l_p) == abs (r_p)) { buff = abs( l_out-r_out ); }
    calculate_position(p_now[0],p_now[1],p_now[2],p_end[0],p_end[1],p_end[2]);
    pid_position();
    motion(lin_out,ang_out);
   // l_ms = l_p*inv_sampletime*wheels_diameter*pi/wheels_encoder;  
    l_error= l_set-abs(l_p);
    r_error= r_set-abs(r_p);
    if (l_error>=-1 && l_error<=1) l_error=0;
    if (r_error>=-1 && r_error<=1) r_error=0;
    l_out += PID_cal(l_error,l_pre_error,l_integral,l_derivative,l_Ppart,l_Ipart,l_Dpart,l_kP,l_kI,l_kD);
    r_out = l_out + buff;
    r_out += PID_cal(r_error,r_pre_error,r_integral,r_derivative,r_Ppart,r_Ipart,r_Dpart,r_kP,r_kI,r_kD);
    if (l_out>= 255) l_out = 255;
 //else if (l_out>10 && l_out<90) l_out+=10;
  if (r_out>= 255) r_out = 255;
//else if (r_out>10 && r_out<90) r_out+=10;
  if (abs(ang_error)<5 && abs(lin_error)<10) {l_out=r_out=l_dir=r_dir=0;}
  pwmOut(l_out,r_out,l_dir,r_dir);
  }
  


  if (DEBUG)
  {    
    delay(10);
    Serial.println((String)  "Position:  " + "x: " +p_now[0]+ "  y: " +p_now[1]+ "  phi: " +p_now[2]*180/pi);
  // Serial.println((String)  p_now[0]+ " " +p_now[1]+ " " +p_now[2]);
  // Serial.println((String)  "lin_error: "+ lin_error + "  ang_error: " + ang_error + "/ w: " + w + " linear: "+ linear );
   // Serial.println ((String) "l_p: "+l_p+" r_p: "+r_p);
   // Serial.println((String) "l_v:   "+ l_v + " r_v:  " +r_v);
   //Serial.println((String) "l_vt:   "+ l_vt + " r_vt:  " +r_vt);
   // Serial.println((String) "l_dir: " + l_dir+ ";      r_dir: " + r_dir);
   // Serial.println((String) " l_ms : "+ l_ms );
   // Serial.println((String) "l_out: "+ l_out + " l_dir: " + l_dir );
   // Serial.println((String) "r_out: " +r_out + " r_dir: " + r_dir );
   // Serial.println((String) "r_out: " +r_out + "  l_out: "+ l_out );
   // Serial.println((String) "");
  }
  l_p=0;
  r_p=0;
  TCNT1 = 60535;
}
/**--------------------------------------------------------------**/