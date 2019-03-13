
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
//  left pules, left pre pulse, lef direction,....
/**--------------pid velocity calculation-------------------------------------------**/
volatile double  l_error=0.0,l_pre_error=0.0,l_integral=0.0,l_derivative=0.0,l_Ppart=0.0,l_Ipart=0.0,l_Dpart=0.0,l_out,l_set,l_ms;
double const l_kP = 0.33, l_kI=6.53 ,l_kD = 0.004;
volatile double  r_error=0.0,r_pre_error=0.0,r_integral=0.0,r_derivative=0.0,r_Ppart=0.0,r_Ipart=0.0,r_Dpart=0.0,r_out,r_set,r_ms;
double const r_kP = 0.33, r_kI=0.53,r_kD = 0.004;
/**----------------pid position calculator------------------------------------------**/
double p_org[]={0.0,0.0,0.0}, p_now[]= {0.0,0.0,0.0}, p_end[]={0.0,0.0,0.0}; //{x,y,phi}
double lin_error=0.0,lin_pre_error=0.0,lin_integral=0.0,lin_derivative=0.0,lin_Ppart=0.0,lin_Ipart=0.0,lin_Dpart=0.0,lin_out,lin_set;
double const p_kP =0.33 ,p_kI=6.53,p_kD = 0.0;
volatile double ang_error=0.0,ang_pre_error=0.0,ang_integral=0.0,ang_derivative=0.0,ang_Ppart=0.0,ang_Ipart=0.0,ang_Dpart=0.0,ang_out,ang_set;
volatile bool  pid_type=0; //0 for angle, 1 for linear 
/**----------------car parameter---------------------**/
const double pi=3.1415;
const double sampletime = 0.1, inv_sampletime = 1/sampletime;
const double wheels_distance = 207, wheels_radius = 32.5, wheels_diameter=65,wheels_encoder = 260;// mm
const double wheel_ticLength = wheels_diameter*pi/wheels_encoder;//0.23mm
const double wheel_ticAngle = 2*pi/(wheels_distance*pi/wheel_ticLength);//0.0022 rad
const bool l_motor=1,r_motor=0;
/**-----------------motion controller----------------------**/
double w=0.0, linear=0.0;
int i=0,sample;

double PID_output;

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
  TCNT1 = 40535; //(25000*4)=100ms
  TIMSK1 |= (1 << TOIE1);                  // Overflow interrupt enable 
  sei();                                  // enable all interrupt
}

void loop() 
{
  p_end[0]= 500;
  p_end[1]= 400;
  p_end[2]=0;
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
/*------------------------------------------------------------*/
void calculate_position(double xt,double yt, double pht, double xs, double ys, double phs)
{
  l_d = l_p*pi*wheels_diameter/wheels_encoder;
  r_d = r_p*pi*wheels_diameter/wheels_encoder;
  double c_d = (l_d + r_d )/2;
  pht = pht + (r_d-l_d)/wheels_distance;
  xt = xt+c_d*cos(pht);
  yt = yt+c_d*sin(pht);

  w=atan2((ys-yt),(xs-xt));
  linear=sqrt((xs-xt)*(xs-xt)+(ys-yt)*(ys-yt));
  //update position
  p_now[0]=xt; //0
  p_now[1]=yt; //0
  p_now[2]=pht; //0
  //update error
  lin_error=linear;
  ang_error=(w - pht);// *180/pi
}
/*------------------------------------------------------------*/
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
    analogWrite(M2_p,Lpwm); digitalWrite(M2_l,0);  
  }

  else if(Ldir==clkw && Rdir==c_clkw)
  {
    analogWrite(M1_p,0-Rpwm); digitalWrite(M1_l,1);
    analogWrite(M2_p,Lpwm); digitalWrite(M2_l,0);  
  }

  else if(Ldir==c_clkw && Rdir==clkw)
  {
    analogWrite(M1_p,Rpwm); digitalWrite(M1_l,0);
    analogWrite(M2_p,0-Lpwm); digitalWrite(M2_l,1);
  }  
}
/**--------------------------------------------------------------**/
void pid_position()
{
  if(abs(ang_error)>0.05)
  {
    ang_out = PID_cal(ang_error,ang_pre_error,ang_integral,ang_derivative,ang_Ppart,ang_Ipart,ang_Dpart,p_kP,p_kI,p_kD);
    pid_type = false;
    if (DEBUG)
    {
      Serial.println((String) "ang_out: " + ang_out);
    }
  }
  else if (abs(lin_error)>5.0)
  { 
    lin_out = PID_cal(lin_error,lin_pre_error,lin_integral,lin_derivative,lin_Ppart,lin_Ipart,lin_Dpart,p_kP,p_kI,p_kD);
    pid_type = true;
    if (DEBUG)
    {
      Serial.println((String) "lin_out: " + lin_out);
    }
  }
  else if (abs(lin_error)<5.0 && abs(ang_error)<0.05)
  { 
    l_out=0;
    r_out=0;
    pwmOut(0,0,0,0);
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
  if(pid_type) l_v=r_v=lin;
  else {r_v = phi*wheels_radius/2.0; l_v = -r_v;}
  /*
  else //curve moving
  {
    l_v = lin - phi*wheels_radius/2.0;
    r_v = lin + phi*wheels_radius/2.0;
  }
  */
//to l_vt and r_vt
  l_vt = l_v*(wheels_encoder/(pi*wheels_diameter))*sampletime;
  r_vt = r_v*(wheels_encoder/(pi*wheels_diameter))*sampletime;

  if (l_v>=0) l_dir = clkw;//go ahead
  else l_dir =c_clkw;      //backhead
  if (r_v>=0) r_dir = clkw;
  else r_dir =c_clkw;
  
  l_set=abs(l_vt);
  r_set=abs(r_vt);
  if (l_set>60) l_set=60;
  if (r_set>60) r_set=60;
}
/*------------------------------------------------------------*/
/**--------------------------------------------------------------**/
ISR(TIMER1_OVF_vect) 
{
  TCNT1 = 53035;
  calculate_position(p_now[0],p_now[1],p_now[2],p_end[0],p_end[1],p_end[2]);
  pid_position();
  motion(lin_out,ang_out);

  l_ms = l_p*inv_sampletime*wheels_diameter*pi/wheels_encoder;
  r_ms = r_p*inv_sampletime*wheels_diameter*pi/wheels_encoder;
  l_error= l_set-abs(l_p);
  r_error= r_set-abs(r_p);
  l_out += PID_cal(l_error,l_pre_error,l_integral,l_derivative,l_Ppart,l_Ipart,l_Dpart,l_kP,l_kI,l_kD);
  r_out += PID_cal(r_error,r_pre_error,r_integral,r_derivative,r_Ppart,r_Ipart,r_Dpart,r_kP,r_kI,r_kD);

  if (DEBUG)
  {  
    Serial.println((String)  "Position:  " + "x: " +p_now[0]+ "  y: " +p_now[1]+ "  phi: " +p_now[2]);
    // Serial.println((String)  p_now[0]+ " " +p_now[1]+ " " +p_now[2]);
    // Serial.println((String)  "lin_error: "+ lin_error + "  ang_error: " + ang_error);
    // Serial.println ((String) "l_p: "+l_p+" r_p: "+r_p);
    Serial.println((String) "l_vt: "+ l_vt );
    Serial.println((String) "l_ms : "+ l_ms );
    // Serial.println((String) "r_vt: " +r_vt);
    //Serial.println((String)  "l_p: "+l_p + " r_p: "+  r_p);
    //   Serial.println((String) "l_out: "+ l_out + " l_dir: " + l_dir);
    // Serial.println((String) "r_out: " +r_out + " r_dir: " + r_dir);
    // Serial.println((String) "");
  }
  
 if (l_out>= 255) l_out=r_out=255;
// else if (l_out<90 && l_out>=50) l_out=r_out=90;
 
  pwmOut(l_out,r_out,l_dir,r_dir);
  l_p=0;
  r_p=0;
  TCNT1 = 40535;

}
/**--------------------------------------------------------------**/