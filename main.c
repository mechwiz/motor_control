#include "NU32.h"          // config bits, constants, funcs for startup and UART
#include "encoder.h"
#include "stdio.h"
#include "stdlib.h"
// include other header files here

#define BUF_SIZE 200
#define SAMPLE_TIME 10  // 10 core timer ticks = 250 ns
#define PLOTPTS 100     // number of data points to plot
#define TRAJPTS 2000     // number of data points to plot

static volatile int ACTarray[PLOTPTS];      // measured values to plot
static volatile int REFarray[PLOTPTS];      // reference values to plot
static volatile int StoringData = 0;        // if thi flag = 1, currently
                                            // storing plot data
static volatile int Trajlength = 0;
static volatile float TRAJarray[TRAJPTS];      // measured values to plot
static volatile float REFJarray[TRAJPTS];      // reference values to plot
static volatile int TrajData = 0;        // if thi flag = 1, currently
                                            // storing plot data

static volatile enum mode{IDLE,PWM,ITEST,HOLD,TRACK}state;
static volatile int pwm_val = 0;
static volatile float Kp_c = 0, Ki_c = 0, Kd_c = 0;
static volatile float Kp = 0, Ki = 0, Kd = 0;
static volatile int Eint_c,eprev;

static volatile float refdeg,edeg;
static volatile float Eint;
static volatile float edegprev;
static volatile int ref = 0;
static volatile float traj_ref = 0;
static volatile int indxref;

void adc_init(void);
unsigned int adc_sample_convert(int);
void current_init(void);
void position_init(void);

void __ISR(_TIMER_4_VECTOR, IPL5SOFT) PosController(void) {
  static float current_angle = 0;

  switch (state) {
    case HOLD:
    {
      encoder_counts();
      edeg = refdeg - ((encoder_counts()-32768)/(4*448.0)*360.0);
      Eint+=edeg;
      float edegder = edeg-edegprev;
      edegprev = edeg;

      edeg = Kp*edeg+Ki*Eint+Kd*edegder;

      if (abs(edeg)>180) {
        if (edeg<0){
          edeg=-500;
        }else{
          edeg=500;
        }
      }else{
      edeg = -500.0 + (edeg+180.0)*(1000.0/360.0);
    }
      break;
    }

    case TRACK:
    {
      traj_ref = REFJarray[indxref];
      encoder_counts();
      current_angle = ((encoder_counts()-32768)/(4*448.0)*360.0);

      edeg = traj_ref - current_angle;
      Eint+=edeg;
      float edegder = edeg-edegprev;
      edegprev = edeg;

      edeg = Kp*edeg+Ki*Eint+Kd*edegder;

      if (abs(edeg)>180) {
        if (edeg<0){
          edeg=-500;
        }else{
          edeg=500;
        }
      }else{
        edeg = -500.0 + (edeg+180.0)*(1000.0/360.0);
      }

      if (TrajData) {
        TRAJarray[indxref] = current_angle; // store data in global arrays
        indxref++;                  // increment plot data index
        }
      if (indxref == Trajlength) {
        indxref = 0;        // reset the plot index
        TrajData = 0;    // tell main data is ready to be sent to MATLAB
        refdeg = traj_ref;
        state = HOLD;
      }
      break;
    }
  }
  LATDINV = 0x200;
  IFS0bits.T4IF = 0;
}

void __ISR(_TIMER_2_VECTOR, IPL5SOFT) Controller(void) {
  static int count = 0;
  static int plotind = 0;     // index for data arrays; counts up to PLOTPTS
  // static int i_actual = 0;
  // static int e = 0, u = 0;

  switch (state) {
    case IDLE:
    {
      OC1RS = 0;  // duty cycle = OC1RS/(PR3+1)
      break;
    }
    case PWM:
    {

      if (pwm_val < 0) {
        LATDSET = 0x100;
      }else{
        LATDCLR = 0x100;
      }
      OC1RS = (int)(abs(pwm_val)/100.0*4000.0+0.5);
      break;
    }
    case ITEST:
    {
      if (count == 25 || count == 50 || count == 75) {
        ref = -ref;
      }

      int i_actual = (int)(2.502*adc_sample_convert(0)-1273.16);
      int e = ref - i_actual;
      int eder = e-eprev;
      eprev = e;
      Eint_c += e;
      if (abs(Eint_c)>500) {
        if (Eint_c<0){
          Eint_c = -500;
        }else{
          Eint_c = 500;
        }
      }
      int u = (int)(Kp_c*e+Ki_c*Eint_c+Kd_c*eder);

      if (u>0) {
        LATDCLR = 0x100;
      } else {
        LATDSET = 0x100;
      }

      u = abs(u);
      if (u>100) {
        u = 100;
      }

      OC1RS = (unsigned int) ((u/100.0) *4000.0);
      if (StoringData) {
        ACTarray[plotind] = i_actual; // store data in global arrays
        REFarray[plotind] = ref;
        plotind++;                  // increment plot data index
        }
      if (plotind == PLOTPTS) {
        plotind = 0;        // reset the plot index
        StoringData = 0;    // tell main data is ready to be sent to MATLAB
      }

      count++;
      if (count > 99) {
        count = 0;
        state = IDLE;
      }
      break;
    }
    case HOLD:
    {
      ref = (int)(edeg);
      int i_actual = (int)(2.502*adc_sample_convert(0)-1273.16);
      int e = ref - i_actual;
      int eder = e-eprev;
      eprev = e;
      Eint_c += e;
      if (abs(Eint_c)>500) {
        if (Eint_c<0){
          Eint_c = -500;
        }else{
          Eint_c = 500;
        }
      }

      int u = (int)(Kp_c*e+Ki_c*Eint_c+Kd_c*eder);

      if (u>0) {
        LATDCLR = 0x100;
      } else {
        LATDSET = 0x100;
      }

      u = abs(u);
      if (u>100) {
        u = 100;
      }

      OC1RS = (unsigned int) ((u/100.0) *4000.0);
      break;
    }
    case TRACK:
    {
      ref = (int)(edeg);
      int i_actual = (int)(2.502*adc_sample_convert(0)-1273.16);
      int e = ref - i_actual;
      int eder = e-eprev;
      eprev = e;
      Eint_c += e;
      if (abs(Eint_c)>500) {
        if (Eint_c<0){
          Eint_c = -500;
        }else{
          Eint_c = 500;
        }
      }

      int u = (int)(Kp_c*e+Ki_c*Eint_c+Kd_c*eder);

      if (u>0) {
        LATDCLR = 0x100;
      } else {
        LATDSET = 0x100;
      }

      u = abs(u);
      if (u>100) {
        u = 100;
      }

      OC1RS = (unsigned int) ((u/100.0) *4000.0);
      break;
    }
  }

  IFS0bits.T2IF = 0;
}

int main()
{
  char buffer[BUF_SIZE];
  NU32_Startup(); // cache on, min flash wait, interrupts on, LED/button init, UART init
  NU32_LED1 = 1;  // turn off the LEDs
  NU32_LED2 = 1;
  int i = 0;
  __builtin_disable_interrupts();
  // in future, initialize modules or peripherals here
  encoder_init();
  adc_init();
  current_init();
  position_init();
  __builtin_enable_interrupts();

  while(1)
  {
    NU32_ReadUART3(buffer,BUF_SIZE); // we expect the next character to be a menu command
    NU32_LED2 = 1;                   // clear the error LED
    switch (buffer[0]) {
      case 'a':
      {
        sprintf(buffer,"%d\r\n",adc_sample_convert(0));
        NU32_WriteUART3(buffer);  // send encoder count to client
        break;
      }

      case 'b':
      {
        sprintf(buffer,"%d\r\n",(int)(2.502*adc_sample_convert(0)-1273.16));
        NU32_WriteUART3(buffer);  // send encoder count to client
        break;
      }

      case 'c':
      {
        encoder_counts();
        sprintf(buffer,"%d\r\n",encoder_counts());
        NU32_WriteUART3(buffer);  // send encoder count to client
        break;
      }

      case 'd':                      // dummy command for demonstration purposes
      {
        // int n = 0;
        // NU32_ReadUART3(buffer,BUF_SIZE);
        // sscanf(buffer, "%d", &n);
        // sprintf(buffer,"%d\r\n", n + 1); // return the number + 1
        // NU32_WriteUART3(buffer);
        // break;

        encoder_counts();
        sprintf(buffer,"%.1f\r\n",((encoder_counts()-32768)/(4*448.0)*360.0));
        NU32_WriteUART3(buffer);  // send encoder count to client
        break;
      }

      case 'e':
      {
        encoder_reset();
        sprintf(buffer,"%d\r\n",encoder_counts());
        NU32_WriteUART3(buffer);  // send encoder count to client
        break;
      }

      case 'f':
      {
        NU32_ReadUART3(buffer,BUF_SIZE);
        sscanf(buffer, "%d", &pwm_val);
        if (pwm_val>100) {
          pwm_val = 100;
        }else if(pwm_val<-100) {
          pwm_val = -100;
        }
        state = PWM;
        sprintf(buffer,"%d\r\n", pwm_val); // return the value
        NU32_WriteUART3(buffer);
        break;
      }

      case 'g':
      {
        NU32_ReadUART3(buffer,BUF_SIZE);
        sscanf(buffer, "%f,%f,%f", &Kp_c, &Ki_c, &Kd_c);
        sprintf(buffer,"%.2f,%.2f,%.2f\r\n", Kp_c,Ki_c,Kd_c); // return the numbers
        NU32_WriteUART3(buffer);
        break;
      }

      case 'h':
      {
        sprintf(buffer,"%.2f,%.2f,%.2f\r\n", Kp_c,Ki_c,Kd_c); // return the numbers
        NU32_WriteUART3(buffer);
        break;
      }

      case 'i':
      {
        NU32_ReadUART3(buffer,BUF_SIZE);
        sscanf(buffer, "%f,%f,%f", &Kp, &Ki, &Kd);
        sprintf(buffer,"%.2f,%.2f,%.2f\r\n", Kp,Ki,Kd); // return the numbers
        NU32_WriteUART3(buffer);
        break;
      }

      case 'j':
      {
        sprintf(buffer,"%.2f,%.2f,%.2f\r\n", Kp,Ki,Kd); // return the numbers
        NU32_WriteUART3(buffer);
        break;
      }

      case 'k':
      {
        sprintf(buffer,"%d\r\n",100); // return the numbers
        NU32_WriteUART3(buffer);
        StoringData = 1;
        Eint_c = 0;
        ref = 200;
        eprev = 0;
        state = ITEST;
        while (StoringData) {       // wait until ISR says data storing is done
            ;   // do nothing
        }
        for (i=0; i<PLOTPTS; i++) { // send plot data to MATALB
                        // when first number sent = 1, MATLAB knows we're done
            sprintf(buffer, "%d %d\r\n", ACTarray[i], REFarray[i]);
            NU32_WriteUART3(buffer);
        }
        break;
      }

      case 'l':
      {
        NU32_ReadUART3(buffer,BUF_SIZE);
        sscanf(buffer, "%f", &refdeg);
        sprintf(buffer,"%.1f\r\n",refdeg); // return the numbers
        NU32_WriteUART3(buffer);
        Eint = 0;
        Eint_c = 0;
        eprev = 0;
        edegprev = 0;
        state = HOLD;
        encoder_reset();
        break;
      }

      case 'm':
      {
        NU32_ReadUART3(buffer,BUF_SIZE);
        sscanf(buffer, "%d", &Trajlength);

        for (i=0; i<Trajlength; i++) { // send plot data to MATALB
                        // when first number sent = 1, MATLAB knows we're done
            NU32_ReadUART3(buffer,BUF_SIZE);
            sscanf(buffer, "%f\r\n",&REFJarray[i]);
        }
        sprintf(buffer,"%d\r\n",Trajlength);
        NU32_WriteUART3(buffer);
        break;
      }

      case 'n':
      {
        NU32_ReadUART3(buffer,BUF_SIZE);
        sscanf(buffer, "%d", &Trajlength);

        for (i=0; i<Trajlength; i++) { // send plot data to MATALB
                        // when first number sent = 1, MATLAB knows we're done
            NU32_ReadUART3(buffer,BUF_SIZE);
            sscanf(buffer, "%f\r\n",&REFJarray[i]);
        }
        sprintf(buffer,"%d\r\n",Trajlength);
        NU32_WriteUART3(buffer);
        break;
      }

      case 'o':
      {
        Eint = 0;
        Eint_c = 0;
        eprev = 0;
        edegprev = 0;
        TrajData = 1;
        indxref = 0;
        encoder_reset();
        sprintf(buffer,"%d\r\n",Trajlength);
        NU32_WriteUART3(buffer);
        state = TRACK;
        while (TrajData) {       // wait until ISR says data storing is done
            ;   // do nothing
        }
        for (i=0; i<Trajlength; i++) { // send plot data to MATALB
                        // when first number sent = 1, MATLAB knows we're done
            sprintf(buffer, "%f %f\r\n", TRAJarray[i], REFJarray[i]);
            NU32_WriteUART3(buffer);
        }
        break;
      }

      case 'p':
      {
        state = IDLE;
        sprintf(buffer,"%d\r\n", pwm_val); // return the value
        NU32_WriteUART3(buffer);
        break;
      }

      case 'r':
      {
        switch (state) {
          case IDLE:
          {
            sprintf(buffer,"%s\r\n", "IDLE"); // return the value
            break;
          }
          case PWM:
          {
            sprintf(buffer,"%s\r\n", "PWM"); // return the value
            break;
          }
          case ITEST:
          {
            sprintf(buffer,"%s\r\n", "ITEST"); // return the value
            break;
          }
          case HOLD:
          {
            sprintf(buffer,"%s\r\n", "HOLD"); // return the value
            break;
          }
          case TRACK:
          {
            sprintf(buffer,"%s\r\n", "TRACK"); // return the value
            break;
          }
        }
        NU32_WriteUART3(buffer);
      }
      case 'q':
      {
        // handle q for quit. Later you may want to return to IDLE mode here.
        break;
      }
      default:
      {
        NU32_LED2 = 0;  // turn on LED2 to indicate an error
        break;
      }
    }
  }
  return 0;
}

void adc_init(void) {
  AD1PCFGbits.PCFG0 = 0;  // AN0 is an adc pin
  AD1CON3bits.ADCS = 2;   // ADC clock period is Tad = 2*(ADCS+1)*Tpb=2*3*12.5ns=75ns
  AD1CON1bits.SSRC = 0b111; //conversion starts when sampling ends
  AD1CON1bits.ASAM = 0;     // autosampling does not begin after conversion
  AD1CON1bits.ADON = 1;     // turn on A/D converter
}

unsigned int adc_sample_convert(int pin) { // sample & convert the value on the given
                                           // adc pin the pin should be configured as an
                                           // analog input in AD1PCFG
    unsigned int elapsed = 0, finish_time = 0;
    AD1CHSbits.CH0SA = pin;                // connect chosen pin to MUXA for sampling
    AD1CON1bits.SAMP = 1;                  // start sampling
    elapsed = _CP0_GET_COUNT();
    finish_time = elapsed + SAMPLE_TIME;
    while (_CP0_GET_COUNT() < finish_time) {
      ;                                   // sample for more than 250 ns
    }
    // AD1CON1bits.SAMP = 0;                 // stop sampling and start converting
    while (!AD1CON1bits.DONE) {
      ;                                   // wait for the conversion process to finish
    }
    return ADC1BUF0;                      // read the buffer with the result
}

void current_init(void) {
  // Timer for 5 kHz current controller
  IPC2bits.T2IP = 5;
  IPC2bits.T2IS = 0;
  IFS0bits.T2IF = 0;
  IEC0bits.T2IE = 1;
  T2CONbits.TCKPS = 0;            //set prescaler to 1
  PR2 = 15999;         //set period register (takes into accnt prescaler of 1)
  TMR2 = 0;
  T2CONbits.ON = 1;         // turn on Timer2

  // Timer for 20 kHz PWM signal
  T3CONbits.TCKPS = 0;
  PR3 = 3999;
  TMR3 = 0;                // initial TMR3 count is 0
  T3CONbits.ON = 1;        // turn on Timer3

  // OC for 20 kHz PWM signal
  OC1CONbits.OCM = 0b110;
  OC1RS = 0;             // duty cycle = OC1RS/(PR3+1) = 0%
  OC1R = 0;              // initialize before turning OC1 on; afterward it is read only
  OC1CONbits.OCTSEL = 1;  // in conjuction with TMR3
  OC1CONbits.ON = 1;       // turn on OC1

  // Set D8 as output
  TRISDbits.TRISD8 = 0;
  LATDbits.LATD8 = 0;
}

void position_init(void) {
  // Timer for 200 Hz position controller
  IPC4bits.T4IP = 5;
  IPC4bits.T4IS = 1;
  IFS0bits.T4IF = 0;
  IEC0bits.T4IE = 1;
  T4CONbits.TCKPS = 3;            //set prescaler to 8
  PR4 = 49999;         //set period register (takes into accnt prescaler of 8)
  TMR4 = 0;
  T4CONbits.ON = 1;         // turn on Timer4

  // Set D9 as output
  TRISDbits.TRISD9 = 0;
  LATDbits.LATD9 = 0;
}
