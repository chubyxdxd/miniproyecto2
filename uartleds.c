//EJERCICIO9, GIRAR ROBOT CON 2 PWMs
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.c"
#include "stdio.h"
#include "stdlib.h"
#include <string.h>
#include "driverlib/adc.h"
#include "driverlib/pwm.h"

#define BUFFER_SIZE 50
#define PWM_FREQUENCY 200

#define SETPOINT 250
#define SETPOINTDIS 20

uint32_t FS = 120000000;
int cnt=0;
int flag=1;
int color=0;
int valx=0;
uint32_t pwmClock;
uint32_t pwmLoad;

int tivaT = 0;  // Variable que transmite número
int tivaR=250; //inicialmente 250
int dis=20;
int sw=0;


void setMotorSpeeds(int pid_output) { 
  pwmClock = SysCtlClockGet()/64;
  pwmLoad = (pwmClock / PWM_FREQUENCY) - 1;
  
  int vel_base = pwmLoad/3;
  //codigo aparte;
  float dis = pid_output;
  if(dis<0)dis=-dis;
  float aux = pwmLoad*(dis/150.0);
  char sendBuffer[20];
  /* intToStr(pid_output, sendBuffer); */
  /* enviar_string(sendBuffer); */
  /* enviar_string("g\r\n"); */
  if(pid_output<0){
    //esta a la izquierda
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 0);
    GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_0, GPIO_PIN_0);
    
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_PIN_2);
    GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_1, 0);
  }
  else{
    //esta a la derecha
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, GPIO_PIN_3);
    GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_0, 0);
    
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, 0);
    GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_1, GPIO_PIN_1);
  }  
  int pwm_left, pwm_right;
  pwm_left=aux;
  pwm_right=(aux-15);
  
  if (pwm_left < 20) pwm_left = 20;
  if (pwm_right < 20) pwm_right = 20;  
  if (pwm_left > pwmLoad/4) pwm_left = pwmLoad/4;
  if (pwm_right > pwmLoad/4) pwm_right = pwmLoad/4;
  intToStr(pwm_right, sendBuffer);
  enviar_string(sendBuffer);
  enviar_string("L\r\n");
  intToStr(pwm_left, sendBuffer);
  enviar_string(sendBuffer);
  enviar_string("R\r\n");
  
  PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, pwm_left);  // PF2 LEFT
  PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, pwm_right); // PK4 RIGHT
}

//PID PARAMETERS
float Kp = 0.13;
float Ki = 0.0;
float Kd = 0.1;

float error = 0, previous_error = 0, integral1 = 0;
void PID_controller(int x){
   error = SETPOINT - x;
   
   integral1 += error;
   float derivative = error - previous_error;
   int pid_output = (int)(Kp * error + Ki * integral1 + Kd * derivative);   

   previous_error = error;   
   setMotorSpeeds(pid_output);
}

void set_vel(int x){
  pwmClock = SysCtlClockGet()/64;
  pwmLoad = (pwmClock / PWM_FREQUENCY) - 1;
  if(x<0){
    //significa que el robot esta lejos, tenemos que hacer que se acerque;
    //MOTOR left
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, GPIO_PIN_3);
    GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_0, 0);
    //MOTOR right 
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_PIN_2);
    GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_1, 0);
    x=-x;
  }
  else{
    //MOTOR left
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 0);
    GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_0, GPIO_PIN_0);
    //MOTOR right 
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, 0);
    GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_1, GPIO_PIN_1);
  }
  int pwm_left, pwm_right;
  pwm_left=x;
  pwm_right=x-20;
  if (pwm_left < 20) pwm_left = 20;
  if (pwm_right < 20) pwm_right = 20;  
  if (pwm_left > pwmLoad) pwm_left = pwmLoad;
  if (pwm_right > pwmLoad) pwm_right = pwmLoad;
  PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, pwm_left);  // PF2 LEFT
  PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, pwm_right); // PK4 RIGHT
}

float integral2=0,error_prev=0;
void PID_distance(int dis){
  float KP = 1.5;
  float KI = 0.2;
  float KD = 0.1;
  float error = SETPOINTDIS - dis;
  integral2 += error;
  float derivative = error - error_prev;
  float output = KP * error + KI * integral2 + KD * derivative;
  error_prev = error;
  set_vel(output);
}

void Timer0IntHandler(void)
{
  TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC); // <-- Agregar esto

  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0); // PC4 ON
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0);
  if (flag == 0)
    {
      girar_robot();
    }
  // Prender el LED según el valor de flag
  else {
    if (flag == 1)
      {
	PID_controller(tivaR);
      }
    else if(flag == 2){
      // si detecta dos o mas objetos prende dos leds
      PID_controller(tivaR);
      GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_PIN_4); // PC4 ON
      GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0);
    }
    else{
      GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0); // PC4 ON
      GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0);
    }
    // seguir al objeto
  }
}


void enviar_string(const char* str)
{
    while (*str)
    {
        UARTCharPut(UART0_BASE, *str);
        str++;
    }
}

void delay_ms(uint32_t ms)
{
    SysCtlDelay((120000000 / 3 / 1000) * ms);
}

void configurar_pwm(void) {
  SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
  
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);  // PK4
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);  // PF2
  
  while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOK));
  while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));
  
  
  GPIOPinConfigure(GPIO_PK4_M0PWM6);  // PK4 -> M0PWM6 (Generator 3, Output 6)
  GPIOPinTypePWM(GPIO_PORTK_BASE, GPIO_PIN_4);

  GPIOPinConfigure(GPIO_PF2_M0PWM2);  // PF2 -> M0PWM2 (Generator 1, Output 2)
  GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);

  uint32_t pwmClock = SysCtlClockGet() / 64;
  uint32_t pwmLoad = (pwmClock / PWM_FREQUENCY) - 1;
  
  PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);  // Para M0PWM2
  PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);  // Para M0PWM6

  /* PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, pwmLoad); */
  PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, pwmLoad);
  PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, pwmLoad);
  
  PWMGenEnable(PWM0_BASE, PWM_GEN_1);
  PWMGenEnable(PWM0_BASE, PWM_GEN_3);
  
  PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);  // PF2
  PWMOutputState(PWM0_BASE, PWM_OUT_6_BIT, true);  // PK4

  PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2,  (pwmLoad * 10) / 100);
  PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6,  (pwmLoad * 10) / 100);
}

void girar_robot(){
  uint32_t pwmClock = SysCtlClockGet() / 64;
  uint32_t pwmLoad = (pwmClock / PWM_FREQUENCY) - 1;
  PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2,  (pwmLoad * 30) / 100);
  PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6,  (pwmLoad * 10) / 100);
  char sendBuffer[20];
  enviar_string("GIRAR");
  enviar_string("\r\n");
}

void robot_recto(int dis){
  pwmClock = SysCtlClockGet() / 64;
  pwmLoad = (pwmClock / PWM_FREQUENCY) - 1;
  PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2,  (pwmLoad * 20) / 100);
  PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6,  (pwmLoad * 12) / 100);
  if(dis>=24){
    //esta a la izquierda
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, GPIO_PIN_3);
    GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_0, 0);
    
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_PIN_2);
    GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_1, 0);
  }
  else if(dis<=16){
    //esta a la derecha
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 0);
    GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_0, GPIO_PIN_0);
    
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, 0);
    GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_1, GPIO_PIN_1);
  }  
}

void intToStr(int num, char *str) {
  int i = 0;
  bool isNegative = false;
  
  if (num == 0) {
    str[i++] = '0';
    str[i] = '\0';
    return;
  }
  
  if (num < 0) {
    isNegative = true;
    num = -num;
  }

  while (num != 0) {
    str[i++] = (num % 10) + '0';
    num = num / 10;
  }
  
  if (isNegative)
    str[i++] = '-';
  
  str[i] = '\0';
  
  // Invertir el string
  int start = 0;
    int end = i - 1;
    while (start < end) {
      char temp = str[start];
      str[start] = str[end];
      str[end] = temp;
      start++;
      end--;
    }
}

// Variables nuevas:
int main(void)
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC); // <-- Agregar esto
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);
  
  GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7); // <-- Agregar esto

    SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | 
                        SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000);
    
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTConfigSetExpClk(UART0_BASE, 120000000, 9600,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));   

    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_3);
    GPIOPinTypeGPIOOutput(GPIO_PORTP_BASE, GPIO_PIN_0);
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeGPIOOutput(GPIO_PORTP_BASE, GPIO_PIN_1);    
    
    configurar_pwm();

    /* TIMERS */
    IntMasterEnable();
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, FS*0.1);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER0_BASE, TIMER_A);
    IntEnable(INT_TIMER0A);
    /* FINAL */
    
    
    char buffer[BUFFER_SIZE];
    uint8_t buffer_index = 0;

    float timedata = 0;
    //float lasttime = 0;
    
    // Bucle principal
    while (1)
      {
	//robot_recto(14);
	// Si hay datos disponibles por UART
	if(tivaR>=170 || tivaR<=330 && flag==1){
	  if(dis>=24 || dis<=16)
	    robot_recto(dis);	  
	}
	else{
	  PID_controller(tivaR);
	}
	if (UARTCharsAvail(UART0_BASE))
	  {
	    char c = UARTCharGet(UART0_BASE);
	    
            if (c == '\n' || c == '\r')
	      {
		buffer[buffer_index] = '\0';
		
                // Procesamiento
		if (strlen(buffer) == 6)
		  {
		    char str_dis[3] = {buffer[0], buffer[1], '\0'};
                    char str_pos1[4] = {buffer[2], buffer[3], buffer[4], '\0'};
                    char str_flag[2] = {buffer[5], '\0'};
		    
                    dis = atoi(str_dis);
                    int pos1 = atoi(str_pos1);
                    int bandera = atoi(str_flag);
		    flag=bandera;
		    int suma = dis + pos1 + bandera;
		    tivaR=pos1;
		    char sendBuffer[20];
		    if(dis==99 || pos1==0)flag=0;
		    /* intToStr(flag, sendBuffer); */
		    /* enviar_string(sendBuffer); */
		    /* enviar_string("F\r\n"); */
		    
		  }
		buffer_index = 0;
		memset(buffer, 0, BUFFER_SIZE);
	      }
            else if (buffer_index < BUFFER_SIZE - 1)
	      {
                buffer[buffer_index++] = c;
	      }
	  }	
      }
}
