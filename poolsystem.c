
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "inc/hw_ints.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/i2c.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "driverlib/pwm.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "NextionDisplay.h"

//UART's
#define GPIO_PA0_U0RX 0x00000001
#define GPIO_PA1_U0TX 0x00000401
#define GPIO_PB0_U1RX 0x00010001
#define GPIO_PB1_U1TX 0x00010401

//endereços do DS1307
#define SLAVE_ADDR 0x68
#define SEC 0x00
#define MIN 0x01
#define HRS 0x02
#define DAY 0x03
#define DATE 0x04
#define MONTH 0x05
#define YEAR 0x06
#define CNTRL 0x07
#define GPIO_PD0_I2C3SCL 0x00030003  //PD0
#define GPIO_PD1_I2C3SDA 0x00030403  //PD1

#define PA6 (*((volatile long *) 0x40004100)) //PA6
#define PA7 (*((volatile long *) 0x40004200)) //PA7
//#define PD (*((volatile long *) 0x40007010))  //PD2
#define PF1 (*((volatile long *) 0x40025008)) //PF1

#define BOIA_ELEV (*((volatile long *) 0x40004010 )) //PA2
#define BOIA_RED (*((volatile long *) 0x40004020 )) //PA3
#define BOIA_ALG (*((volatile long *) 0x40004040 )) //PA4
#define BOIA_CL  (*((volatile long *) 0x40004080 )) //PA5
#define GPIO_PF2_M1PWM6 0x00050805 //PF2

#define BOIA_CENTRAL (*((volatile long *) 0x40025040 )) //PF4
#define BOMBA_CL (*((volatile long *) 0x40006080 )) //PC5
#define BOMBA_ALG (*((volatile long *) 0x40006100 )) //PC6


int jPH=0, saiu_inttimer, begin=0,x=0,back=0, pagina=0,MODO=0,pag=0,nao_modo=0,entrou_confi=0,modotemporario=0,configs=0;
int travar, vazao_ml_redutor,vazao_ml_elevador,vazao_ml_cloro,vazao_s,v_timer_ELE,v_timer_RED, v_timer_CL, v_timer_ALG,vazao_ml_algicida=0;
float V_piscina,medida_redutor,medida_elevador,agua_base, medida_final_RED, medida_final_ELE,medida_final_ALG,medida_final_CL,DOSAGEM_FINAL_RED,DOSAGEM_FINAL_ELE=0.0;
uint64_t TIME_RED,TIME_ELE,TIME_CL,TIME_ALG=0;
float volume_piscina=0.0;
int sai=0;
float DOSAGEM_REDUTOR1,DOSAGEM_REDUTOR2,DOSAGEM_AGUA,DOSAGEM_ELEVADOR1,DOSAGEM_ELEVADOR2,DOSAGEM_CLORO, DOSAGEM_ALGICIDA=0.0;
void print_float( float var);
uint32_t UARTDecGet(uint32_t ui32Base);

void WTimer0A_IntHandler(void);
void UART7_IntHandler(void);
void Config_UART0 (void);
void Config_UART1 (void);
void CONFIGURACAO_WTIMER(void);
void config_TIMER(uint64_t T);
void InitI2C3(void);
void valores_display(void);
void nivel_central(void);
unsigned char dec2bcd(unsigned char valor);
unsigned char bcd2dec(unsigned char valor);
void SetTime(char hora, char minuto, char segundo);
void SetData(char dia, char mes, char ano);
unsigned char GetData(unsigned char end);
int inteiro,inteiro1,inteiro2=0;
float inteiro2_decimal,valor_final=0.0;
char vet2[7]={"t1.txt"};
void niveis(void);
void config_INTPORTF(void);
int m=0;
char vetor_display[30];
int volume=0;
void iniciar_display(void);
void esperas(void);
void loop(void);
void modo(void);
void progress_bar(float PH);
void DOSAGEM(float dosagem);
void mostrar_PH(float PH_disp);
void print_RTC(float PH_RTC, unsigned char dia,unsigned char mes,unsigned char ano,unsigned char hora, unsigned char min,unsigned char seg);
int alterarCONF ();
void menu (void);

int count( int i) 
{
  int ret=0;
  while (i!=0){ i=i/10;ret++;}
  return ret;
}
char dosa_ph_disp(int vava);

int main (void)
{
  
  SysCtlClockSet(SYSCTL_SYSDIV_1| SYSCTL_USE_OSC | SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
  InitI2C3();
  Config_UART0 (); 
  
  
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_2| GPIO_PIN_3| GPIO_PIN_4| GPIO_PIN_5);
  GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_2| GPIO_PIN_3| GPIO_PIN_4|GPIO_PIN_5, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
  
  unsigned char sec, min, hour, date, month, year, sec_a=0;
  uint32_t ADC1Value[4],ADC1Value2[4],ADC1Value2_turbi[4],ADC1Value_turbi[4],volt0,volt1,volt2,volt3,volt_medio, dec_volt, MEDIA2=0;
  float Volt_decimal,Volt_decimal_turbi,Volt_decimalFINAL,MEDIA, MEDIA_turbidez, SOMA , SOMA_turbi , SOMAO, PH, pH=0.0;
  int primeira=0;
  float menor,soma_menor,menor_turbi=0.0;
  float maior,soma_maior,maior_turbi=0.0;
  
  
  
  SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
  
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_6| GPIO_PIN_7);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
  GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4);
  GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
  
  

  CONFIGURACAO_WTIMER();
  
  
  
  SetData(16,11,20); // dia,mes,ano ? comentar depois
 SetTime(15,00,00); // hora,minuto,segundo ? comentar depois 
  
  PA7=0x00;
  
  
  // SysCtlDelay(30000000); //delay sensor pH
  
  //SysCtlDelay(950000000); //delay sensor pH
  
  
  
  ConfigUART7();
  UART7_IntHandler();
  iniciar_display();
  UARTprintf("Iniciando..."); UARTprintf("\n");
  MODO=0; 
  nao_modo=0;
  menu ();
  
  UARTprintf("\n\n");
  print_float(volume_piscina);
  DOSAGEM_AGUA=1000;
  
  
  UARTprintf("\x1B[2J\x1B[0;0H");
  UARTprintf("\n----Análise do pH----");
  UARTprintf("\nAguarde...");
  //UARTIntDisable(UART7_BASE,UART_INT_RX|UART_INT_RT);
  
  
  //UART7_IntHandler();
  
  while(1)
  {
    while(alterarCONF()==0)
    {
      
      if(saiu_inttimer==0)
      {
        nivel_central();
        niveis();
        //SysCtlDelay(30000000); 
        SOMAO=0.0;
        pH=0.0;PH=0.0;
        //jPH=0; soma_menor=0.0; soma_maior=0.0; menor=0.0;maior=0.0; V_piscina=0.0; medida_elevador=0.0;medida_redutor=0.0;MEDIA=0.0; MEDIA_turbidez=0.0;
        GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_5);
        ADCSequenceConfigure(ADC1_BASE, 2, ADC_TRIGGER_PROCESSOR, 0);
        
        ADCSequenceStepConfigure(ADC1_BASE, 2, 0, ADC_CTL_CH8);
        ADCSequenceStepConfigure(ADC1_BASE, 2, 1, ADC_CTL_CH8);
        ADCSequenceStepConfigure(ADC1_BASE, 2, 2, ADC_CTL_CH8);
        ADCSequenceStepConfigure(ADC1_BASE, 2 ,3, ADC_CTL_CH8|ADC_CTL_IE|ADC_CTL_END);
        ADCHardwareOversampleConfigure(ADC1_BASE, 64); //faça um teste sem esta instrução
        ADCSequenceEnable(ADC1_BASE, 2);
        while(jPH<400)
        {
          SOMA=0;
          ADCProcessorTrigger(ADC1_BASE, 2);
          while(ADCIntStatus(ADC1_BASE, 2, false)==0);
          ADCIntClear(ADC1_BASE, 2);
          ADCSequenceDataGet(ADC1_BASE, 2, ADC1Value);
          
          MEDIA=(ADC1Value[0]+ADC1Value[1]+ADC1Value[2]+ADC1Value[3])/4;
          
          if(jPH==0)
          {
            for(int i=0;i<4;i++)
            {
              ADC1Value2[i]=ADC1Value[i];
              menor=ADC1Value2[0];
              maior=ADC1Value2[0];
              
            }
          }
          for(int i=3;i>0;i--)
          {
            
            ADC1Value2[i]=ADC1Value2[i-1]; 
            
          }
          ADC1Value2[0]= MEDIA;
          
          for(int i=3;i>=0;i--)
          {
            
            SOMA =  SOMA  + ADC1Value2[i];
            
            if(ADC1Value2[i]<menor)
            {
              menor= ADC1Value2[i];
            }
            if(ADC1Value2[i]>maior)
            {
              maior=ADC1Value2[i];
            }
            
          }    
          if(jPH==0)
          {
            
            soma_menor= SOMA;
            soma_maior= SOMA;
            SOMAO=0;
          }
          
          if( SOMA< soma_menor)
          {
            soma_menor= SOMA;
          }
          else if( SOMA> soma_maior)
          {
            soma_maior= SOMA;
          }
          
          SOMAO=SOMAO+SOMA;
          
          jPH++;
          
        }
        jPH=0;
        SOMAO=(SOMAO-(soma_maior+soma_menor))/400;
        
        SOMAO=(SOMAO-(maior+menor))/2;
        
        Volt_decimal= ( SOMAO*33/40950)-0.1295;
        
        pH=(20.625+(-8.1198*(  Volt_decimal)));
        
        //PH=pH;
        
        PH=7.38;
        progress_bar(PH);
        
        // UARTprintf("\npH = ");
        print_float(PH);
        UARTprintf("\n");
        print_float(Volt_decimal);
        UARTprintf("\n");
        
        mostrar_PH(PH);
        
        V_piscina=volume_piscina;
        agua_base=DOSAGEM_AGUA;
        vazao_s=60;
        
        
        if(PH>7.0 && PH<7.4)
        {
          PF1=0x02;
          UARTprintf("\npH está no intervalo de idealidade!");
          
          saiu_inttimer=1;
          
        }
        
        
        else if( PH >= 6.2 && PH <= 7.0) //  ELEVADOR
        {
          medida_elevador=DOSAGEM_ELEVADOR2;
          medida_final_ELE= (V_piscina*medida_elevador)/agua_base;
          UARTprintf("\n");
          
          vazao_ml_elevador=76;
          
          
          
          if(PH>=6.2 && PH<6.3){x=0;}
          
          else if(PH>=6.3 && PH<6.4){x=1;}
          
          else if(PH>=6.4 && PH<6.5){x=2;}
          
          else if(PH>=6.5 && PH<6.6){x=3;}
          
          else if(PH>=6.6 && PH<6.7){x=4;}
          
          else if(PH>=6.7 && PH<6.8){x=5;}
          
          else if(PH>=6.8 && PH<6.9){x=6;}
          
          else if(PH>=6.9 && PH<=7.0){x=7;}
          
          
          DOSAGEM_FINAL_ELE=((-medida_final_ELE/8)*x) + medida_final_ELE;
          print_float(DOSAGEM_FINAL_ELE);
          
          v_timer_ELE=((DOSAGEM_FINAL_ELE)*vazao_s)/vazao_ml_elevador;
          
          UARTprintf("\n%d",v_timer_ELE);
          TIME_ELE= (v_timer_ELE)/0.0000000625;
          config_TIMER( TIME_ELE);
          
          TimerEnable(WTIMER0_BASE, TIMER_A); //inicia
          PA7=0xFF;
          PF1=0x02;
          
          DOSAGEM(DOSAGEM_FINAL_ELE);
          
          UARTprintf("\npH - entre que 6,2 e 7 ------- ELEVADOR");
          while(saiu_inttimer==0);
          PA7=0x00;
        }
        else if(PH < 6.2) // ELEVADOR
        {
          medida_elevador=DOSAGEM_ELEVADOR1;
          medida_final_ELE= (V_piscina*medida_elevador)/agua_base;
          UARTprintf("\n");
          print_float( medida_final_ELE);
          vazao_ml_elevador=76;
          v_timer_ELE=(medida_final_ELE*vazao_s)/vazao_ml_elevador;
          UARTprintf("\n%d",v_timer_ELE);
          TIME_ELE= (v_timer_ELE)/0.0000000625;
          config_TIMER( TIME_ELE);
          
          TimerEnable(WTIMER0_BASE, TIMER_A); //inicia
          PA7=0xFF;
          PF1=0x02;
          DOSAGEM(medida_final_ELE);
          UARTprintf("\npH - menor/igual a 6,2 ------- ELEVADOR");
          while(saiu_inttimer==0); 
          PA7=0x00;
        }
        else if(PH>=8) // REDUTOR
        {
          medida_redutor=DOSAGEM_REDUTOR1;
          medida_final_RED= (V_piscina*medida_redutor)/agua_base;
          print_float( medida_final_RED);
          vazao_ml_redutor=75;
          v_timer_RED=(medida_final_RED*vazao_s)/vazao_ml_redutor;
          UARTprintf("\n%d",v_timer_RED);
          TIME_RED= (v_timer_RED)/0.0000000625;
          config_TIMER( TIME_RED);
          
          TimerEnable(WTIMER0_BASE, TIMER_A); //inicia
          PA6=0xFF;
          PF1=0x02;
          DOSAGEM(medida_final_RED);
          UARTprintf("\npH - maior que 8 ------- REDUTOR");
          while(saiu_inttimer==0);
          PA6=0x00;
        }
        else if(PH >= 7.4 && PH < 8) // REDUTOR
        {
          medida_redutor=DOSAGEM_REDUTOR2;
          medida_final_RED= (V_piscina*medida_redutor)/agua_base;
          UARTprintf("\n");
          
          vazao_ml_redutor=75;
          
          
          if(PH>=7.4 && PH<7.5){x=5;}
          
          else if(PH>=7.5 && PH<7.6){x=4;}
          
          else if(PH>=7.6 && PH<7.7){x=3;}
          
          else if(PH>=7.7 && PH<7.8){x=2;}
          
          else if(PH>=7.8 && PH<7.9){x=1;}
          
          else if(PH>=7.9 && PH<8){x=0;}
          
          DOSAGEM_FINAL_RED=((-medida_final_RED/6)*x) + medida_final_RED;
          print_float(DOSAGEM_FINAL_RED);
          
          v_timer_RED=((DOSAGEM_FINAL_RED)*vazao_s)/vazao_ml_redutor;
          
          
          UARTprintf("\n%d",v_timer_RED);
          TIME_RED= (v_timer_RED)/0.0000000625;
          config_TIMER( TIME_RED);
          
          TimerEnable(WTIMER0_BASE, TIMER_A); //inicia
          PA6=0xFF;
          PF1=0x02;
          DOSAGEM(DOSAGEM_FINAL_RED);
          UARTprintf("\npH - entre 7,4 e 8 ------- REDUTOR");
          
          while(saiu_inttimer==0);
           PA6=0x00;
          UARTprintf("FDP");
        }
      }
      /* 
      GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_4);
      ADCSequenceConfigure(ADC1_BASE, 2, ADC_TRIGGER_PROCESSOR, 0);
      
      ADCSequenceStepConfigure(ADC1_BASE, 2, 0, ADC_CTL_CH9);
      ADCSequenceStepConfigure(ADC1_BASE, 2, 1, ADC_CTL_CH9);
      ADCSequenceStepConfigure(ADC1_BASE, 2, 2, ADC_CTL_CH9);
      ADCSequenceStepConfigure(ADC1_BASE, 2 ,3, ADC_CTL_CH9|ADC_CTL_IE|ADC_CTL_END);
      ADCHardwareOversampleConfigure(ADC1_BASE, 64); //faça um teste sem esta instrução
      ADCSequenceEnable(ADC1_BASE, 2);
      */
      
      if(saiu_inttimer==1){UARTprintf("\n\rNova verificação em 7 ou 2 dias...");}
      
      else if(saiu_inttimer==2){UARTprintf("\n\rNova verificação em 2 horas...");}
      UARTprintf("\n");
      
      
      unsigned char h_inicial,dia_inicial,min_inicial=0;
      
      begin=0;
      SysCtlDelay(30000000); 
      sai=0;
      pag=0;
      while ( ((saiu_inttimer!=0) && (begin==0)) && (alterarCONF()==0))
      {
        
        year = GetData(YEAR);
        month = GetData(MONTH);
        date = GetData(DATE);
        hour = GetData(HRS);
        min = GetData(MIN);
        sec = GetData(SEC);
        
        if(sec!=sec_a)
        {
          UARTprintf("\r%02d/%02d/%02d - %02d:%02d:%02d",date,month,year,hour,min,sec);
          sec_a=sec;
          //page 12 display
          print_RTC(PH,date,month,year,hour,min,sec);
          
        }
        if(primeira==0)
        {  
          primeira=1;
          h_inicial=hour;
          min_inicial=min;
          dia_inicial=date;
        }
        if(saiu_inttimer==1)
        {
          
          if(sai==0)
          {
            sai=1;
            /*char vet_pag[8]={"page 19"};
            for(int p=0; p<7; p++) UARTCharPut(UART7_BASE, vet_pag[p]);
            UARTCharPut(UART7_BASE, 0xFF);
            UARTCharPut(UART7_BASE, 0xFF);
            UARTCharPut(UART7_BASE, 0xFF);
            
            medida_final_CL= (V_piscina* DOSAGEM_CLORO)/agua_base;
            UARTprintf("\n");
            print_float( medida_final_CL);
            vazao_ml_cloro=76;
            v_timer_CL=(medida_final_CL*vazao_s)/vazao_ml_cloro;
            UARTprintf("\n%d",v_timer_CL);
            TIME_CL = (v_timer_CL)/0.0000000625;
            config_TIMER( TIME_CL);
            
            TimerEnable(WTIMER0_BASE, TIMER_A); //inicia
            PA7=0xFF;
            PF1=0x02;
            DOSAGEM(medida_final_CL);
            //UARTprintf("\npH - menor/igual a 6,2 ------- ELEVADOR");
            while(saiu_inttimer==1); 
            PA7=0x00; */
            saiu_inttimer=1; pag=0; primeira=0;
          }
          
          if((MODO==2)&&(date == (dia_inicial +7)) && (hour == h_inicial) && (min == min_inicial))
          {
            begin=1;
            saiu_inttimer=0;
            j=0;
            pag=0;
          }
          else if((MODO==1)&&(date == (dia_inicial +2)) && (hour == h_inicial) && (min == min_inicial))
          {
            begin=1;
            saiu_inttimer=0;
            j=0;
            pag=0;
          }
          //else if((date == (dia_inicial+1))&& (hour == h_inicial) && (min == min_inicial))
         // else if((date == (dia_inicial+1))&& (hour == (h_inicial-2)) && (min == min_inicial))
          //{
            char vet_pag[8]={"page 20"};
            for(int p=0; p<7; p++) UARTCharPut(UART7_BASE, vet_pag[p]);
            UARTCharPut(UART7_BASE, 0xFF);
            UARTCharPut(UART7_BASE, 0xFF);
            UARTCharPut(UART7_BASE, 0xFF);
            
            
            
            SOMA_turbi=0.0;
            int lu=0;
            while(lu<400)
            {
              ADCProcessorTrigger(ADC1_BASE, 2);
              while(ADCIntStatus(ADC1_BASE, 2, false)==0);
              ADCIntClear(ADC1_BASE, 2);
              ADCSequenceDataGet(ADC1_BASE, 2, ADC1Value_turbi);
              
              MEDIA_turbidez=(ADC1Value_turbi[0]+ADC1Value_turbi[1]+ADC1Value_turbi[2]+ADC1Value_turbi[3])/4;
              
              if(lu==0)
              {
                for(int i=0;i<4;i++)
                {
                  ADC1Value2_turbi[i]=ADC1Value_turbi[i];
                  menor_turbi=ADC1Value2_turbi[0];
                  maior_turbi=ADC1Value2_turbi[0];
                  
                }
              }
              for(int i=3;i>0;i--)
              {
                
                ADC1Value2_turbi[i]=ADC1Value2_turbi[i-1]; 
                
              }
              ADC1Value2_turbi[0]=  MEDIA_turbidez;
              
              for(int i=3;i>=0;i--)
              {
                
                SOMA_turbi =  SOMA_turbi  + ADC1Value2_turbi[i];
                
                if(ADC1Value2_turbi[i]<menor_turbi)
                {
                  menor_turbi= ADC1Value2_turbi[i];
                }
                if(ADC1Value2_turbi[i]>maior_turbi)
                {
                  maior_turbi=ADC1Value2_turbi[i];
                }
                
              } 
              lu++;
            }
            
            lu=0;
            SOMA_turbi=(SOMA_turbi -(menor_turbi+menor_turbi))/1598;
            Volt_decimal_turbi= (SOMA_turbi*33/40950);
            float NTU_val=0.0;
            NTU_val= ((-2083.4722)* (Volt_decimal_turbi* Volt_decimal_turbi))+(7864.9339*Volt_decimal_turbi)-4419.4579;
            NTU_val=1.11; 
            UARTprintf("\x1B[2J\x1B[0;0H");print_float( NTU_val);
            UARTprintf("\n"); 
            print_float(Volt_decimal_turbi);
            
            if(NTU_val >= 1.0)
            {
              medida_final_ALG= (V_piscina* DOSAGEM_ALGICIDA)/agua_base;
              UARTprintf("\n");
              print_float( medida_final_ALG);
              vazao_ml_algicida=76;
              v_timer_ALG=(medida_final_ALG*vazao_s)/vazao_ml_algicida;
              UARTprintf("\n%d",v_timer_ALG);
              TIME_ALG = (v_timer_ALG)/0.0000000625;
              config_TIMER( TIME_ALG);
              
              TimerEnable(WTIMER0_BASE, TIMER_A); //inicia
              PA6=0xFF;
              PF1=0x02;
              DOSAGEM(1.11);
            
              while(saiu_inttimer==1); 
              PA6=0x00;
              sai=0;
              saiu_inttimer=1;
            }
          //}
        }
        
        else if(saiu_inttimer==2)
        {
          sai=0;
          if( (hour == (h_inicial +2)) && (min == min_inicial))
          {
            begin=1;
            saiu_inttimer=0;
            j=0;
            pag=0;
          }
          
        }
        
        
      }
      if(MODO==0){break;}
      else 
      {
        primeira=0;begin=0;
      }
    }
    
    menu();
    UARTprintf("olaaaa");
  }
}
void print_float( float var)
{
  int x1=0,x2=0,x3=0;
  x1=(int)var;
  if(x1!= 0)
  {
    x2=(var*10)-(((int)var)*10);
    x2=fabs(x2);
    x3=(var*100)-(((int)var)*100);
    x3=fabs(x3)%10;
    UARTprintf("%d.%d%d",x1,x2,x3);
  }
  else
  {
    x1=0;
    x2=(var*10);
    x2=fabs(x2);
    x3=(var*100)-(int)var*100;
    x3=fabs(x3);
    if(var<0)
    {
      UARTprintf("-%d.%d%d",x1,x2,x3);
      
    }
    else
    {
      UARTprintf("%d.%d%d",x1,x2,x3);
      
    }
    
  }
  
  
}

void valores_display(void)
{
  for(int p=0; p<6; p++) UARTCharPut(UART7_BASE, vet2[p]);
  UARTCharPut(UART7_BASE, 0x3D);// Sinal de igual
  UARTCharPut(UART7_BASE, 0x22);  
  for(int p=0; p<m+1; p++)UARTCharPut(UART7_BASE, vetor_display[p]);
  UARTCharPut(UART7_BASE, 0x22); 
  UARTCharPut(UART7_BASE, 0xFF);
  UARTCharPut(UART7_BASE, 0xFF);
  UARTCharPut(UART7_BASE, 0xFF);
}


void config_TIMER(uint64_t T)
{
  TimerLoadSet64(WTIMER0_BASE,(T)-1);
  TimerEnable(WTIMER0_BASE, TIMER_A); //inicia
}
void CONFIGURACAO_WTIMER(void)
{
  ////Configuração da GPIO F ////
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
  GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);
  
  ////Configuração do Timer0A para one shot – 16 bits//
  SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER0);
  TimerConfigure(WTIMER0_BASE, TIMER_CFG_A_ONE_SHOT);
  TimerIntEnable(WTIMER0_BASE, TIMER_TIMA_TIMEOUT);
  IntEnable(INT_WTIMER0A_TM4C123);
  IntRegister(INT_WTIMER0A_TM4C123, WTimer0A_IntHandler);
  
  
}
uint32_t UARTDecGet(uint32_t ui32Base)
{
  uint32_t numero=0,tamanho=0;
  char caractere;
  
  caractere= UARTCharGet(ui32Base);
  while(caractere!= 0x0D)//CR - ENTER
  {
    if((caractere>='0')&&(caractere<='9'))
    {
      numero=10*numero+(caractere-0x30);
      tamanho++;
      UARTCharPut(ui32Base, caractere);
    }
    else if((caractere==0x08)&&(tamanho!=0))//backspace
    {
      numero/=10;
      tamanho--;
      UARTCharPut(ui32Base, caractere);
    }
    caractere= UARTCharGet(ui32Base);
  }
  return numero;
}
void WTimer0A_IntHandler(void) //ISR do Timer0A
{
  
   PF1=0x00;
   PA6=0x00;
   PA7=0x00;
  // BOMBA_ALG=0x00;
  
  saiu_inttimer=2;
  UARTprintf("\nvaaaaamooooooooooooo");
  
  
  PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT, false); 
  PWMGenDisable(PWM1_BASE, PWM_GEN_1); 
  
  TimerDisable(WTIMER0_BASE, TIMER_A);
  TimerIntClear(WTIMER0_BASE, TIMER_TIMA_TIMEOUT);
  
}
void InitI2C3(void)
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C3);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
  GPIOPinConfigure(GPIO_PD0_I2C3SCL);
  GPIOPinConfigure(GPIO_PD1_I2C3SDA);
  GPIOPinTypeI2CSCL(GPIO_PORTD_BASE, GPIO_PIN_0);
  GPIOPinTypeI2C(GPIO_PORTD_BASE, GPIO_PIN_1);
  I2CMasterInitExpClk(I2C3_BASE, SysCtlClockGet(), false);
  //ativa pull up pelos pinos PB6 E PB7
  //na Launchpad, PB6 está conectado a PD0/I2C3SCL e PB7 a PD1/I2C3SDA
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
  GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_6);
  GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_7);
  GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_6, GPIO_STRENGTH_12MA,
                   GPIO_PIN_TYPE_STD_WPU);
  GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_7, GPIO_STRENGTH_12MA,
                   GPIO_PIN_TYPE_STD_WPU);
}

//transforma um número do formato decimal para BCD
unsigned char dec2bcd(unsigned char valor)
{
  return (((valor/10)<<4) | (valor%10));
}
//transforma um número do formato BCD para decimal
unsigned char bcd2dec(unsigned char valor)
{
  return (((valor&0xF0)>>4)*10) + (valor&0x0F);
}

void SetTime(char hora, char minuto, char segundo)
{
  uint8_t vet[3];
  vet[0]=dec2bcd(segundo);
  vet[1]=dec2bcd(minuto);
  vet[2]=dec2bcd(hora);
  I2CMasterSlaveAddrSet(I2C3_BASE, SLAVE_ADDR, false); //end. do dispositivo, R/S=0
  I2CMasterDataPut(I2C3_BASE, SEC); //end. inicial de gravação SEC=0x00
  I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_SEND_START); //start
  while(I2CMasterBusy(I2C3_BASE));
  for(char i=0; i<3; i++)
  {
    I2CMasterDataPut(I2C3_BASE, vet[i]);
    I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_SEND_CONT); //envia
    while(I2CMasterBusy(I2C3_BASE));
  }
  I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH); //stop
  while(I2CMasterBusy(I2C3_BASE));
}

void SetData(char dia, char mes, char ano)
{
  uint8_t vet[3];
  vet[0]=dec2bcd(dia);
  vet[1]=dec2bcd(mes);
  vet[2]=dec2bcd(ano);
  I2CMasterSlaveAddrSet(I2C3_BASE, SLAVE_ADDR, false); //end. do dispositivo, R/S=0
  I2CMasterDataPut(I2C3_BASE, DATE); //end. inicial de gravação DATE=0x04
  I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_SEND_START); //start
  while(I2CMasterBusy(I2C3_BASE));
  for(char i=0; i<3; i++)
  {
    I2CMasterDataPut(I2C3_BASE, vet[i]);
    I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_SEND_CONT); //envia
    while(I2CMasterBusy(I2C3_BASE));
  }
  I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH); //stop
  while(I2CMasterBusy(I2C3_BASE));
}

unsigned char GetData(unsigned char end)
{
  unsigned char dado;
  I2CMasterSlaveAddrSet(I2C3_BASE, SLAVE_ADDR, false); //R/S=0 (Send)
  I2CMasterDataPut(I2C3_BASE, end); //end. Memória RAM do RTC
  I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_SINGLE_SEND);
  while(I2CMasterBusy(I2C3_BASE));
  I2CMasterSlaveAddrSet(I2C3_BASE, SLAVE_ADDR, true); //R/S=1 (Read)
  I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE); //sel. recepção
  while(I2CMasterBusy(I2C3_BASE));
  dado=I2CMasterDataGet(I2C3_BASE); //leitura
  return bcd2dec(dado);
}
void loop(void)
{
  static bool ACTIVE = 0;
  unsigned short ADDR, DATA_ADDR = 0;//end do termo, end do valor a ser recuperado
  int valor=0,tamanho=0,flag=0,k=0, voltarponto=0,o=0,b=0;
  bool lixo;
  char valordisplay;
  Config_UART0 ();
  m=0;
  inteiro2=0,inteiro=0,inteiro1=0;
  inteiro2_decimal=0.0,valor_final=0.0;
  
  
  
  while(LocateOnBuffer("NEXT_VOLUME:", &ADDR, &DATA_ADDR )==0)
  {
    lixo=0; 
    
    
    if (LocateOnBuffer("VOLUME1:", &ADDR, &DATA_ADDR ))
    {
      
      ACTIVE = (UARTBUFF[DATA_ADDR] - 48);
      
      ClearOnBuffer("VOLUME1:",ADDR);
      
      if(ACTIVE==1){valor=1; lixo=1;}
    }
    else if (LocateOnBuffer("VOLUME2:", &ADDR, &DATA_ADDR ))
    {
      
      Config_UART0 ();  
      ACTIVE = (UARTBUFF[DATA_ADDR] - 48);
      ClearOnBuffer("VOLUME2:",ADDR); 
      if(ACTIVE==1){valor=2;lixo=1;}
    }
    else if (LocateOnBuffer("VOLUME3:", &ADDR, &DATA_ADDR ))
    {
      
      Config_UART0 (); 
      ACTIVE = (UARTBUFF[DATA_ADDR] - 48);
      ClearOnBuffer("VOLUME3:",ADDR); 
      if(ACTIVE==1){valor=3; lixo=1;}
      
    }
    else if (LocateOnBuffer("VOLUME4:", &ADDR, &DATA_ADDR ))
    {
      
      Config_UART0 (); 
      ACTIVE = (UARTBUFF[DATA_ADDR] - 48);
      ClearOnBuffer("VOLUME4:",ADDR);
      if(ACTIVE==1){valor=4; lixo=1;}
      
    }
    else if (LocateOnBuffer("VOLUME5:", &ADDR, &DATA_ADDR ))
    {
      
      Config_UART0 (); 
      ACTIVE = (UARTBUFF[DATA_ADDR] - 48);
      ClearOnBuffer("VOLUME5:",ADDR);
      if(ACTIVE==1){valor=5;  lixo=1;}
    }
    else if (LocateOnBuffer("VOLUME6:", &ADDR, &DATA_ADDR ))
    {
      
      Config_UART0 (); 
      
      ACTIVE = (UARTBUFF[DATA_ADDR] - 48);
      
      ClearOnBuffer("VOLUME6:",ADDR);
      
      if(ACTIVE==1){valor=6; lixo=1;}
      
    }
    else if (LocateOnBuffer("VOLUME7:", &ADDR, &DATA_ADDR ))
    {
      
      Config_UART0 (); 
      
      ACTIVE = (UARTBUFF[DATA_ADDR] - 48);
      
      ClearOnBuffer("VOLUME7:",ADDR);
      
      if(ACTIVE==1){valor=7;   lixo=1;}
      
    }
    else if (LocateOnBuffer("VOLUME8:", &ADDR, &DATA_ADDR ))
    {
      
      Config_UART0 (); 
      
      ACTIVE = (UARTBUFF[DATA_ADDR] - 48);
      
      ClearOnBuffer("VOLUME8:",ADDR);
      
      if(ACTIVE==1){valor=8; lixo=1;}
      
    }
    else if (LocateOnBuffer("VOLUME9:", &ADDR, &DATA_ADDR ))
    {
      
      Config_UART0 (); 
      
      ACTIVE = (UARTBUFF[DATA_ADDR] - 48);
      
      ClearOnBuffer("VOLUME9:",ADDR);
      
      if(ACTIVE==1){valor=9;;lixo=1;}
    }
    else if (LocateOnBuffer("VOLUME0:", &ADDR, &DATA_ADDR ))
    {
      
      Config_UART0 (); 
      
      ACTIVE = (UARTBUFF[DATA_ADDR] - 48);
      
      ClearOnBuffer("VOLUME0:",ADDR);
      
      if(ACTIVE==1){valor=0; lixo=1;}
      
    }
    else if (LocateOnBuffer("CLEAR:", &ADDR, &DATA_ADDR ))
    {
      
      Config_UART0 (); 
      
      ACTIVE = (UARTBUFF[DATA_ADDR] - 48);
      
      ClearOnBuffer("CLEAR:",ADDR);
      
      if((ACTIVE==1)&&(tamanho!=0))
      {
        if(o==0)inteiro/=10;
        if(o==1)inteiro1/=10;
        if(vetor_display[m]=='.')o=1; 
        
        tamanho--;
        
        for(int p=0; p<6; p++) UARTCharPut(UART7_BASE, vet2[p]);
        UARTCharPut(UART7_BASE, 0x3D);// Sinal de igual
        UARTCharPut(UART7_BASE, 0x22);  
        if(m==0)
        {
          m=0;
          flag=0;
          for(int p=0; p<m+1; p++)UARTCharPut(UART7_BASE,'0');
          UARTCharPut(UART7_BASE, 0x22); 
          UARTCharPut(UART7_BASE, 0xFF);
          UARTCharPut(UART7_BASE, 0xFF);
          UARTCharPut(UART7_BASE, 0xFF);
        }
        else 
        {
          k=0;
          m--;
          for(int p=0; p<m+1; p++)
          {
            UARTCharPut(UART7_BASE,vetor_display[p]);
            if(vetor_display[p]=='.')k=1;
          }
          UARTCharPut(UART7_BASE, 0x22); 
          UARTCharPut(UART7_BASE, 0xFF);
          UARTCharPut(UART7_BASE, 0xFF);
          UARTCharPut(UART7_BASE, 0xFF);
          if(valordisplay=='.'){k=0;valordisplay='0';}
          
        }
        lixo=0;
        
      }
    }
    else if (LocateOnBuffer("PONTO:", &ADDR, &DATA_ADDR ))
    {
      
      Config_UART0 ();  
      ACTIVE = (UARTBUFF[DATA_ADDR] - 48);
      ClearOnBuffer("PONTO:",ADDR);
      if(ACTIVE==1 && flag==1){valordisplay='.'; lixo=0;}
    }
    else if (LocateOnBuffer("BACK:", &ADDR, &DATA_ADDR ))
    {
      ACTIVE = (UARTBUFF[DATA_ADDR] - 48);
      ClearOnBuffer("BACK1:",ADDR);
      if(ACTIVE==1)
      {
        back=1;
        if(pagina==2 || pagina==5 || pagina==8 || pagina ==10)pagina=pagina-2;
        else{ pagina--;}
        
        
        break;
      }
    }
    
    if((valor>=0)&&(valor<=9)&&(lixo==1))
    {  
      // UARTprintf("%d",valor);
      valordisplay=valor+0x30;
      
      
      if(flag==0)
      {  
        vetor_display[m]=valordisplay;
        flag=1;
        for(int p=0; p<6; p++) UARTCharPut(UART7_BASE, vet2[p]);
        UARTCharPut(UART7_BASE, 0x3D);// Sinal de igual
        UARTCharPut(UART7_BASE, 0x22);  
        for(int p=0; p<m+1; p++)UARTCharPut(UART7_BASE, vetor_display[p]);
        UARTCharPut(UART7_BASE, 0x22); 
        UARTCharPut(UART7_BASE, 0xFF);
        UARTCharPut(UART7_BASE, 0xFF);
        UARTCharPut(UART7_BASE, 0xFF);
      }
      else
      {
        m++;
        vetor_display[m]=valordisplay;
        valores_display();
      }
      if(o==0)
      {
        inteiro=(10*inteiro)+(valor);
      }
      else if(o==1)
      {
        inteiro1=(10*inteiro1)+(valor);
      }
      tamanho++;
      
    }
    else if((valordisplay=='.') && (k==0))
    {
      voltarponto++;
      if(b==0)
      {
        inteiro1=inteiro;
        inteiro=0; 
      }
      else if(b==1)
      {
        o=0;
        inteiro=0; 
      }
      b=1;   
      k=1;
      m++;
      vetor_display[m]=valordisplay;
      
      for(int p=0; p<6; p++) UARTCharPut(UART7_BASE, vet2[p]);
      UARTCharPut(UART7_BASE, 0x3D);// Sinal de igual
      UARTCharPut(UART7_BASE, 0x22);  
      for(int p=0; p<m+1; p++)UARTCharPut(UART7_BASE, vetor_display[p]);
      UARTCharPut(UART7_BASE, 0x22); 
      UARTCharPut(UART7_BASE, 0xFF);
      UARTCharPut(UART7_BASE, 0xFF);
      UARTCharPut(UART7_BASE, 0xFF);
      tamanho++;
    }
  }
  ClearOnBuffer("NEXT_VOLUUME:",ADDR); 
  if(back==0)pagina++;
  
  
  if(voltarponto==0)
  {
    inteiro1=inteiro; 
    valor_final= (float)inteiro1;
  }
  else if(voltarponto!=0 && o==1)
  {
    valor_final= (float)inteiro1;
    
  }
  else if(voltarponto!=0 && o==0)
  {
    inteiro2=inteiro;
    inteiro2_decimal=(float)inteiro2;
    
    while(inteiro2!=0)
    {
      inteiro2_decimal= (inteiro2_decimal/10.00);
      inteiro2=(inteiro2/10);
    }
    
    valor_final= ((float)inteiro1)+inteiro2_decimal;
  }
  
  if(back==1)valor_final=0.0;
  back=0;
  //UARTprintf("\n");
  //print_float(valor_final);
  
}
void iniciar_display(void)
{
  static bool ACTIVE = 0;
  unsigned short ADDR, DATA_ADDR = 0;//end do termo, end do valor a ser recuperado
  
  while(LocateOnBuffer("N_INI:", &ADDR, &DATA_ADDR )==0);
  
  ClearOnBuffer("N_INI:",ADDR);   
  
}
void esperas(void)
{
  static bool ACTIVE = 0;
  unsigned short ADDR, DATA_ADDR = 0;//end do termo, end do valor a ser recuperado
  
  while(LocateOnBuffer("NEXT_INICIO:", &ADDR, &DATA_ADDR )==0);
  ClearOnBuffer("NEXT_INICIO:",ADDR);   
  
  pagina++;
  
}
void modo(void)
{
  static bool ACTIVE = 0;
  unsigned short ADDR, DATA_ADDR = 0;//end do termo, end do valor a ser recuperado
  int sair=0;
  
  while(sair==0)
  {
    if(LocateOnBuffer("BACK:", &ADDR, &DATA_ADDR))
    {
      ACTIVE = (UARTBUFF[DATA_ADDR] - 48);
      ClearOnBuffer("BACK:",ADDR);
      if(ACTIVE==1){pagina=0;sair=1;}
    }
    else if(LocateOnBuffer("INTENSIVO:", &ADDR, &DATA_ADDR))
    {
      ACTIVE = (UARTBUFF[DATA_ADDR] - 48);
      ClearOnBuffer("INTENSIVO:",ADDR);
      if(ACTIVE==1){MODO=1;sair=1;}
    }
    else if(LocateOnBuffer("MODERADO:", &ADDR, &DATA_ADDR))
    {
      ACTIVE = (UARTBUFF[DATA_ADDR] - 48);
      ClearOnBuffer("MODERADO:",ADDR);
      if(ACTIVE==1){MODO=2;sair=1;}
    }
  }
}
void progress_bar(float PH)
{
  
  char vet_pagPH[8]={"page 15"};
  for(int p=0; p<7; p++) UARTCharPut(UART7_BASE, vet_pagPH[p]);
  UARTCharPut(UART7_BASE, 0xFF);
  UARTCharPut(UART7_BASE, 0xFF);
  UARTCharPut(UART7_BASE, 0xFF);
  if(PH<6.2){NexTrocaValor("100", "j15.val");}
  else if(PH>=6.2 && PH<6.3){NexTrocaValor("100", "j0.val");}
  else if(PH>=6.3 && PH<6.4){NexTrocaValor("100", "j1.val");}
  else if(PH>=6.4 && PH<6.5){NexTrocaValor("100", "j2.val");}
  else if(PH>=6.5 && PH<6.6){NexTrocaValor("100", "j3.val");}
  else if(PH>=6.6 && PH<6.7){NexTrocaValor("100", "j4.val");}
  else if(PH>=6.7 && PH<6.8){NexTrocaValor("100", "j5.val");}
  else if(PH>=6.8 && PH<6.9){NexTrocaValor("100", "j6.val");}
  else if(PH>=6.9 && PH<=7.0){NexTrocaValor("100", "j7.val");}
  else if(PH>7.0 && PH<7.4){NexTrocaValor("100", "j8.val");}
  else if(PH>=7.4 && PH<7.5){NexTrocaValor("100", "j9.val");}
  else if(PH>=7.5 && PH<7.6){NexTrocaValor("100", "j10.val");}
  else if(PH>=7.6 && PH<7.7){NexTrocaValor("100", "j11.val");}
  else if(PH>=7.7 && PH<7.8){NexTrocaValor("100", "j12.val");}
  else if(PH>=7.8 && PH<7.9){NexTrocaValor("100", "j13.val");}
  else if(PH>=7.9 && PH<=8.0){NexTrocaValor("100", "j14.val");}
  else if(PH>8.0){NexTrocaValor("100", "j16.val");}
  NexTrocaLegenda("Processo finalizado...","t17.txt");
  
  SysCtlDelay(30000000); //delay sensor pH
}
void DOSAGEM(float dosagem)
{
  char vetor[10]={' ',' ',' ',' ',' '};
  char vet_DOSA[7]={"t1.txt"};
  
  int x1=0,x2=0,x3=0;
  
  x1=(int)dosagem;
  
  if(x1!= 0)
  {
    x2=(dosagem*10)-(((int)dosagem)*10);
    x2=fabs(x2);
    x3=(dosagem*100)-(((int)dosagem)*100);
    x3=fabs(x3)%10;
    UARTprintf("%d.%d%d",x1,x2,x3); 
    
    
    int Dig=count(x1);
    
    while (Dig!=0) 
    {
      vetor[Dig]=((x1%10)+0x30);
      x1/=10;
      Dig--;
    }
    Dig=count((int)dosagem);
    vetor[Dig+1]='.';
    vetor[Dig+2]=x2+0x30;
    vetor[Dig+3]=x3+0x30;
  }
  else
  { 
    x1=0;
    x2=(dosagem*10);
    x2=fabs(x2);
    x3=(dosagem*100)-(int)dosagem*100;
    x3=fabs(x3);
    if(dosagem<0)
    {
      UARTprintf("-%d.%d%d",x1,x2,x3);
      vetor[0]='0';
    }
    else
    {
      UARTprintf("%d.%d%d",x1,x2,x3);
      for(int i=0;i<4;i++)
      {
        if(i==0)vetor[i]=x1+0x30;
        if(i==1)vetor[i]='.';
        if(i==2)vetor[i]=x2+0x30;
        if(i==3)vetor[i]=x3+0x30;
      }
    }
  }
  for(int p=0; p<6; p++) UARTCharPut(UART7_BASE,vet_DOSA[p]);
  UARTCharPut(UART7_BASE, 0x3D);// Sinal de igual
  UARTCharPut(UART7_BASE, 0x22);  
  for(int p=0; p<9; p++)UARTCharPut(UART7_BASE,  vetor[p]);
  UARTCharPut(UART7_BASE, 0x22); 
  UARTCharPut(UART7_BASE, 0xFF);
  UARTCharPut(UART7_BASE, 0xFF);
  UARTCharPut(UART7_BASE, 0xFF);
  
  
}
void mostrar_PH(float PH_disp)
{
  
  
  char PH_C[6]={' ',' ',' ',' ',' ',' '};
  
  char vet_PH[7]={"t0.txt"};
  if(PH_disp > 7.0 && PH_disp < 7.4) 
  {
    
    char vet_pag[8]={"page 16"};
    for(int p=0; p<7; p++) UARTCharPut(UART7_BASE, vet_pag[p]);
    UARTCharPut(UART7_BASE, 0xFF);
    UARTCharPut(UART7_BASE, 0xFF);
    UARTCharPut(UART7_BASE, 0xFF);
    NexTrocaLegenda("pH está na região de idealidade!","t7.txt");
    NexTrocaLegenda("Aplicação de Cloro...","t1.txt");
    NexTrocaLegenda("PRÓXIMA ETAPA - Neutralização","t2.txt");
    
  }
  else 
  {
    
    char vet_pag[8]={"page 17"};
    for(int p=0; p<7; p++) UARTCharPut(UART7_BASE, vet_pag[p]);
    UARTCharPut(UART7_BASE, 0xFF);
    UARTCharPut(UART7_BASE, 0xFF);
    UARTCharPut(UART7_BASE, 0xFF);
    
    if(PH_disp >= 7.4)
    {
      NexTrocaLegenda("Aplicação de REDUTOR","t2.txt");
      NexTrocaLegenda("Água em condição de ALCALINIDADE!","t7.txt");
    }
    
    else if(PH_disp <= 7.0)
    {
      NexTrocaLegenda("Aplicação de ELEVADOR","t2.txt");
      NexTrocaLegenda("Água em condição de ACIDEZ!","t7.txt");
      
    }
    NexTrocaLegenda("Dosagem em andamento...","t8.txt");
  }
  char vetor_ph[6]={' ',' ',' ',' ',' ',' '};
  int x1=0,x2=0,x3=0;
  float vava=PH_disp;
  x1=(int)vava;
  
  if(x1!= 0)
  {
    x2=(vava*10)-(((int)vava)*10);
    x2=fabs(x2);
    x3=(vava*100)-(((int)vava)*100);
    x3=fabs(x3)%10;
    UARTprintf("%d.%d%d",x1,x2,x3); 
    
    
    int Dig=count(x1);
    
    while (Dig!=0) 
    {
      vetor_ph[Dig]=((x1%10)+0x30);
      x1/=10;
      Dig--;
    }
    Dig=count((int)vava);
    vetor_ph[Dig+1]='.';
    vetor_ph[Dig+2]=x2+0x30;
    vetor_ph[Dig+3]=x3+0x30;
  }
  else
  { 
    x1=0;
    x2=(vava*10);
    x2=fabs(x2);
    x3=(vava*100)-(int)vava*100;
    x3=fabs(x3);
    if(vava<0)
    {
      UARTprintf("-%d.%d%d",x1,x2,x3);
      vetor_ph[0]='0';
    }
    else
    {
      UARTprintf("%d.%d%d",x1,x2,x3);
      for(int i=0;i<4;i++)
      {
        if(i==0)vetor_ph[i]=x1+0x30;
        if(i==1)vetor_ph[i]='.';
        if(i==2)vetor_ph[i]=x2+0x30;
        if(i==3)vetor_ph[i]=x3+0x30;
      }
    }
  }
  UARTprintf("\n\n%s",vetor_ph);
  for(int p=0; p<6; p++)UARTCharPut(UART7_BASE,vet_PH[p]);
  UARTCharPut(UART7_BASE, 0x3D);// Sinal de igual
  UARTCharPut(UART7_BASE, 0x22);  
  for(int p=0; p<5; p++)UARTCharPut(UART7_BASE, vetor_ph[p]);  
  UARTCharPut(UART7_BASE, 0x22); 
  UARTCharPut(UART7_BASE, 0xFF);
  UARTCharPut(UART7_BASE, 0xFF);
  UARTCharPut(UART7_BASE, 0xFF);
  
  
}
void print_RTC(float PH_RTC, unsigned char dia,unsigned char mes,unsigned char ano,unsigned char hora, unsigned char min,unsigned char seg)
{     
  
  if(pag==0)
  {
    char vet_pag[8]={"page 18"};
    for(int p=0; p<7; p++) UARTCharPut(UART7_BASE, vet_pag[p]);
    UARTCharPut(UART7_BASE, 0xFF);
    UARTCharPut(UART7_BASE, 0xFF);
    UARTCharPut(UART7_BASE, 0xFF);
    
    if(sai==0)
    {
      if(PH_RTC > 7.0 && PH_RTC < 7.4 && (MODO==1)){NexTrocaLegenda("Nova verificação","t1.txt"); NexTrocaLegenda("em 2 dias...","t0.txt");}  
      else if(PH_RTC> 7.0 && PH_RTC < 7.4 && (MODO==2)){NexTrocaLegenda("Nova verificação","t1.txt");NexTrocaLegenda("em 7 dias...","t0.txt");}  
      else if(PH_RTC<=7.0 || PH_RTC>=7.4){ NexTrocaLegenda("Nova verificação","t1.txt");NexTrocaLegenda("em 2 horas...","t0.txt");} 
      
    }
    else if(sai==1){NexTrocaLegenda("Nova verificação","t1.txt");NexTrocaLegenda("em 1 dia...","t0.txt");}  
  }
  pag++;
  char vet_rtc[7] = {"t8.txt"};
  int valores_rtc[20];
  for(int p=0; p<6; p++) UARTCharPut(UART7_BASE, vet_rtc[p]);
  UARTCharPut(UART7_BASE, 0x3D);// Sinal de igual
  UARTCharPut(UART7_BASE, 0x22);  
  
  int c_dia,c_mes,c_ano,c_hora,c_min,c_seg;
  c_dia=dia;
  
  UARTprintf("%d",c_dia);
  int i=0;
  
  c_dia=dia/10;
  valores_rtc[i]=c_dia;
  i++;
  c_dia=dia%10;
  valores_rtc[i]=c_dia;
  i++;
  valores_rtc[i]='/'-48;
  i++;
  c_mes=mes/10;
  valores_rtc[i]=c_mes;
  i++;
  c_mes=mes%10;
  valores_rtc[i]=c_mes;
  i++;
  valores_rtc[i]='/'-48;
  i++;
  c_ano=ano/10;
  valores_rtc[i]=c_ano;
  i++;
  c_ano=ano%10;
  valores_rtc[i]=c_ano;
  
  
  i++;  valores_rtc[i]=' '-48;
  i++;valores_rtc[i]=' '-48;
  i++;valores_rtc[i]=' '-48;
  i++;
  //horario
  c_hora=hora/10;
  valores_rtc[i]=c_hora;
  i++;
  c_hora=hora%10;
  valores_rtc[i]=c_hora;
  i++;
  valores_rtc[i]=':'-48;
  i++;
  c_min=min/10;
  valores_rtc[i]=c_min;
  i++;
  c_min=min%10;
  valores_rtc[i]=c_min;
  i++;
  valores_rtc[i]=':'-48;
  i++;
  c_seg=seg/10;
  valores_rtc[i]=c_seg;
  i++;
  c_seg=seg%10;
  valores_rtc[i]=c_seg;
  
  
  for(int p=0; p<19; p++){UARTCharPut(UART7_BASE,(valores_rtc[p]+0x30));}
  UARTCharPut(UART7_BASE, 0x22); 
  UARTCharPut(UART7_BASE, 0xFF);
  UARTCharPut(UART7_BASE, 0xFF);
  UARTCharPut(UART7_BASE, 0xFF);
}
int alterarCONF ()
{
  UART7_IntHandler();
  static bool ACTIVE = 0;
  unsigned short ADDR, DATA_ADDR = 0;//end do termo, end do valor a ser recuperado
  int alteracao=0;
  
  if(LocateOnBuffer("MODO:", &ADDR, &DATA_ADDR)==1)
  {
    ACTIVE = (UARTBUFF[DATA_ADDR] - 48);
    ClearOnBuffer("MODO:",ADDR);
    if(ACTIVE==1)
    {
      
      alteracao=1;MODO=0;pagina=12;nao_modo=0;
      char vet_pag[8]={"page 12"};
      for(int p=0; p<7; p++) UARTCharPut(UART7_BASE, vet_pag[p]);
      UARTCharPut(UART7_BASE, 0xFF);
      UARTCharPut(UART7_BASE, 0xFF);
      UARTCharPut(UART7_BASE, 0xFF);
    }
  }
  else if(LocateOnBuffer("DOSAGENS:", &ADDR, &DATA_ADDR)==1)
  {
    
    ACTIVE = (UARTBUFF[DATA_ADDR] - 48);
    ClearOnBuffer("DOSAGENS:",ADDR);
    if(ACTIVE==1)
    {
      modotemporario=MODO;
      alteracao=1;MODO=0;pagina=0;nao_modo=1;
      char vet_pag[7]={"page 1"};
      for(int p=0; p<6; p++) UARTCharPut(UART7_BASE, vet_pag[p]);
      UARTCharPut(UART7_BASE, 0xFF);
      UARTCharPut(UART7_BASE, 0xFF);
      UARTCharPut(UART7_BASE, 0xFF);
    }
  }
  else if( begin==1)alteracao=1;
  return alteracao;
}
void menu (void)
{
  volume_piscina, DOSAGEM_REDUTOR1,DOSAGEM_REDUTOR2,DOSAGEM_ELEVADOR1,DOSAGEM_ELEVADOR2, DOSAGEM_CLORO, DOSAGEM_ALGICIDA=0.0;
  
  UART7_IntHandler();
  
  while(MODO==0)
  {
    while(pagina<11)
    {
      switch( pagina )
      {
        
      case 0:
        loop();
        volume_piscina=valor_final;
        UARTprintf("\n");
        print_float( volume_piscina);
        break;
      case 1:
        esperas();
        break;
      case 2:
        loop();
        DOSAGEM_REDUTOR1=valor_final;
        UARTprintf("\n");
        print_float( DOSAGEM_REDUTOR1);
        break;
      case 3:
        loop();
        DOSAGEM_REDUTOR2=valor_final;
        UARTprintf("\n");
        print_float(DOSAGEM_REDUTOR2);
        break;
      case 4:
        esperas();
        break;
      case 5:
        loop();
        DOSAGEM_ELEVADOR1=valor_final;
        UARTprintf("\n");
        print_float(DOSAGEM_ELEVADOR1);
        break;
      case 6:
        loop();
        DOSAGEM_ELEVADOR2=valor_final;
        UARTprintf("\n");
        print_float(DOSAGEM_ELEVADOR2);
        break; 
      case 7:
        esperas();
        break;
      case 8:
        loop();
        DOSAGEM_CLORO=valor_final;
        UARTprintf("\n");
        print_float(DOSAGEM_CLORO);
        break;
      case 9:
        esperas();
        break;
      case 10:
        loop();
        DOSAGEM_ALGICIDA=valor_final;
        UARTprintf("\n");
        print_float(DOSAGEM_ALGICIDA);
        break;
      }
    }
    if(nao_modo==0)
    {
      char vet_pag[8]={"page 12"};
      for(int p=0; p<7; p++) UARTCharPut(UART7_BASE, vet_pag[p]);
      UARTCharPut(UART7_BASE, 0xFF);
      UARTCharPut(UART7_BASE, 0xFF);
      UARTCharPut(UART7_BASE, 0xFF); 
      modo();
      
    }
    else if(nao_modo==1)MODO=modotemporario;
  }
}
void niveis(void)
{  
  int n=0,alteracao_elev,alteracao_red,alteracao_alg,alteracao_cl=0;   
  char vet_ni[8]={"page 14"};
  for(int p=0; p<7; p++) UARTCharPut(UART7_BASE, vet_ni[p]);
  UARTCharPut(UART7_BASE, 0xFF);
  UARTCharPut(UART7_BASE, 0xFF);
  UARTCharPut(UART7_BASE, 0xFF); 
  
  
  do
  {
    if(BOIA_ELEV!=0x00)
    {
      alteracao_elev=1;
      UARTprintf("\rnivel ABAIXO-ELEV");
      NexTrocaValor("1","b1.val");  
    }
    else{alteracao_elev=0;  NexTrocaValor("0","b1.val");}
    
    if(BOIA_RED!=0x00)
    {
      alteracao_red=1;
      UARTprintf("\n\rnivel ABAIXO-RED");
      NexTrocaValor("1","b2.val");  
    }
    else{ alteracao_red=0;  NexTrocaValor("0","b2.val");  }
    
    if(BOIA_ALG!=0x00)
    {
      alteracao_alg=1;
      UARTprintf("\n\rnivel ABAIXO-ALG");
      NexTrocaValor("1","b3.val");  
    } 
    else{alteracao_alg=0;  NexTrocaValor("0","b3.val");  }
    
    if(BOIA_CL!=0x00) 
    {
      alteracao_cl=1;
      UARTprintf("\n\rnivel ABAIXO-CL");
      NexTrocaValor("1","b0.val");  
    }
    else{alteracao_cl=0;  NexTrocaValor("0","b0.val");  }
    
    SysCtlPWMClockSet(SYSCTL_PWMDIV_16); //clock do PWM
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); //PF2 e PF3
    GPIOPinConfigure(GPIO_PF2_M1PWM6 ); //PWM0 saída 0
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);
    PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN);
    //Gerador 0 para saídas 0 e 1
    
    
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, 1000); //Período ? máx. (2^16)
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, 500); //tH da saída 6
    PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT, true); //habilita a saída 6 do PWM0 (PF2).
    PWMGenEnable(PWM1_BASE, PWM_GEN_3); //habilita o gerador 3.
    SysCtlDelay(3000000);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, 600); //Período ? máx. (2^16)
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, 300); //tH da saída 6
    PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT, true); //habilita a saída 6 do PWM0 (PF2).
    PWMGenEnable(PWM1_BASE, PWM_GEN_3); //habilita o gerador 3.
    SysCtlDelay(3000000);
    PWMGenDisable(PWM1_BASE, PWM_GEN_3); //habilita o gerador 3.
    SysCtlDelay(3000000);
  }
  while(alteracao_elev==1 || alteracao_red==1 || alteracao_alg==1 || alteracao_cl==1);
  
  UARTprintf("Todos os produtos estão disponíveis...");
}
void nivel_central(void)
{
  int encheu=0;
  char vet_nic[8]={"page 13"};
  for(int p=0; p<7; p++) UARTCharPut(UART7_BASE, vet_nic[p]);
  UARTCharPut(UART7_BASE, 0xFF);
  UARTCharPut(UART7_BASE, 0xFF);
  UARTCharPut(UART7_BASE, 0xFF); 
  
  do
  {
    if(BOIA_CENTRAL!=0x00) 
    {
      UARTprintf("oi");
      encheu=0;UARTprintf("eei");  NexTrocaValor("0","b0.val"); NexTrocaLegenda("Reservatório está recebendo água...","t3.txt");
    }
    else { NexTrocaValor("1","b0.val");  
    NexTrocaLegenda("Nível necessário atingido!","t3.txt");
    encheu=1;  }
  }while(encheu==0);
  SysCtlDelay(30000000);
}

