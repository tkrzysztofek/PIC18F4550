//PIC18F4550

#define _XTAL_FREQ 8000000

#define RS PORTCbits.RC0
#define EN PORTCbits.RC1
#define D4 PORTDbits.RD0
#define D5 PORTDbits.RD1
#define D6 PORTDbits.RD2
#define D7 PORTDbits.RD3

#include <xc.h>
#include "lcd.h"
#include <stdio.h>


// BEGIN CONFIG
#pragma config WDT = OFF, BOR = ON, BORV = 0, LVP = OFF
#pragma config FOSC = HS

int delayTime = 0;
unsigned short sec = 0;
unsigned short cycle1,cycle2,cycle3;

void interrupt isr()
{
    INTCONbits.T0IF = 0;        //Clear the Timer 0 interrupt flag
    TMR0 = 65036;   ;   //Load the starting value back into the timer

    delayTime++;

    if(delayTime >= 1000)
    {
        delayTime = 0;
        //We reach here when 6.5 seconds has elapsed.
        //Here you can put in the code for whatever is supposed to happen
        //after 6.5 seconds. In this case, toggle the LED.
        sec++;
        cycle1 = 1; 
        cycle2 = 1;
        cycle3 = 1;
        //Lcd_Clear();
    }
}

/*void pwm(float U_pwm){
    CCP1CON = 0b00001100;   // Enable PWM on CCP1
    T2CON = 0b00000100;     // Enable TMR2 with prescaler = 1
    
    PR2 = 100-1;    // PWM period = (PR2+1) * prescaler * Tcy = 1ms
    CCPR1L = U_pwm*100/5; // pulse width = CCPR1L * prescaler * Tcy = 100us
    }*/

//EEPROM
void ee_write_byte(unsigned char address, int wartosc){

    EEDATA = wartosc;
    EEADR = address;
    // start write sequence as described in datasheet, page 91
    EECON1bits.EEPGD = 0;
    EECON1bits.CFGS = 0;
    EECON1bits.WREN = 1; // enable writes to data EEPROM
    INTCONbits.GIE = 0;  // disable interrupts
    EECON2 = 0x55;
    EECON2 = 0x0AA;
    EECON1bits.WR = 1;   // start writing
       
    EECON1bits.WREN = 0;
    INTCONbits.GIE = 1;  // enable interrupts
}

float ee_read_byte(unsigned char address){
    EEADR = address;
    EECON1bits.CFGS = 0;
    EECON1bits.EEPGD = 0;
    EECON1bits.RD = 1;
    //wartosc = EEDATA;
    return EEDATA;
}

//ADC
float adc(int bity){
       
  ADCON1bits.VCFG = 0;      //REFERENCE VDD
  ADCON2bits.ADFM = 1;      //ADC result is right justified
  ADCON2bits.ADCS = 0b001;  //A/D Conversion Clock  Fosc/8
  ADCON1bits.PCFG = 0b1101; //WYBOR PINOW
    
  ADCON0bits.CHS0 = bity;        //WYBOR KANALU
  ADCON0bits.ADON = 1;           //Turn on the ADC
     __delay_us(5);              //Wait the acquisition time (about 5us).
     ADCON0bits.GO = 1;          //start the conversion
     while(ADCON0bits.GO==1){};  //wait for the conversion to end

        int result = (ADRESH<<8)+ADRESL;	//combine the 10 bits of the conversion
        return (result*3.3)/1023;        
}


int main(){
      
  TRISD = 0x00;     // OUT
  TRISC = 0x00;
  TRISB = 0xFF;     // IN
  TRISA = 0xFF;  
  
  
     ///////////////////
    // Timer 0 Setup //
   ///////////////////
  
    T0CONbits.T08BIT = 0; //TIMER 16-BIT
    
    //OPTION_REGbits.PSA = 0; //Prescaler assigned to Timer 0 (other option is to
                             //the Watchdog timer (WDT))
    T0CONbits.PSA = 0;
            
    //OPTION_REGbits.PS = 0b001;  //Set the prescaler to 1:4
    T0CONbits.T0PS = 0b001;
    
    //OPTION_REGbits.T0CS = 0;  //Use the instruction clock (Fcy/4) as the timer
                                //clock. Other option is an external oscillator
                                //or clock on the T0CKI pin.
    T0CONbits.T0CS = 0;
            
    INTCONbits.T0IF = 0;        //Clear the Timer 0 interrupt flag
    TMR0 = 65036;               //Load the starting value into the timer
    INTCONbits.T0IE = 1;        //Enable the Timer 0 interrupt    
    INTCONbits.GIE = 1;         //set the Global Interrupt Enable
    
    
  char s[20];
  float U,I;
  int r;
  float P,E;
  float max=0;
  int rozpocznij; 
  int reset;
  
  Lcd_Init();  
  Lcd_Clear();
  
      //stop = ee_read_byte(0x00);
      //max = ee_read_byte(0x10);
      //E = ee_read_byte(0x20);
    
  while(1){
           
      if(cycle1==1){
        U = adc(0b000);    
     
        // Ustalamy tryb dzien/noc
        if(U*2>max){ 
            rozpocznij++;            
            if(rozpocznij>20) rozpocznij=20;
        }
        else{
            rozpocznij--;            
            if(rozpocznij<0) rozpocznij=0;
        }
        cycle1=0;
      }
      
    /////////////// DZIEN //////////////////
        if(rozpocznij > 11){ 
            
            if(reset==1){
                E = 0;
                max = 0;
                reset=0;
            }
                
        
         if(cycle2==1){
             I = adc(0b001);
             P=U*I;
             E = E + P;
             if(U > max) max = U;
             cycle2 = 0;             
         }      
                  
         sprintf(s, "U=%.2fV Um=%.2fV", U,max);    
         Lcd_Set_Cursor(1,1);
         Lcd_Write_String(s);
            
         sprintf(s, "P=%.2fW E=%.2f", P,E);
         Lcd_Set_Cursor(2,1);
         Lcd_Write_String(s);
        }
     
    /////////////// NOC //////////////////
        else if(rozpocznij < 9){ 
                              
         sprintf(s, "Wyprodukowano:" );    
         Lcd_Set_Cursor(1,1);
         Lcd_Write_String(s);
            
         sprintf(s, "E=%.2fV Ws", E);
         Lcd_Set_Cursor(2,1);
         Lcd_Write_String(s);
         
         reset=1;
      
      }
      
    /////////// Zmiana trybu ////////////////
        else{
         sprintf(s, "Zmiana trybu...");    
         Lcd_Set_Cursor(1,1);
         Lcd_Write_String(s);        
         
        }
         
    /*
     
     if(wystartowanie > 10) stop=0;
                            
     if(zatrzymanie>10){
         stop=1;
         if(ee_read_byte(0x00)!=stop) ee_write_byte(0x00, stop); 
         if(ee_read_byte(0x10)!=max) ee_write_byte(0x10, max);
         if(ee_read_byte(0x20)!=E) ee_write_byte(0x20, E);
     }
     
     else stop=0;      
         
    if(stop==1){             
     
    }
     
    else{    
     if(cycle1 == 1){         
        E = E + P;      
        if(U > max) max = U;
        if(U*2 < max) zatrzymanie++;
        else zatrzymanie = 0;        
        cycle1 = 0;
     }       
    
    }    
         
    //pwm(4.67); 
        */    
    r++;
    if(r>25){
        Lcd_Clear();
        r=0;
    }       
                  
  }
    
  return 0;
}