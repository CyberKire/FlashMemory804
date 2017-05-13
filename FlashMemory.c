#include "xc.h"

//Configuration Register - FOSC
#pragma config FCKSM = 3    //both cock switching and fail-safe modes are disabled
#pragma config OSCIOFNC = 0//OSCO pin is a general purpose I/O pin
#pragma config POSCMD = 3   //primary oscillators are disabled

//Configuration Register - FOSCSEL
#pragma config IESO = 0     //start with a user-selected oscillator at reset
#pragma config FNOSC = 7    //slect FRC oscillator with postscalar at reset

//Configuration Register - FICD
#pragma config JTAGEN = 0   //JTAG is disabled

#define PPSUnLock   __builtin_write_OSCCONL(OSCCON & 0xBF)
#define PPSLock     __builtin_write_OSCCONL(OSCCON | 0x40)

unsigned char SPI_Transmit(unsigned char TxValue)
{
    while(SPI1STATbits.SPITBF==1);//Wait until the TX buffer is empty due to a prior process
    SPI1BUF = TxValue;            //When empty, send the byte to the TX buffer
    while(SPI1STATbits.SPIRBF==0);//As valid bits shifts out of SDO, junk bits are received from SDI
                                  //Wait until the RX buffer is full of junk data
    return SPI1BUF;               //When full, read the junk data in RX buffer through SPI1BUF
}

unsigned char SPI_Receive()
{
    while(SPI1STATbits.SPITBF==1);//Wait until the TX buffer is empty due to a prior process
    SPI1BUF = 0xBB;               //When empty, send the junk byte (0x00) to the TX buffer
    while(SPI1STATbits.SPIRBF==0);//As junk bits shifts out of SDO, valid bits are received from the SDI
                                  //Wait until the RX buffer is full of valid data
    return SPI1BUF;               //When full, read the valid data in RX buffer through SPI1BUF
}

void WREN()
{
    LATBbits.LATB5 = 0; //Lower SS for instruction delivery
    SPI_Transmit(0x06); //Send the WREN instruction
    LATBbits.LATB5 = 1; //Raise SS for instruction completion
}

unsigned char Read_Status_Reg()
{
    LATBbits.LATB5 = 0;                  //Lower SS for instruction delivery
    SPI_Transmit(0x05);                  //Send the Read status register instruction
    unsigned char status = SPI_Receive();//Read the status register
    LATBbits.LATB5 = 1;                  //Raise SS for instruction completion
    return status;
}

unsigned char Write_Byte (unsigned char command, long int address, unsigned char SentByte)
{
    LATBbits.LATB5 = 0;                  //Lower SS to start PP
    SPI_Transmit(command);               //Send the PP instruction
    SPI_Transmit((address >> 16) & 0xFF);//Send the  MS byte of the 24-bit address
    SPI_Transmit((address >> 8) & 0xFF); //Send the mid byte of the 24-bit address
    SPI_Transmit(address & 0xFF);        //Send the LS byte of the 24-bit address
    SPI_Transmit(SentByte);
    unsigned char JunkByte = SPI_Receive();
    LATBbits.LATB5 = 1;                  //Raise SS for instruction completion
    return JunkByte;
}

unsigned char Read_Byte (unsigned char command, long int address)
{
    LATBbits.LATB5 = 0;                  //Lower SS to start PP
    SPI_Transmit(command);               //Send the PP instruction
    SPI_Transmit((address >> 16) & 0xFF);//Send the MS byte of the 24-bit address
    SPI_Transmit((address >> 8) & 0xFF); //Send the mid byte of the 24-bit address
    SPI_Transmit(address & 0xFF);        //Send the LS byte of the 24-bit address
    unsigned char ReceivedByte = SPI_Receive();
    LATBbits.LATB5 = 1;                  //Raise SS for instruction completion
    return ReceivedByte;
}

unsigned int limit1, limit2;
void delay(limit1,limit2)
{
    unsigned int i,j;
    for(i=0;i<limit1;i++)
    {
        for(j=0;j<limit2;j++);
    }
}


void main()
{
    //Clock source definition - A single FRC internal clock
    //OSCTUN Register
    OSCTUNbits.TUN = 0;     //select FRC = 7.37MHz
    
    //CLKDIV Register
    CLKDIVbits.ROI = 0;     //interrupts have no effect on clock recovery
    CLKDIVbits.FRCDIV = 3;  //FOSC=FRC/8=7.37MHz/8 and FP=FOSC/2=460KHz
    
    //SPI configuration MASTER mode sending 8 bits
    SPI1CON1bits.DISSCK = 0;//Enable the internal SPI clock
    SPI1CON1bits.DISSDO = 0;//Enable the SPI data output, SDO
    SPI1CON1bits.MODE16 = 0;//Enable the 8-bit data mode
    SPI1CON1bits.SSEN = 0;  //This unit is not a slave so Slave Select pin is not used
    SPI1CON1bits.MSTEN = 1; //Enable MASTER mode
    SPI1CON1bits.SMP = 0;   //Sample data at he pos edge when received at the neg edge of SCK
    SPI1CON1bits.CKE = 1;   //Output data changes when SCK goes from ACTIVE to IDLE state
    SPI1CON1bits.CKP = 0;   //SCK polarity: IDLE is low pase and ACTIVE is high phase of SCK
    SPI1CON1bits.PPRE = 1;  //Primary SPI clock pre-scale is 1:1
    SPI1CON1bits.SPRE = 7;  //Secondary SPI clock pre-scale is 1:1 -> SCK = 460KHz
    SPI1STATbits.SPIROV = 0;//Clear initial overflow bit in case an overflow condition in SPI1BUF
    SPI1STATbits.SPIEN = 1; //Enable the SPI interface
    
    //Peripheral Pin Select with RP pins
    PPSUnLock;
    RPOR4bits.RP8R = 8;     //RP8 is an output for SCK1
    RPINR20bits.SCK1R = 8;  //RP8 is an input for SCK1
    RPOR4bits.RP9R = 7;     //RP9 is an output for SDO1
    RPINR20bits.SDI1R = 7;  //RP7 is an input for SDI1
    //RPOR3bits.RP6R = 9;     //RP6 is an output for SS1
    RPOR2bits.RP5R = 9;     //RP5 is an output for SS1
    PPSLock;
    
    //Define I/O
    //TRISBbits.TRISB6 = 0;   //Configure RB6 as an output for SS
    TRISBbits.TRISB5 = 0;   //Configure RB5 as an output for SS
    TRISBbits.TRISB8 = 0;   //Configure RB8 as an output for SCK1
    TRISBbits.TRISB9 = 0;   //Configure RB9 as an output SDO1
    TRISBbits.TRISB7 = 1;   //Configure RB7 as an input for SD1
    
    AD1PCFGL=0xFFFF;    //Pin RA0, RA1, RA2, RA3 are assigned to be digital I/O pins
    TRISAbits.TRISA0=1; //Pin RA0 is assigned as input for record
    TRISAbits.TRISA1=1; //Pin RA1 is assigned as input for play
    TRISAbits.TRISA2=0; //Pin RA2 is assigned as output for Red LED
    TRISAbits.TRISA3=0; //Pin RA3 is assigned as output for Green LED
    TRISAbits.TRISA4=1; //Pin RA4 is assigned as input for erase
    TRISAbits.TRISA8=0; //Pin RA8 is assigned as input for Yellow LED
    
    
    while(1)
    {
        //Press erase button and wait for LED to turn off
        //When the LED goes from ON to OFF, this indicates that the memory is cleared
        if(PORTAbits.RA4==1)    //Erase button is pressed
        {
            //Process to clear FLASH memory
            LATAbits.LATA8=1;   //Yellow LED on
            LATBbits.LATB5=0;  //Lower SS for instruction delivery
            SPI_Transmit(0x50);//Enable-Write_Status-Register opcode
            LATBbits.LATB5=1;  //Raise SS for instruction completion
        
            LATBbits.LATB5=0;  //Lower SS for instruction delivery
            SPI_Transmit(0x01);//Write-Status-Register opcode
            SPI_Transmit(0x00);//Status Register:0x00
                               //Must set BP2,BP1,BP0 to 0 in order to take off write protect
            LATBbits.LATB5=1;
    
            LATBbits.LATB5=0;
            SPI_Transmit(0x05);//Read status register Opcode
            SPI_Receive();     //Read status register
            LATBbits.LATB5=1;
    

            WREN();
            LATBbits.LATB5=0;
            SPI_Transmit(0xC7);//Opcode for chip erase
            LATBbits.LATB5=1;
    
            while((Read_Status_Reg() & 0x01) == 0x01);//Wait for memory to be clear
    
            Read_Byte(0x03,0x020000);//Check when data is 0xFF
    
            Read_Status_Reg();            
            delay(100,1000);
            LATAbits.LATA8=0;   //Yellow LED off
            LATAbits.LATA2=1;   //Red LED on
            LATAbits.LATA3=1;   //Green LED on
            delay(100,100);
            
            WREN();
            while((Read_Status_Reg() & 0x02)==0x00);//While WEL=0 keep reading the status register Read_Status_Reg();
            LATBbits.LATB5=0;
            SPI_Transmit(0xAD);//AAI Word Program instruction opcode
            SPI_Transmit(0x02);//Address 0x020000
            SPI_Transmit(0x00);
            SPI_Transmit(0x00);
            SPI_Transmit(0xAB);//Word Byte Data: 0xABCD//This will be junk
            SPI_Transmit(0xCD);
            LATBbits.LATB5=1;
            while((Read_Status_Reg()& 0x01) == 0x01);//Polling Busy bit in the status register
            
            LATAbits.LATA8=1;   //Yellow LED on
            LATAbits.LATA2=0;   //Red LED off
            LATAbits.LATA3=0;   //Green LED off
            delay(100,1000);
            
        }
        else                    //Erase button is not pressed
        {
            LATAbits.LATA8=0;   //Yellow LED off
            
        }
        
        //Press and Hold Record button
        if(PORTAbits.RA0==1)    //Record button is pressed
        {
            LATAbits.LATA2=1;   //Red LED on
            while((Read_Status_Reg()& 0x01) == 0x01);//Polling Busy bit in the status register
    
            LATBbits.LATB5=0;
            SPI_Transmit(0xAD);//AAI opcode
            SPI_Transmit(0xEF);//Word Byte Data: 0xEFBA Actual Data
            SPI_Transmit(0xBA);
            LATBbits.LATB5=1;
        }
        else                    //Record button not pressed
        {
            LATAbits.LATA2=0;   //Red LED off
        }
        

        if(PORTAbits.RA1==1)    //Play button is pressed
        {
            LATAbits.LATA3=1;   //Green LED on
            
            LATBbits.LATB5=0;    
            SPI_Transmit(0x04);//Opcode WRDI to exit AAI Mode
            LATBbits.LATB5=1;
            
            
            Read_Status_Reg();//Read status register
            LATBbits.LATB5=0;
            SPI_Transmit(0x03);//Read opcode
            SPI_Transmit(0x02);//3-byte start address
            SPI_Transmit(0x00);
            SPI_Transmit(0x00);
            while(SPI_Receive()!=0xFF)//Read all the data that was written until we reach the area of memory that is clear
            {
                SPI_Receive();
            }
            LATBbits.LATB5=1;
            
        }
        else                    //Play button is not pressed
        {
            LATAbits.LATA3=0;   //Green LED off
        }


    }
    
}