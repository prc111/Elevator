
/* uses inbyte , x , bd : variables */
/* SDA SCL to b defined in main program */

//sfr P1 = 0x90 ;
//sbit SDA = P1^2;
//sbit SCL = P1^3; 
// delay introduced in SCL SDA command line for testing 8574AN TEXAS on 16-02-2019

#define _I2C_

data unsigned char inbyte,x ;
bdata unsigned char bd1 ;
bit istatus;
sbit d0 = bd1^0;
sbit d1 = bd1^1;
sbit d2 = bd1^2;
sbit d3 = bd1^3;
sbit d4 = bd1^4;
sbit d5 = bd1^5;
sbit d6 = bd1^6;
sbit d7 = bd1^7;

void sdah(void) {SDA=1;SDA=1;}
void sdal(void) {SDA=0; SDA=0;}
void sclh(void) {SCL=1; SCL=1; }
void scll(void) {SCL=0; SCL = 0 ;}

void i2c_start(void){
                    sdah();sclh();
                    istatus=SDA;sdal();scll(); 
                    }
void i2c_stop(void){scll();sdal(); sclh(); sdah(); }

/*
char i2cinchar()
 {
     inbyte = 0 ;
     for ( x =0 ; x<8 ; x++)
      {
        inbyte *=2; sclh(); inbyte |= SDA ; scll() ;
      }
        return (inbyte);
 }
*/
void  i2c_outchar(char o_char)
{
   bd1 = o_char ;
   
   if(d7) sdah() ; else sdal(); sclh(); scll();
   if(d6) sdah() ; else sdal(); sclh(); scll();
   if(d5) sdah() ; else sdal(); sclh(); scll();
   if(d4) sdah() ; else sdal(); sclh(); scll();
   if(d3) sdah() ; else sdal(); sclh(); scll();
   if(d2) sdah() ; else sdal(); sclh(); scll();
   if(d1) sdah() ; else sdal(); sclh(); scll();
   if(d0) sdah() ; else sdal(); sclh(); scll();

   sdah();
   sclh();
   istatus = SDA ;
   scll();
   
  // return (status) ;
}


#ifdef _I2C_

//void i2c_nack(void) { sdah(); sclh(); scll(); }


void i2c_inchar()
 {
     inbyte = 0 ;
     for ( x =0 ; x<8 ; x++)
      {
        inbyte *=2; sclh(); inbyte |= SDA ; scll() ;
      }
//        return (inbyte);
 }

void i2c_out(unsigned char address, unsigned char dbyte )
{
        i2c_start();
        i2c_outchar(address);
        i2c_outchar(dbyte);
        i2c_stop();
}

unsigned char  i2c_in(unsigned char address)
{
        i2c_start();
        i2c_outchar(address | 0x01 );
        i2c_inchar();
        i2c_stop();
        return(inbyte);
}

#endif
