/*======================================*/
/* CN[h                         */
/*======================================*/
#include 	<no_float.h>
#include 	<stdio.h>
#include    <machine.h>
#include    "h8_3048.h"

/*======================================*/
/* V{θ`                         */
/*======================================*/

/* θέθ */
#define         TIMER_CYCLE     3071    /* ^C}ΜTCN 1ms     */
                                        /* Σ/8Εgp·ικA     */
                                        /* Σ/8 = 325.5[ns]         */
                                        /* TIMER_CYCLE =          */
                                        /*      1[ms] / 325.5[ns]   */
                                        /*               = 3072     */
#define         PWM_CYCLE       42152   /* PWMΜTCN 16ms       */
                                        /* PWM_CYCLE =            */
                                        /*      16[ms] / 325.5[ns]  */
                                        /*               = 49152    */
int         SERVO_CENTER  = 4138;    /* T[{ΜZ^l         */
#define         HANDLE_STEP     26      /* 1KͺΜl                */

/* }XNlέθ ~F}XN θ(³ψ)@F}XN³΅(Lψ) */
#define MASK2_2         0x66            /* ~~~~             */
#define MASK2_0         0x60            /* ~~~~~~             */
#define MASK0_2         0x06            /* ~~~~~~             */
#define MASK3_3         0xe7            /* ~~             */
#define MASK0_3         0x07            /* ~~~~~             */
#define MASK3_0         0xe0            /* ~~~~~             */
#define MASK4_0         0xf0            /* ~~~~             */
#define MASK0_4         0x0f            /* ~~~~             */
#define MASK4_4         0xff            /*              */

/*======================================*/
/* vg^CvιΎ                     */
/*======================================*/
void init( void );
unsigned char sensor_inp( unsigned char mask );
unsigned char dipsw_get( void );
unsigned char pushsw_get( void );
void led_out( unsigned char led );
void speed( int accele_l, int accele_r );
void handle( int angle );
void sp_mode( int mode_l, int mode_r );
void timer( unsigned long timer_set );

/*======================================*/
/* O[oΟΜιΎ                 */
/*======================================*/
unsigned long   cnt0;                   /* timerΦp              */
unsigned long   cnt1;                   /* mainΰΕgp             */

int pattern = 0;

/************************************************************************/
/* CvO                                                     */
/************************************************************************/
void main( void )
{
    unsigned char   now_sw;             /* »έfBbvXCb`L― */
    unsigned char   before_sw;          /* OρfBbvXCb`L― */
    unsigned char   c = 0;              /* μΖp                   */
    int             in = 0;             /* μΖp                   */

    /* }CR@\Μϊ» */
    init();                             /* ϊ»                   */
    init_sci1( 0x00, 79 ); 				/* SCI1ϊ» 				*/ 
	set_ccr( 0x00 );                    /* SΜθέΒ         */

    /* Οϊ» */
    before_sw = dipsw_get();
    cnt1 = 0;

    /* }CRJ[ΜσΤϊ» */
    handle( 0 );
    speed( 0, 0 );
    led_out( 0x0 );
	
	pattern = 0;
    while( 1 ) {
		switch( pattern ){
			case 0:
				handle( 0 );
    			speed( 0, 0 );
    			led_out( 0x0 );
				c = 0;
				in = 0;
				
				/* j[ */
            	printf( "\n\n" );
            	printf( "MCR_kazoom Ver.1.x"
                	    "Test Program(H8-3048F / RY3048Fone Ver.) Ver1.00\n" );
            	printf( "\n" );
            	printf( "1 : LEDΜeXg\n" );
        	    printf( "2 : XCb`ΜeXg\n" );
    	        printf( "3 : T[{[^ΜeXg\n" );
    	        printf( "4 : T[{Z^[lΜ²?\n" ); 
				printf( "5 : fW^ZTξΒΜeXg\n" );
	            printf( "6 : [^ΜeXg\n" );
        	    printf( "\n" );
    	        printf( "1-6ΜπόΝ΅Δ­Ύ³’ " );
				pattern = 1;
				break;
			
			case 1:
				scanf("%d", &in );
				if(in > 6 || in < 0 ){
					printf("όΝͺs³Ε·\n\n");
					pattern = 0;
				}
				else{
					pattern = in * 10;	
					cnt1 = 0;
				}
				scanf( "%*[^\n]" );
				break;
			
			case 10:
				/* LEDΜeXg */		
				printf("\nLEDͺ0.5b¨«Ι_΅ά·B\n");
				pattern = 11;
				cnt1 = 0;
				break;
			
			case 11:
				if( cnt1 >= 2000 ){
					cnt1 = 0;
					c++;
					if( c >= 2 ) pattern = 0;	
				}
				if( cnt1 < 500 ){
					led_out(0x1);
				}
				else if( cnt1 < 1000 ){
					led_out(0x2);	
				}
				else if( cnt1 < 1500 ){
					led_out(0x3);	
				}
				else if( cnt1 < 2000 ){
					led_out(0x0);				
				}
				break;
			
			case 20:
				/* XCb`ΜeXg */
				printf("\nXCb`πeXg΅ά·\n" );
				pattern = 21;
				cnt1 = 0;
				break;
			
			case 21:
				if(!pushsw_get()){
					led_out(0x1);	
				}
				else{
					led_out(0x2);	
				}
				if( cnt1 > 5000 ){
					pattern = 0;
					cnt1 = 0;
				}
				break;
			
			case 30:
				/* T[{[^ΜeXg */ // x[VbN}VΕΝΕε49?
				printf("T[{π1b¨«Ι 0 -> 45 -> -45 ΜΙ?©΅ά·B\n");
				printf("Εεpx: ");
				scanf("%d",&in);
				pattern = 31;
				cnt1 = 0;
				break;
				
			case 31:
				printf("0\n");
				handle(0);
				timer(3000);
				printf("%d\n",in);
				handle(in);
				timer(3000);
				printf("-%d\n",in);
				handle(-1*in);
				timer(3000);
				pattern = 0;
				break;
			
			case 40:
				/* T[{Z^[lΜ²? */
				printf("T[{Z^[lΜ²?π΅ά·B\n");
				printf("-1πόΝΕIΉΕ·B\n");
				printf("»έΜlΝ %d Ε·\n\n",SERVO_CENTER);
				pattern = 41;
				cnt1 = 0;
				break;
			
			case 41:
				handle(0);
				printf("SERVO_CENTER= ");
				scanf("%d", &in);
				if( in == -1 ){
					pattern = 0;	
				}
				else{
					SERVO_CENTER = in;
				}
				break;
			
			case 50:
				/* fW^ZTξΒΜeXg */
				printf("fW^ZTξΒ(Ver.5)ΜeXgΕ·\n");
				pattern = 51;
				cnt1 = 0;
				break;
			
			case 51:
				printf("sensor= 0x%x\n", sensor_inp(MASK4_4));
				if( cnt1 > 5000 ) pattern = 0;
				break;
			
			
			case 60:
				/* [^ΜeXg */
				printf("[^ΜeXg\n");
				printf("1:Ά  2:E  0:IΉ\n");
				scanf("%d",&in);
				if(in == 0 ) pattern = 0;
				else{
					pattern = 60+in;
				}
				cnt1 = 0;
				break;
			
			case 61:
				/* Ά */
				printf("Ά: β~ u[L\n");
				sp_mode(0, 0 );
				speed(0, 0);
				timer(1000);
				
				printf("Ά: ³]50% u[L\n");
				sp_mode(0, 0 );
				speed(50, 0);
				timer(1000);
				
				printf("Ά: ³]50% t[\n");
				sp_mode(1, 0 );
				speed(50, 0);
				timer(1000);
				
				printf("Ά: t]50% u[L\n");
				sp_mode(0, 0 );
				speed(-50, 0);
				timer(1000);
				
				printf("Ά: t]50% t[\n");
				sp_mode(1, 0 );
				speed(-50, 0);
				timer(1000);
				
				printf("Ά: ³]100% u[L\n");
				sp_mode(0, 0 );
				speed(100, 0);
				timer(1000);
			
				printf("Ά: ³]100% t[\n");
				sp_mode(1, 0 );
				speed(100, 0);
				timer(1000);
				
				printf("Ά: t]100% u[L\n");
				sp_mode(0, 0 );
				speed(-100, 0);
				timer(1000);
			
				printf("Ά: t]100% t[\n");
				sp_mode(1, 0 );
				speed(-100, 0);
				timer(1000);
				speed(0, 0);
				pattern = 0;
				break;
			
			case 62:
				/* E */
				printf("E: β~ u[L\n");
				sp_mode(0, 0 );
				speed(0, 0);
				timer(1000);
				
				printf("E: ³]50% u[L\n");
				sp_mode(0, 0 );
				speed(0, 50);
				timer(1000);
				
				printf("E: ³]50% t[\n");
				sp_mode(0, 1 );
				speed(0, 50);
				timer(1000);
				
				printf("E: t]50% u[L\n");
				sp_mode(0, 0 );
				speed(0, -50);
				timer(1000);
				
				printf("E: t]50% t[\n");
				sp_mode(0, 1 );
				speed(0, -50);
				timer(1000);
				
				printf("E: ³]100% u[L\n");
				sp_mode(0, 0 );
				speed(0, 100);
				timer(1000);
			
				printf("E: ³]100% t[\n");
				sp_mode(0 ,1 );
				speed(0, 100);
				timer(1000);
				
				printf("E: t]100% u[L\n");
				sp_mode(0, 0 );
				speed(0, -100);
				timer(1000);
			
				printf("E: t]100% t[\n");
				sp_mode(0, 1 );
				speed(0, -100);
				timer(1000);
				speed(0, 0);
				pattern = 0;
				break;
		}
    
    }
}

/************************************************************************/
/* H8/3048Fΰ W[@ϊ»                                       */
/************************************************************************/
void init( void )
{
    /* |[gΜόoΝέθ */
    P1DDR = 0xff;
    P2DDR = 0xff;
    P3DDR = 0xff;
    P4DDR = 0xff;
    P5DDR = 0xff;
    P6DDR = 0xf0;                       /* CPUξΒγΜDIP SW        */
    P8DDR = 0xff;
    P9DDR = 0xf7;                       /* ΚM|[g               */
    PADDR = 0xff;
    PBDR  = 0xc0;
    PBDDR = 0xfe;                       /* [^hCuξΒVol.3  */
    /* ¦ZTξΒΜP7ΝAόΝκpΘΜΕόoΝέθΝ θάΉρ     */

    /* ITU0 1ms²ΖΜθέ */
    ITU0_TCR = 0x23;
    ITU0_GRA = TIMER_CYCLE;
    ITU0_IER = 0x01;

    /* ITU3,4 Zbg―ϊPWM[h ΆE[^AT[{p */
    ITU3_TCR = 0x23;
    ITU_FCR  = 0x3e;
    ITU3_GRA = PWM_CYCLE;               /* όϊΜέθ               */
    ITU3_GRB = ITU3_BRB = 0;            /* Ά[^ΜPWMέθ        */
    ITU4_GRA = ITU4_BRA = 0;            /* E[^ΜPWMέθ        */
    ITU4_GRB = ITU4_BRB = SERVO_CENTER; /* T[{ΜPWMέθ          */
    ITU_TOER = 0x38;

    /* ITUΜJEgX^[g */
    ITU_STR = 0x09;
}

/************************************************************************/
/* ITU0 θέ                                                    */
/************************************************************************/
#pragma interrupt( interrupt_timer0 )
void interrupt_timer0( void )
{
    ITU0_TSR &= 0xfe;                   /* tONA             */
    cnt0++;
    cnt1++;
}

void timer( unsigned long timer_set ){
	cnt0 = 0;
	while(cnt0 < timer_set);	
}
/************************************************************************/
/* ZTσΤo(eXg[hp)                                       */
/* ψ@ }XNl                                                      */
/* ίθl ZTl                                                      */
/************************************************************************/
unsigned char sensor_inp( unsigned char mask )
{
    unsigned char sensor;

    sensor  = ~P7DR;
    sensor &= mask;

    return sensor;
}

/************************************************************************/
/* fBbvXCb`lΗέέ                                           */
/* ίθl XCb`l 0`15                                              */
/************************************************************************/
unsigned char dipsw_get( void )
{
    unsigned char sw;

    sw  = ~P6DR;                        /* fBbvXCb`Ηέέ */
    sw &= 0x0f;
	sw = 0x0f-sw;

    return  sw;
}

/************************************************************************/
/* vbVXCb`lΗέέ                                           */
/* ίθl XCb`l ON:1 OFF:0                                         */
/************************************************************************/
unsigned char pushsw_get( void )
{
    unsigned char sw;

    sw  = ~PBDR;                        /* XCb`Μ ι|[gΗέέ */
    sw &= 0x01;

    return  sw;
}

/************************************************************************/
/* LED§δ                                                              */
/* ψ@XCb`l LED0:bit0 LED1:bit1  "0":Α "1":_              */
/* α)0x3¨LED1:ON LED0:ON  0x2¨LED1:ON LED0:OFF                       */
/************************************************************************/
void led_out( unsigned char led )
{
    unsigned char data;

    led = ~led;
    led <<= 6;
    data = PBDR & 0x3f;
    PBDR = data | led;
}

/************************************************************************/
/* ¬x§δ                                                             */
/* ψ@ Ά[^:-100`100 , E[^:-100`100                       */
/*        0Εβ~A100Ε³]100%A-100Εt]100%                        */
/************************************************************************/
void speed( int accele_l, int accele_r )
{
    unsigned char   sw_data;
    unsigned long   speed_max;

    sw_data  = dipsw_get() + 5;         /* fBbvXCb`Ηέέ */
    speed_max = (unsigned long)(PWM_CYCLE-1) * sw_data / 20;

    /* Ά[^ */
    if( accele_l >= 0 ) {
        PBDR &= 0xfb;
        ITU3_BRB = speed_max * accele_l / 100;
    } else {
        PBDR |= 0x04;
        accele_l = -accele_l;
        ITU3_BRB = speed_max * accele_l / 100;
    }

    /* E[^ */
    if( accele_r >= 0 ) {
        PBDR &= 0xf7;
        ITU4_BRA = speed_max * accele_r / 100;
    } else {
        PBDR |= 0x08;
        accele_r = -accele_r;
        ITU4_BRA = speed_max * accele_r / 100;
    }
}

/************************************************************************/
/* T[{nhμ                                                   */
/* ψ@ T[{μpxF-90`90                                       */
/*        -90ΕΆΦ90xA0ΕάΑ·?A90ΕEΦ90xρ]                  */
/************************************************************************/
void handle( int angle )
{
    ITU4_BRB = SERVO_CENTER - angle * HANDLE_STEP;
}

/************************************************************************/
/* [^β~?μiu[LAt[j                                   */
/* ψ@ Ά[^:FREE or BRAKE , E[^:FREE or BRAKE               */
/************************************************************************/
void sp_mode( int mode_l, int mode_r )
{
    if( mode_l ) {
        PADR |= 0x01;
    } else {
        PADR &= 0xfe;
    }
    if( mode_r ) {
        PADR |= 0x02;
    } else {
        PADR &= 0xfd;
    }
}

/************************************************************************/
/* end of file                                                          */
/************************************************************************/
