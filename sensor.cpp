//#include "mbed.h"
#include "math.h"
#include <stdio.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <math.h>
#include <signal.h>
#include <bcm2835.h>
//  development done on MBED, LPC1768 ARM CORTEX M3,
//  Using 16Kram

// un-comment to switch on, print on screen parameters and information
#define  DEBUGOUT

#ifdef DEBUGOUT

#define   PRNT_PARAM_PTAT
#define   PRNT_PARAM_CALC_TA
#define   DEBUG_EEPROM
#define   DEBUG_CONFIG
#define   DEBUG_STATUS
#define   DEBUG_READ_MEM
#define   PRNT_Calc_Vdd
#define   PRNT_
#define   PRNT_VDD_param
#define   PRNT_TA_param
#define   PRNT_OFFSET_param
#define   PRNT_SENSITIVITY_param
#define   PRNT_Kv_param
#define   PRNT_Kta_param
#define   PRNT_GAIN_param
#define   PRNT_KsTa_param
#define   PRNT_KsTo_param
#define   PRNT_SENS_ALPHA_CP_param
#define   PRNT_OFFSET_CP_param
#define   PRNT_Kv_CP_param
#define   PRNT_KTa_CP_param
#define   PRNT_TGC_param

#endif
// matrix
// 0------------------------------NCOLS-1> index j, X
// |
// |
// |
// |
// |
// |
// |
// |
// NROWS-1, index i, Y
// coord[row][col]

void rd_mlx_config();
void wr_mlx_config() ;
void analyse_config();
void GET_A_KEY();
void read_mem(char A_MSB, char A_LSB,int cnt);
void Reset_New_Data_In_Ram2();
void Restore_VDD_param();
void Restore_TA_param();
void Restore_OFFSET_param();
void Restore_SENSITIVITY_param();
void Restore_Kv_param();
void Restore_Kta_param();
void Restore_GAIN_param();
void Restore_KsTa_param();
void Restore_KsTo_param();
void Restore_SENS_ALPHA_CP_param();
void Restore_OFFSET_CP_param();
void Restore_Kv_CP_param();
void Restore_KTa_CP_param();
void Restore_TGC_param();
void Set_Sensor_In_Chess_mode();
void Get_Image_Chess();
void Get_Image2(int);
short median(int n, short x[]);
short Pat_Boundaries(int x,int y);
void  Correct_Outliers(int fr,int pat);

// MBED SPECIFIC
//==============
//

/*extern I2C i2c;                     // sda, scl
extern Serial pc;                //Virtual serial port over USB, tx, rx
extern Serial pc2;*/
                                      //Virtual serial port over USB


// ABI MBED TO BCM2835 SPECIFIC
//==============
//



//=======================
// sensor and application
//=======================
#include "defines.h"

extern char         refesh_rate;
int                 pattern = LINE;

// --------------------
// - FOR MLX ROUTINES -
// --------------------

char                eeprom[(0x2740-0x2400)*2];  // 1664 bytes
char                cmd[5];
char                reg_config[2];
char                status[2];

char                Ram_store[8];              // 8 bytes reserved, increase size if read_mem requires more
unsigned char       ir_data[NROWS*NCOLS*2+2];   // toke a few bytes more to be sure!
char                SENSOR_ID[32];

// to store calib data recalculated from eeprom storage

float               Pix_os_ref_range_1[NROWS][NCOLS];
float               Alpha[NROWS][NCOLS];
float               Kv_odd_odd, Kv_even_odd, Kv_odd_even, Kv_even_even;
float               Kta_range_1[NROWS][NCOLS];
float               ALPHA_CP_subpage_0, ALPHA_CP_subpage_1;
float               OFFSET_CP_subpage_0,OFFSET_CP_subpage_1;
float               KVcp;
float               Kta_cp;
float               TGC;
int                 VPTAT;
int                 VBE;
int                 DELTA_V;
float               Vdd_25 = -13234.02;
static const float  Vdd_V0  = 3.3;
float               Kv_Vdd = -3203.33;
float               K_gain=1;

/* Amplifier gain */
float               GainMeas_25_3v2 =  6625.217;

/* Ambient Temperature */
float               VPTAT_25 = 12304;
float               Kv_PTAT = 0.0066;
float               Kt_PTAT = 42.3;
float               ALPHA_PTAT= 9;

/* Object Temp */
float               KsTa= -0.002;
float               KsTo = -0.0008;
float               emmisivity = 1.0;
float               Ta0;
float               g_Vdd;
float               g_Ta;
short               IMA[NROWS][NCOLS];
short               OUTLIERS [16][3];

int                 frame=0;
int                 i,j,k;                      // general usage in loops
extern short        PRNT;
char                c;
char                str[6];
int                 x=0,y=0;
float               R_T=0;
int                 next_frame;
float               pix_gain_cp_sp0, pix_gain_cp_sp1;
float               Kg;
int                 outliers_present=0;

// ************
// * get_ptat *
// ************

void get_ptat()
{
  int a1;

  // get RAM value 0x0720
  read_mem( (char) 0x07, (char) 0x20,(int) 2);
  a1 = ((int)Ram_store[0])*256+(int)Ram_store[1];
  if (a1 >32767 ) a1-=65536;
  VPTAT= a1;

  // get RAM value 0x0700
  read_mem( (char) 0x07, (char) 0x00,(int) 2);
  a1 = ((int)Ram_store[0])*256+(int)Ram_store[1];
  if (a1>32767) a1-=65536;
  VBE= a1;

  // get RAm value 0x072a
  read_mem( (char) 0x07, (char) 0x2a,(int) 2);
  a1 = ((int)Ram_store[0])*256+(int)Ram_store[1];
  if (a1>32767) a1-=65536;
  DELTA_V = a1;

#ifdef PRNT_PARAM_PTAT
  printf("VPTAT=%d\n\r",VPTAT);
  printf("VBE=%d\n\r",VBE);
  printf("DELTA_V=%d\n\r",DELTA_V);
#endif
} // end get_ptat()

// ****************
// * Calculate TA *
// ****************

float Calc_Ta()
{
   get_ptat();


   float VPTAT_art = (float)(VPTAT/(VPTAT*ALPHA_PTAT+VBE))*262144.0;
   float DELTA_Volt = ((float)DELTA_V-Vdd_25)/Kv_Vdd;

   float Ta = VPTAT_art/(1+Kv_PTAT*DELTA_Volt);
   Ta = (Ta-VPTAT_25)/Kt_PTAT+25;

#ifdef PRNT_PARAM_CALC_TA
   printf("-Ta %f\n\r",Ta);
   printf("-VPTAT %d\n\r",VPTAT);
   printf("-ALPHA_PTAT: %f \r\n", ALPHA_PTAT);
   printf("-VBE: %d \r\n", VBE);
   printf("-DELTA_Volt: %f \r\n", DELTA_Volt);
   printf("-Vdd_25: %f \r\n", Vdd_25);
   printf("-Kv_Vdd: %f \r\n", Kv_Vdd);
   printf("-VPTAT_art: %f \r\n", VPTAT_art);
   printf("-Kv_Vdd: %f \r\n", Kv_Vdd);
   printf("-Kv_PTAT: %f \r\n", Kv_PTAT);
   printf("-Kt_PTAT: %f \r\n", Kt_PTAT);
#endif
   return Ta;

} // end Calc_Ta()


// ***********************
// * set_IR_refresh_rate *
// ***********************

void set_IR_refresh_rate(char c)
{
   refesh_rate = c;
   rd_mlx_config();

   switch (c) {
      case '0':
          printf("IR refresh 0.5 Hz\n\r");
          R_T = 1.0/0.5;
          reg_config[0] = reg_config[0] & 0xfc;
          reg_config[1] = reg_config[1] & 0x7f;
          break;
      case '1':
          printf("IR refresh 1 Hz\n\r");
          reg_config[0] = reg_config[0] & 0xfc;
          reg_config[1] = reg_config[1] | 0x80;
          R_T = 1.0/1.0;
          break;
      case '2':
          printf("IR refresh 2 Hz\n\r");
          reg_config[0] = reg_config[0] & 0xfc;
          reg_config[0] = reg_config[0] | 0x01;
          reg_config[1] = reg_config[1] & 0x7f;
          R_T = 1.0/2.0;
          break;
      case '4':
          printf("IR refresh 4 Hz\n\r");
          reg_config[0] = reg_config[0] & 0xfc;
          reg_config[0] = reg_config[0] | 0x01;
          reg_config[1] = reg_config[1] | 0x80;
          R_T = 1.0/4.0;
          break;
      case '8':
          printf("IR refresh 8 Hz\n\r");
          reg_config[0] = reg_config[0] & 0xfc;
          reg_config[0] = reg_config[0] | 0x02;
          reg_config[1] = reg_config[1] & 0x7f;
          R_T = 1.0/8.0;
          break;
      case 'A':
      case 'a':
          printf("IR refresh 16 Hz\n\r");
          reg_config[0] = reg_config[0] & 0xfc;
          reg_config[0] = reg_config[0] | 0x02;
          reg_config[1] = reg_config[1] | 0x80;;
          R_T = 1.0/16.0;
          break;
      case 'B':
      case 'b':
          printf("IR refresh 32 Hz\n\r");
          reg_config[0] = reg_config[0] & 0xfc;
          reg_config[0] = reg_config[0] | 0x03;
          reg_config[1] = reg_config[1] & 0x7f;
          R_T = 1.0/32.0;
          break;
      case 'C':
      case 'c':
          printf("IR refresh 64 Hz\n\r");
          reg_config[0] = reg_config[0] & 0xfc;
          reg_config[0] = reg_config[0] | 0x03;
          reg_config[1] = reg_config[1] | 0x80;
          R_T = 1.0/64.0;
          break;
      default:
          printf("WRONG SELECTION, DEFAULTING TO 4 HZ\n\r");
          reg_config[0] = reg_config[0] & 0xfc;
          reg_config[0] = reg_config[0] | 0x01;
          reg_config[1] = reg_config[1] | 0x80;
          refesh_rate = '4';
          R_T = 1.0/4.0;
          break;
   }
   wr_mlx_config();
}

// **********************
// * get_IR_refresh_rate*
// **********************

void get_IR_refresh_rate()
{
    printf("IR refresh 0[=0.5] 1,2,4,8,a[=16],b[=32],c[=64] HZ: \n\r");
    c =getchar();
    set_IR_refresh_rate(c);
} // end get_IR_refresh_rate


// ***************************
// * Start_sensor_To_measure *
// ***************************

void Start_sensor_To_measure(){
    cmd[0] = 0x80;
    cmd[1] = 0x00;
    char a[2];
    //int 	read (int address, char *data, int length, bool repeated=false)
    /*if(!bcm2835_init())
    {
    	printf("Error !bcm2835_init() line  341\n\r");
    	return;
    }*/
    bcm2835_i2c_begin();
    bcm2835_i2c_setSlaveAddress(I2C_ADDR);

    if (!bcm2835_i2c_write_read_rs((char *)&cmd, 2, (char *)&a, 2) == BCM2835_I2C_REASON_OK) {
      printf("Error Start measerments  BCM2835_I2C_REASON_OK was not OK \n\r");
      return;
    }




    /*cp = ( VCP_BYTES[1] << 8 ) | VCP_BYTES[0];
    return cp;*/

    //i2c.write((I2C_ADDR<<1), cmd, 2,1);

    //i2c.read((I2C_ADDR<<1), a, 2,1);    // read the two-byte echo result


    // set start of measurement bit and clear bit data available in ram
    a[1] = a[1] & 0xF7;
    /*if(!bcm2835_init())
    {
    	printf("Error !bcm2835_init() line  367\n\r");
    	return;
    }*/
    bcm2835_i2c_begin();
    bcm2835_i2c_setSlaveAddress(I2C_ADDR);
    if (bcm2835_i2c_write((const char *)&cmd, 2) == BCM2835_I2C_REASON_OK )
      printf("CORRECT Line 350 \n\r");


    /*i2c.write((I2C_ADDR<<1), cmd, 2,0);
    (void)i2c.write(a[0]);
    (void)i2c.write(a[1]);
    bcm2835_i2c_end(); */

    (void)bcm2835_i2c_write((const char *)&a[0], 1);
    (void)bcm2835_i2c_write((const char *)&a[1], 1);
    bcm2835_i2c_end();
} // end Start_sensor_To_measure

// **********************
// * get_eeprom content *
// **********************

void get_eeprom()
{



    usleep(5);
    cmd[0] = 0x24;
    cmd[1] = 0x00;
    /*if(!bcm2835_init())
    {
    	printf("Error !bcm2835_init() line  400\n\r");
    	return;
    }*/
    bcm2835_i2c_begin();
    bcm2835_i2c_setSlaveAddress(I2C_ADDR);

    if (bcm2835_i2c_write_read_rs((char *)&cmd, 2, (char *)&eeprom, (0x2740-0x2400)*2) == BCM2835_I2C_REASON_OK )
      printf("CORRECT Line 377 \n\r");

    //i2c.write((I2C_ADDR<<1), cmd, 2,1);
    //bcm2835_i2c_read_register_rs((I2C_ADDR<<1), eeprom, (0x2740-0x2400)*2);
    //i2c.read((I2C_ADDR<<1), eeprom, (0x2740-0x2400)*2,1);    // read the two-byte echo result
    bcm2835_i2c_end();

    usleep(5);
#ifdef  DEBUG_EEPROM
    int i=0;
    int j=0;
    printf("\n\raddr %x ",i);
    for (int i=0; i<=(0x2740-0x2400)*2; i++) {
       if (j > 7 ) {
          j=0;
          printf("\n\raddr %x ",i);
       }
       printf("%x ",eeprom[i]);
       j++;
    }
    printf("\n\rEEPROM-rd-done\r\n");

#endif
    int b1 = ((int)eeprom[0x0e])*256+(int)eeprom[0x0f];
    int b2 = ((int)eeprom[0x10])*256+(int)eeprom[0x11];
    int b3 = ((int)eeprom[0x12])*256+(int)eeprom[0x13];
    (void)sprintf(SENSOR_ID,"Sensor ID = %x-%x-%x\r\n",b1,b2,b3);

} // end get_eeprom()

// *****************
// * rd_mlx_config *
// *****************

void rd_mlx_config()
{

    printf("CORRECt 1 rd_mlx__config\n\r");
    cmd[0] = 0x80;
    cmd[1] = 0x0d;
    reg_config[0] = 0x00;
    reg_config[1] = 0x00;
    printf("CORRECt 1.5 rd_mlx__config\n\r");



    usleep(5);
    printf("CORRECt 2 rd_mlx__config\n\r");
    /*if(!bcm2835_init())
    {
    	printf("Error !bcm2835_init() line  457\n\r");
    	return;
    }*/
    bcm2835_i2c_begin();
    bcm2835_i2c_setSlaveAddress(I2C_ADDR);
    //i2c.write((I2C_ADDR<<1), cmd, 2,1);
    printf("CORRECt 3 rd_mlx__config\n\r");

    //i2c.read((I2C_ADDR<<1), reg_config, 2,1);    // read the two-byte echo result

    if (bcm2835_i2c_write_read_rs((char *)&cmd, 2, (char *)&reg_config, 2) == BCM2835_I2C_REASON_OK ) printf("CORRECT Line 428 read reg_config\n\r");
    else printf("ERROR Line 428 read reg_config\n\r");

    bcm2835_i2c_end();
    printf("CORRECt 4 rd_mlx__config EXITING\n\r");

    usleep(5);
#ifdef DEBUG_CONFIG
    printf("RD-config[reg0,reg1] %d, %d\r\n", reg_config[0],reg_config[1]);
#endif
} //end rd_mlx_config

// *****************
// * wr_mlx_config *
// *****************

void wr_mlx_config()
{
    cmd[0] = 0x80;
    cmd[1] = 0x0d;



    usleep(5);
    /*if(!bcm2835_init())
    {
    	printf("Error !bcm2835_init() line  393\n\r");
    	return;
    }*/
    bcm2835_i2c_begin();
    bcm2835_i2c_setSlaveAddress(I2C_ADDR);

    if (bcm2835_i2c_write((const char *)&cmd, 2) == BCM2835_I2C_REASON_OK)
      printf("CORRECT Line 454 write write_config\n\r");

    //i2c.write((I2C_ADDR<<1), cmd, 2,1);

    (void)bcm2835_i2c_write((const char *)&reg_config[0], 1);
    (void)bcm2835_i2c_write((const char *)&reg_config[1], 1);
    bcm2835_i2c_end();

    /*(void)i2c.write(reg_config[0]);
    (void)i2c.write(reg_config[1]);
    bcm2835_i2c_end();*/

    usleep(5);
#ifdef DEBUG_CONFIG
    printf("WR-config[reg0,reg1] %d, %d\r\n", reg_config[0],reg_config[1]);
#endif
} //end wr_mlx_config

// *****************
// * rd_mlx_status *
// *****************

void rd_mlx_status()
{
    cmd[0] = 0x80;
    cmd[1] = 0x00;


    usleep(5);
    /*if(!bcm2835_init())
    {
    	printf("Error !bcm2835_init() line  531\n\r");
    	return;
    }*/
    bcm2835_i2c_begin();
    bcm2835_i2c_setSlaveAddress(I2C_ADDR);

    if (!bcm2835_i2c_write_read_rs((char *)&cmd, 2, (char *)&status, 2)   == BCM2835_I2C_REASON_OK)
      printf("ERROR Line 488 write write_config\n\r");
    bcm2835_i2c_end();

    /*i2c.write((I2C_ADDR<<1), cmd, 2,1);
    i2c.read((I2C_ADDR<<1), status, 2,1);    // read the two-byte echo result
    bcm2835_i2c_end();*/
    usleep(5);
#ifdef DEBUG_STATUS
   printf("RD-status[reg0,reg1] %d, %d\r\n", status[0],status[1]);
#endif
} //end rd_mlx_status

// *****************
// * wr_mlx_status *
// *****************

void wr_mlx_status()
{
    cmd[0] = 0x80;
    cmd[1] = 0x00;



    usleep(5);
    /*if(!bcm2835_init())
    {
    	printf("Error !bcm2835_init() line  564\n\r");
    	return;
    }*/
    bcm2835_i2c_begin();
    bcm2835_i2c_setSlaveAddress(I2C_ADDR);

    if (bcm2835_i2c_write((const char *)&cmd, 2) == BCM2835_I2C_REASON_OK)
      printf("CORRECT Line 516 write write_config\n\r");

    //i2c.write((I2C_ADDR<<1), cmd, 2,1);

    (void)bcm2835_i2c_write((const char *)&status[0], 1);
    (void)bcm2835_i2c_write((const char *)&status[1], 1);
    bcm2835_i2c_end();


  /*  i2c.write((I2C_ADDR<<1), cmd, 2,1);
    (void)i2c.write(status[0]);
    (void)i2c.write(status[1]);
    bcm2835_i2c_end();*/

    usleep(5);
#ifdef DEBUG_STATUS
    printf("WR-status[reg0,reg1] %d, %d\r\n", status[0],status[1]);
#endif
} //end wr_mlx_status



// *************************
// * wr_mlx_EEPROM_refresh *
// *************************
//   with char c :
//  '0'-->IR refresh 0.5 Hz
//  '1'-->IR refresh 1 Hz
//  '2'-->IR refresh 2 Hz
//  '4'-->IR refresh 4 Hz
//  '8'-->IR refresh 8 Hz
//  'A'-->IR refresh 16 Hz
//  'B'-->IR refresh 32 Hz
//  'C'-->IR refresh 64 Hz

void wr_mlx_EEPROM_refresh(char c)
{
    set_IR_refresh_rate(c);
    cmd[0] = 0x24;
    cmd[1] = 0x0C;



    usleep(5);
    /*if(!bcm2835_init())
    {
    	printf("Error !bcm2835_init() line  617\n\r");
    	return;
    }*/
    bcm2835_i2c_begin();
    bcm2835_i2c_setSlaveAddress(I2C_ADDR);

    if (bcm2835_i2c_write((const char *)&cmd, 2) == BCM2835_I2C_REASON_OK)
      printf("CORRECT Line 565 write reg_config\n\r");

    //i2c.write((I2C_ADDR<<1), cmd, 2,1);

    (void)bcm2835_i2c_write((const char *)&reg_config[0], 1);
    (void)bcm2835_i2c_write((const char *)&reg_config[1], 1);
    bcm2835_i2c_end();


    /*(void)i2c.write(reg_config[0]);
    (void)i2c.write(reg_config[1]);
    bcm2835_i2c_end();*/
    usleep(5);
#ifdef DEBUG_CONFIG
    printf("NEW REFRESH RATE WRITTEN-TO-EEPROM[reg0,reg1] %d, %d\r\n", reg_config[0],reg_config[1]);
#endif
} //end wr_mlx_EEPROM_refresh

// ***************************************
// * GET_A_KEY, get a keystroke to con't *
// ***************************************

void GET_A_KEY()
{
    printf("\n\rStrike a key to continue ");
    (void)getchar();
    printf("\n\r");
    return;
} // GET_A_KEY()

// ******************
// * analyse_config *
// ******************

void analyse_config()
{
   printf("Max prgm loop duration must be < %f seconds\r\n", R_T);
   printf("config[reg0,reg1] %d, %d\r\n", reg_config[0],reg_config[1]);
   char tmp =reg_config[0]>>2;
   tmp = tmp & 0x03;
   switch ((int)tmp){
      case 0:
          printf("16 bit resolution\r\n"); break;
      case 1:
          printf("17 bit resolution\r\n"); break;
      case 2:
          printf("18 bit resolution\r\n"); break;
      case 3:
          printf("19 bit resolution\r\n"); break;
      default:
          printf("RESOLUTION-->NA\r\n");
          break;
   }
   tmp =(reg_config[0] & 0x03)<<1;
   tmp =tmp | ((reg_config[1] & 0x80)>>7);
   switch ((int)tmp){
      case 0:
          printf("0.5 Hz\r\n");break;
      case 1:
          printf("1 HZ\r\n");break;
      case 2:
          printf("2 Hz\r\n");break;
      case 3:
          printf("4 Hz\r\n");break;
      case 4:
          printf("8 Hz\r\n");break;
      case 5:
          printf("16 Hz\r\n");break;
      case 6:
          printf("32 Hz\r\n");break;
      case 7:
          printf("64 Hz\r\n");break;
      default:
          printf("REFRESH_RATE-->NA\r\n");
          break;
   }
   tmp =(reg_config[1]>>1) & 0x01;
   if (tmp == 0) printf("Device in continuous mode\r\n");
   else printf("Device in step mode\r\n");
} // end analyse_config

// ************
// * read_mem *
// ************

void read_mem(char A_MSB, char A_LSB,int cnt)     // cnt = number of bytes!
{
    cmd[0] = A_MSB;
    cmd[1] = A_LSB;
    if (cnt > 32) cnt =32;
    for (int a=0;a<cnt;a++) {Ram_store[a] = 0x00;}



    usleep(5);
    /*if(!bcm2835_init())
    {
    	printf("Error !bcm2835_init() line  721\n\r");
    	return;
    }*/
    bcm2835_i2c_begin();
    bcm2835_i2c_setSlaveAddress(I2C_ADDR);

    if (bcm2835_i2c_write_read_rs((char *)&cmd, 2, (char *)&Ram_store, cnt)  == BCM2835_I2C_REASON_OK)
      printf("CORRECT Line 665 write reg_config\n\r");
    bcm2835_i2c_end();

    /*i2c.write((I2C_ADDR<<1), cmd, 2,1);
    i2c.read((I2C_ADDR<<1), Ram_store, cnt,1);    // read the two-byte echo result
    bcm2835_i2c_end();*/
    usleep(5);
#ifdef DEBUG_READ_MEM
    printf("MSB=%x,LSB=%x \r\n",A_MSB,A_LSB);
    for (int a=0;a<cnt;a++)
        printf("Ram_store[%d]= %x\r\n",a,Ram_store[a]);
#endif
} //end read_mem



// ***************************************
// * get_array:                          *
// *    Gets the IR raw data from sensor *
// ***************************************

void get_array()
{

    usleep(5);
    cmd[0] = 0x04;
    cmd[1] = 0x00;
    /*if(!bcm2835_init())
    {
    	printf("Error !bcm2835_init() line  757\n\r");
    	return;
    }*/
    bcm2835_i2c_begin();
    bcm2835_i2c_setSlaveAddress(I2C_ADDR);

    if (bcm2835_i2c_write_read_rs((char *)&cmd, 2, (char *)&ir_data, (NROWS*NCOLS)*2)  == BCM2835_I2C_REASON_OK)
      printf("CORRECT Line 696 write ir_data\n\r");

    /*i2c.write((I2C_ADDR<<1), cmd, 2,1);
    i2c.read((I2C_ADDR<<1), (char *)ir_data, (NROWS*NCOLS)*2,1);    // read the two-byte echo result*/


    bcm2835_i2c_end();
    usleep(5);
} // end get_array

// **********************************
// * Measurement_done2              *
// * based on the frame toggle flag *
// **********************************

void  Measurement_done2()
{
    int frame2;
    while(1){
       (void)read_mem(0x80, 0x00,2);               // read 2 bytes!
       if ( (Ram_store[1]&0x01) == 0x00) frame2 =0;
       if ( (Ram_store[1]&0x01) == 0x01) frame2 =1;
       if ((frame2 ^ frame) == 1){
          if (frame == 1) frame =0;
          else frame = 1;
          break;
       }
     }//   end while
   return;
} // end Measurement_done2

// ***********************************************
// * Measurement_done                            *
// * based on new data toggle flag               *
// * in combination with Start_sensor_To_measure *
// ***********************************************

void  Measurement_done()
{
int timeout=250;
   while(1){
      (void)read_mem(0x80, 0x00,2);
      sleep(1);
      timeout--;
      if( (Ram_store[1] & 0x08) == 0x08) break;
      if (timeout <=0) break;
   } // end while

   // read 2 bytes!
   if ( (Ram_store[1]&0x01) == 0x00) frame =0;
   if ( (Ram_store[1]&0x01) == 0x01) frame =1;
   return;
} // end Measurement_done

// *********************
// * Restore_VDD_param *
// *********************

void Restore_VDD_param()
{
   // Kv_vdd
   // ======
   int a = ((int)eeprom[(0x2433-0x2400)*2])*256+(int)eeprom[(0x2433-0x2400)*2+1];

   int a1 = a & 0xff00;
   float b = (float)a1/256.0;
   if (b > 127) b= b - 256;
   Kv_Vdd = b*32.0;

   // Vdd_25
   // ======
   a1 = a & 0x00ff;
   b=(float)a1;
   b= b - 256.0;
   Vdd_25 = ((float)b)*32.0-8192.0;

#ifdef PRNT_VDD_param
   printf("Kv_vdd:%f\n\r", Kv_Vdd);
   printf("Vdd_25:%f\n\r", Vdd_25);
#endif
} // end Restore_VDD_param

// ********************
// * Restore_TA_param *
// ********************

void Restore_TA_param()
{
   // Kv_PTAT
   // ======
   int a = ((int)eeprom[(0x2432-0x2400)*2])*256+(int)eeprom[(0x2432-0x2400)*2+1];
   int a1 = a & 0xfc00;
   float b = (float)a1/1024.0;

   if (b > 31) b= b - 64;
   Kv_PTAT = b/4096.0;

   // Kt_PTAT
   // =======
   a1 = a & 0x03ff;
   b = (float)a1;

   if (b > 511) b= b - 1024;
   Kt_PTAT = b/8.0;

   // VPTAT_25
   // ========
   a = ((int)eeprom[(0x2431-0x2400)*2])*256+(int)eeprom[(0x2431-0x2400)*2+1];
   if (a > 32767 ) a= a - 65536;
   VPTAT_25 = (float)a;

   // KPTAT / ALPHA_PTAT
   // ==================
   a = ((int)eeprom[(0x2410-0x2400)*2])*256+(int)eeprom[(0x2410-0x2400)*2+1];
   a1 = (a & 0xf000)/4096;
   ALPHA_PTAT = (float)a1/4.0 +8;

#ifdef PRNT_TA_param
   printf("Kv_PTAT:%f\n\r", Kv_PTAT);
   printf("Kt_PTAT:%f\n\r", Kt_PTAT);
   printf("VPTAT_25:%f\n\r", VPTAT_25);
   printf("ALPHA_PTAT:%f\n\r", ALPHA_PTAT);
#endif

} // end Restore_TA_param



// ************************
// * Restore_OFFSET_param *
// ************************

void Restore_OFFSET_param()
{
    // OFFSET_avg
    // ==========
    int a = ((int)eeprom[(0x2411-0x2400)*2])*256+(int)eeprom[(0x2411-0x2400)*2+1];
    if (a > 32767 ) a= a - 65536;
    float OFFSET_avg = (float)a;

    // OCCscaleremnant
    a = ((int)eeprom[(0x2410-0x2400)*2])*256+(int)eeprom[(0x2410-0x2400)*2+1];
    unsigned int OCCscaleremnant = a & 0x000f;

    // OCC_scale_row
    int OCC_scale_row = (a & 0x0f00)/256;

     // OCC_scale_col
    int OCC_scale_col = (a & 0x00f0)/16;

    int adr,adr1,adr2;
    for (int row=0; row<NROWS; row++) {
       // OCC_row
       adr = (((int)row/4)+0x2412)-0x2400;

       a = ((int)eeprom[adr*2])*256+(int)eeprom[adr*2+1];
       int OCC_row;
       switch(row%4){
          case 0:
            OCC_row = a & 0x000f;
            break;
          case 1:
            OCC_row = a & 0x00f0;
            OCC_row = OCC_row / 16;
            break;
          case 2:
            OCC_row = a & 0x0f00;
            OCC_row = OCC_row / 256;
            break;
          case 3:
            OCC_row = a & 0xf000;
            OCC_row = OCC_row / 4096;
            break;
          default :
            OCC_row = a & 0x000f;
            printf("OCC_row->this default can not!!\n\r");
            break;
       }
       if ( OCC_row > 7) OCC_row -=16;
       for (int col=0; col<NCOLS; col++) {
         #ifdef PRNT_OFFSET_param
         printf("row address[%d,%d]=%x\n\r",row,col,adr+0x2400);
         #endif
         // OCC_col
         adr1 = (((int)col/4)+0x2418)-0x2400;
         #ifdef PRNT_OFFSET_param
         printf("col address[%d,%d]=%x\n\r",row,col,adr1+0x2400);
         #endif
         a = ((int)eeprom[adr1*2])*256+(int)eeprom[adr1*2+1];
         int OCC_col;
         switch(col%4){
            case 0:
                OCC_col = a & 0x000f;
                break;
            case 1:
                OCC_col = a & 0x00f0;
                 OCC_col = OCC_col / 16;
                break;
            case 2:
                OCC_col = a & 0x0f00;
                OCC_col = OCC_col / 256;
                break;
            case 3:
                OCC_col = a & 0xf000;
                OCC_col = OCC_col / 4096;
                break;
            default :
                OCC_col = a & 0x000f;
                printf("OCC_col->this default can not!!\n\r");
                break;
         }
         if ( OCC_col > 7) OCC_col -=16;

         // get pix-offset

        adr2 = ((row)*32+(col)+0x2440)-0x2400;
        a = ((int)eeprom[adr2*2])*256+(int)eeprom[adr2*2+1];
        int offset_row_col= a & 0xfc00;
        offset_row_col=  offset_row_col/1024;
        if ( offset_row_col > 31) offset_row_col -=64;
        // insert value in array of Pix_os_ref_range_1[NROWS][NCOLS]
        Pix_os_ref_range_1[row][col]=OFFSET_avg+(float)OCC_row*(float)pow(2.0,(double)OCC_scale_row)+(float)OCC_col*(float)pow(2.0,(double)OCC_scale_col);
        Pix_os_ref_range_1[row][col] = Pix_os_ref_range_1[row][col] + (float)pow(2.0,(double)OCCscaleremnant)*(float)offset_row_col;
        #ifdef PRNT_OFFSET_param
        printf("pix-offset address has value [%d,%d]=%d\n\r",row,col,offset_row_col);
        printf("Pix_os_ref_range_1[%d,%d] = %f\n\r",row,col,Pix_os_ref_range_1[row][col]);
        #endif
        } // for rows
     } // for cols
}  // end Restore_OFFSET_param

// *****************************
// * Restore_SENSITIVITY_param *
// *****************************

void Restore_SENSITIVITY_param()
{
    // fill out  Alpha[NROWS][NCOLS];
    int a,a1;
    int adr, adr1, adr2;
    int ACC_row, ACC_col;

    // Alpha_scale
    // ===========
    a = ((int)eeprom[(0x2420-0x2400)*2])*256+(int)eeprom[(0x2420-0x2400)*2+1];
    int Alpha_scale = (a & 0xf000)/4096 + 30;

    #ifdef PRNT_SENSITIVITY_param
    printf("Alpha_scale=%d\n\r",Alpha_scale);
    #endif

    // Alpha_ref
    // ==========
    a1 = ((int)eeprom[(0x2421-0x2400)*2])*256+(int)eeprom[(0x2421-0x2400)*2+1];
    float Alpha_ref = (float)a1;

    #ifdef PRNT_SENSITIVITY_param
    printf("Alpha_ref=%f\n\r",Alpha_ref);
    #endif

    // ACC_scale_row
    // =============
    int ACC_scale_row=(a & 0x0f00)/256;

    #ifdef PRNT_SENSITIVITY_param
    printf("ACC_scale_row=%d\n\r",ACC_scale_row);
    #endif

    // ACC_scale_col
    // =============
    int ACC_scale_col=(a & 0x00f0)/16;

    #ifdef PRNT_SENSITIVITY_param
     printf("ACC_scale_col=%d\n\r",ACC_scale_col);
    #endif

    // ACC_scale_remnant
    // =============
     int ACC_scale_remnant= a & 0x000f;

    #ifdef PRNT_SENSITIVITY_param
     printf("ACC_scale_remnant=%d\n\r",ACC_scale_remnant);
    #endif

    // loop over rows and cols
    for (int row=0; row<NROWS; row++) {
       // ACC_row
       adr = (((int)row/4)+0x2422)-0x2400;
       a = ((int)eeprom[adr*2])*256+(int)eeprom[adr*2+1];
       switch(row%4)
       {
        case 0:
           ACC_row = a & 0x000f;
           break;
       case 1:
           ACC_row = a & 0x00f0;
           ACC_row = ACC_row / 16;
           break;
       case 2:
           ACC_row = a & 0x0f00;
           ACC_row = ACC_row / 256;
           break;
       case 3:
           ACC_row = a & 0xf000;
           ACC_row = ACC_row / 4096;
           break;
       default :
           ACC_row = a & 0x000f;
           printf("ACC OCC_row->this default can not!!\n\r");
           break;
       }

       if ( ACC_row > 7) ACC_row -=16;

       for (int col=0; col<NCOLS; col++) {
#ifdef PRNT_SENSITIVITY_param
          printf("Restore_SENSITIVITY_param for pix[%d,%d]\n\r",row,col);
          printf("row address[%d,%d]=%x, content :%x\n\r",row,col,adr+0x2400,a);
          printf("ACC_row[%d] = %d\n\r",row,ACC_row);
#endif
          // ACC_col
          adr1 = (((int)col/4)+0x2428)-0x2400;
          a = ((int)eeprom[adr1*2])*256+(int)eeprom[adr1*2+1];
#ifdef PRNT_SENSITIVITY_param
          printf("ACC_col address[%d,%d]=%x\n\r",row,col,adr1+0x2400);
#endif
          switch(col%4)
            {
            case 0:
                ACC_col = a & 0x000f;
                break;
            case 1:
                ACC_col = a & 0x00f0;
                ACC_col = ACC_col / 16;
                break;
            case 2:
                ACC_col = a & 0x0f00;
                 ACC_col = ACC_col / 256;
                break;
            case 3:
                ACC_col = a & 0xf000;
                 ACC_col = ACC_col / 4096;
                break;
            default :
                ACC_col = a & 0x000f;
                printf("ACC_col->this default can not!!\n\r");
                break;
          }

          if ( ACC_col > 7) ACC_col -=16;

#ifdef PRNT_SENSITIVITY_param
          printf("ACC_col[%d,%d] = %d, content:%x\n\r",row,col,ACC_col,a);
#endif

          // get alpha_pixel_row_col

          adr2 = ((row)*32+(col)+0x2440)-0x2400;
#ifdef PRNT_SENSITIVITY_param
          printf("pix-offset address[%d,%d]=%x\n\r",row,col,adr2+0x2400);
#endif
          a = ((int)eeprom[adr2*2])*256+(int)eeprom[adr2*2+1];
          int alpha_pixel_row_col= a & 0x03f0;

          alpha_pixel_row_col=  alpha_pixel_row_col/16;
          if ( alpha_pixel_row_col > 31) alpha_pixel_row_col -=64;

#ifdef PRNT_SENSITIVITY_param
          printf("alpha_pixel_row_col[%d,%d] = %d\n\r",row,col,alpha_pixel_row_col);
#endif
          Alpha[row][col]=(float)ACC_row*(float)pow(2.0,(double)ACC_scale_row)+(float)ACC_col*(float)pow(2.0,(double)ACC_scale_col);
          Alpha[row][col] = Alpha[row][col] + (float)pow(2.0,(double)ACC_scale_remnant)*(float)alpha_pixel_row_col;
          Alpha[row][col] = (Alpha[row][col]+(float)Alpha_ref)/(float)pow(2.0,(double)Alpha_scale);

#ifdef PRNT_SENSITIVITY_param
          printf("Alpha[%d,%d] = %e\n\r",row,col,Alpha[row][col]);
#endif

        } // for rows

     } // for cols
} // end Restore_SENSITIVITY_param

// ********************
// * Restore_Kv_param *
// ********************

void Restore_Kv_param()
{
    // Kv_odd_odd, Kv_even_odd, Kv_odd_even, Kv_even_even;
    int a,aa;
    float two_power_kv_scale;

    // Kv_scale
    a = ((int)eeprom[(0x2438-0x2400)*2])*256+(int)eeprom[(0x2438-0x2400)*2+1];
    a = (a & 0x0f00)/256;
    two_power_kv_scale= (float)pow(2.0,(double)a);

    // Kv_odd_odd
    a = ((int)eeprom[(0x2434-0x2400)*2])*256+(int)eeprom[(0x2434-0x2400)*2+1];
    aa = (a & 0xf000)/4096;
    if (aa > 7) aa=aa-16;
    Kv_odd_odd = (float)aa/two_power_kv_scale;
#ifdef PRNT_Kv_param
    printf("Kv_odd_odd=%f\n\r",Kv_odd_odd);
#endif

    // Kv_even_odd
    aa = (a & 0x0f00)/256;
    if (aa > 7) aa=aa-16;
    Kv_even_odd = (float)aa/two_power_kv_scale;
#ifdef PRNT_Kv_param
    printf("Kv_even_odd=%f\n\r",Kv_even_odd);
#endif
    // Kv_odd_even
    aa = (a & 0x00f0)/16;
    if (aa > 7) aa=aa-16;
    Kv_odd_even = (float)aa/two_power_kv_scale;
#ifdef PRNT_Kv_param
    printf("Kv_odd_even=%f\n\r",Kv_odd_even);
#endif
    // Kv_even_even
    aa = (a & 0x000f);
    if (aa > 7) aa=aa-16;
    Kv_even_even = (float)aa/two_power_kv_scale;
#ifdef PRNT_Kv_param
    printf("Kv_even_even=%f\n\r",Kv_even_even);
#endif

} // end Restore_Kv_param

// ****************
// * Get_Odd_Even *
// ****************

int Get_Odd_Even(int row,int col)
{
    if (row % 2){               // odd row
       if (col % 2) return 0;   // odd col
       else return 2;           // even col
    }
    else{                       // even row
      if (col % 2) return 1;    // odd col
       else return 3;           // even col
    }

}   // end Get_Odd_Even

// *********************
// * Restore_Kta_param *
// *********************

void Restore_Kta_param()
{

    // Kta_range_1[NROWS][NCOLS]
    float Kta_odd_odd, Kta_even_odd, Kta_odd_even, Kta_even_even;
    int a,aa, tmp_Kta;
    float two_power_kta_scale1,two_power_kta_scale2;

    // Kta_scale1
    a = ((int)eeprom[(0x2438-0x2400)*2])*256+(int)eeprom[(0x2438-0x2400)*2+1];
    aa = (a & 0x00f0)/16+8;
    two_power_kta_scale1= (float)pow(2.0,(double)aa);

    // Kta_scale2
    aa = (a & 0x000f);
    two_power_kta_scale2= (float)pow(2.0,(double)aa);

    // Kta_odd_odd
    a = ((int)eeprom[(0x2436-0x2400)*2])*256+(int)eeprom[(0x2436-0x2400)*2+1];

    aa = (a & 0xff00)/256;

    if (aa > 127) aa=aa-256;

    Kta_odd_odd = (float)aa;
#ifdef PRNT_Kta_param
    printf("Kta_odd_odd=%f\n\r",Kta_odd_odd);

#endif
    // Kta_even_odd
    aa = (a & 0x00ff);
    if (aa > 127) aa=aa-256;
    Kta_even_odd = (float)aa;
#ifdef PRNT_Kta_param
    printf("Kta_even_odd=%f\n\r",Kta_even_odd);
#endif
    // Kta_odd_even
    a = ((int)eeprom[(0x2437-0x2400)*2])*256+(int)eeprom[(0x2437-0x2400)*2+1];
    aa = (a & 0xff00)/256;
    if (aa > 127) aa=aa-256;
    Kta_odd_even = (float)aa;
#ifdef PRNT_Kta_param
    printf("Kta_odd_even=%f\n\r",Kta_odd_even);

#endif
    // Kta_even_even
    aa = (a & 0x00ff);
    if (aa > 127) aa=aa-256;
    Kta_even_even = (float)aa;
#ifdef PRNT_Kta_param
    printf("Kta_even_even=%f\n\r",Kta_even_even);
#endif

    for (int row=0; row<NROWS; row++) {
      for (int col=0; col<NCOLS; col++) {
         aa = ((row)*32+(col)+0x2440)-0x2400;
          a = ((int)eeprom[aa*2])*256+(int)eeprom[aa*2+1];
#ifdef PRNT_Kta_param
         printf("Kta_address[%d,%d]=%x, content=%x\n\r",row,col,aa+0x2400,a);
#endif

         int Kta= a & 0x000e;
#ifdef skip_outliers
        if ((a & 0x0001)==0x0001) {
            outliers_present =1;
            Kta_range_1[row][col] = -1;    // outlier
            OUTLIERS[ind][0]=row; OUTLIERS[ind][1]=col;
            if (pattern == LINE) {OUTLIERS[ind][2]=row%2;ind++;}
            else {OUTLIERS[ind][2]=(row+col)%2 ;ind++;}
            if (ind >=16) {printf("outlier count too high /n/r");//GET_A_KEY();
          }
            else continue;
         }
#endif
         Kta=  Kta/2;
         if (Kta > 3) Kta=Kta-8;
         aa = Get_Odd_Even(row+1,col+1);

         switch (aa){
             case 0:
               tmp_Kta = Kta_odd_odd;
              #ifdef PRNT_Kta_param
               printf("[%d,%d] = Kta_odd_odd\n\r",row,col);
              #endif
               break;
             case 1:
               tmp_Kta = Kta_even_odd;
               #ifdef PRNT_Kta_param
               printf("[%d,%d] = Kta_even_odd\n\r",row,col);
               #endif
               break;
             case 2:
               tmp_Kta = Kta_odd_even;
               #ifdef PRNT_Kta_param
               printf("[%d,%d] = Kta_odd_even\n\r",row,col);
               #endif
               break;
             case 3:
               tmp_Kta = Kta_even_even;
               #ifdef PRNT_Kta_param
               printf("[%d,%d] = Kta_even_even\n\r",row,col);
               #endif
               break;
             default:tmp_Kta = Kta_odd_odd;printf("Kta-> this may not happen\n\r");break;
         } // end switch aa
         Kta_range_1[row][col]= (tmp_Kta+(float)Kta*two_power_kta_scale2)/two_power_kta_scale1;

#ifdef PRNT_Kta_param
         printf("Kta[%d,%d] = %e\n\r",row,col,Kta_range_1[row][col]);
#endif
      } // end for col
    } // end for row

} // end Restore_Kta_param

// **********************
// * Restore_GAIN_param *
// **********************

void Restore_GAIN_param()
{
    // GainMeas_25_3v2
    int a;
    a = ((int)eeprom[(0x2430-0x2400)*2])*256+(int)eeprom[(0x2430-0x2400)*2+1];
    if (a > 32767) a=a-65536;
    GainMeas_25_3v2 = (float)a;
    #ifdef PRNT_GAIN_param
    printf("GainMeas_25_3v2 = %f\n\r",GainMeas_25_3v2);
    #endif
} // end Restore_GAIN_param

// **********************
// * Restore_KsTa_param *
// **********************

void Restore_KsTa_param()
{
    // KsTa
    int a;
    a = ((int)eeprom[(0x243c-0x2400)*2])*256+(int)eeprom[(0x243c-0x2400)*2+1];
    a = (a & 0xff00)/256;
    if (a > 127) a=a-256;
    KsTa = (float)a/8192.0;
    #ifdef PRNT_KsTa_param
    printf("KsTa = %f\n\r",KsTa);
    #endif
} // end Restore_KsTa_param

// **********************
// * Restore_KsTo_param *
// **********************

void Restore_KsTo_param()
{
    // KsTo
    int a,aa;
    a = ((int)eeprom[(0x243d-0x2400)*2])*256+(int)eeprom[(0x243d-0x2400)*2+1];
    a = (a & 0xff00)/256;
    if (a > 127) a=a-256;

    aa = ((int)eeprom[(0x243f-0x2400)*2])*256+(int)eeprom[(0x243f-0x2400)*2+1];
    aa = (aa & 0x000f)+8;

    KsTo = (float)a/(float)pow(2.0,(double)aa);
    #ifdef PRNT_KsTo_param
    printf("KsTo = %f\n\r",KsTo);
    #endif
} // end Restore_KsTo_param()

// *******************************
// * Restore_SENS_ALPHA_CP_param *
// *******************************

void Restore_SENS_ALPHA_CP_param()
{
     // ALPHA_CP_subpage_0
     // ALPHA_CP_subpage_1
    int a,aa,a_cp;
    // alpha cp scale
    a = ((int)eeprom[(0x2420-0x2400)*2])*256+(int)eeprom[(0x2420-0x2400)*2+1];
    a = (a & 0xf000)/4096+27; // scale

    // cp ratio
    aa = ((int)eeprom[(0x2439-0x2400)*2])*256+(int)eeprom[(0x2439-0x2400)*2+1];
    int ratio = (aa & 0xfc00);
    ratio = ratio /1024;

    if (ratio > 31) ratio=ratio-64;


    // ALPHA_CP_subpage_0
    a_cp = (aa & 0x03ff);
    ALPHA_CP_subpage_0= (float)a_cp/ (float)pow(2.0,(double)a);
    // ALPHA_CP_subpage_1
    ALPHA_CP_subpage_1 = ALPHA_CP_subpage_0*(float)(1+(float)ratio/128);

    #ifdef PRNT_SENS_ALPHA_CP_param
    printf("ratio = %d\n\r",ratio);
    printf("ALPHA_CP_subpage_0 = %e\n\r",ALPHA_CP_subpage_0);
    printf("ALPHA_CP_subpage_1 = %e\n\r",ALPHA_CP_subpage_1);
    #endif

} // end Restore_SENS_ALPHA_CP_param()

// ***************************
// * Restore_OFFSET_CP_param *
// ***************************

void Restore_OFFSET_CP_param()
{
    // OFFSET_CP_subpage_0
    // OFFSET_CP_subpage_1
    int a,aa;

    // OFFSET_CP_subpage_0
    a = ((int)eeprom[(0x243a-0x2400)*2])*256+(int)eeprom[(0x243a-0x2400)*2+1];
    aa = (a & 0x03ff);
    if (aa > 511) aa=aa-1024;
    OFFSET_CP_subpage_0 = (float)aa;

    //OFFSET_CP_subpage_1
    aa = (a & 0xfc00);
    aa = aa/1024;
    if (aa > 31) aa=aa-64;
    OFFSET_CP_subpage_1 = OFFSET_CP_subpage_0 + (float)aa;
    #ifdef PRNT_OFFSET_CP_param
    printf("OFFSET_CP_subpage_0 = %e\n\r",OFFSET_CP_subpage_0);
    printf("OFFSET_CP_subpage_1 = %e\n\r",OFFSET_CP_subpage_1);
    #endif

} // Restore_OFFSET_CP_param()

// ***********************
// * Restore_Kv_CP_param *
// ***********************

void Restore_Kv_CP_param()
{
    int a,aa;

    // KVcp_EE
    a = ((int)eeprom[(0x243b-0x2400)*2])*256+(int)eeprom[(0x243b-0x2400)*2+1];
    aa = (a & 0xff00)/256;
    if (aa > 127) aa=aa-256;
    // Kv scale
    a = ((int)eeprom[(0x2438-0x2400)*2])*256+(int)eeprom[(0x2438-0x2400)*2+1];
    a = (a & 0x0f00)/256;
    KVcp = (float)aa/(float)pow(2.0,(double)a);
    #ifdef PRNT_Kv_CP_param
    printf("KVcp = %e\n\r",KVcp);
    #endif
} // end

// ************************
// * Restore_KTa_CP_param *
// ************************

void Restore_KTa_CP_param()
{
    // Kta_cp
    int a,aa;

    // KTacp_EE
    a = ((int)eeprom[(0x243b-0x2400)*2])*256+(int)eeprom[(0x243b-0x2400)*2+1];
    aa = (a & 0x00ff);//256;
    if (aa > 127) aa=aa-256;

    // KTa_scale
    a = ((int)eeprom[(0x2438-0x2400)*2])*256+(int)eeprom[(0x2438-0x2400)*2+1];
    a = (a & 0x00f0)/16+8;

    Kta_cp = (float)aa/(float)pow(2.0,(double)a);
#ifdef PRNT_KTa_CP_param
    printf("Kta_cp = %e\n\r",Kta_cp);
#endif
} // end Restore_Kta_CP_param()

// *********************
// * Restore_TGC_param *
// *********************

void Restore_TGC_param()
{
    int a;
    // TGC
    a = ((int)eeprom[(0x243c-0x2400)*2])*256+(int)eeprom[(0x243c-0x2400)*2+1];
    a = (a & 0x00ff);
    if (a > 127) a=a-256;

    TGC = (float)a/32.0;
    #ifdef PRNT_TGC_param
    printf("TGC = %e\n\r",TGC);
    #endif
} // end Restore_TGC_param()

// **********************************
// Calculate VDD:
//      Gets: ram 0x072A data
//      Returns : calculated VDD
// **********************************

float Calc_Vdd(){

  read_mem( (char) 0x07, (char) 0x2a,(int) 2);
  int a1 = ((int)Ram_store[0])*256+(int)Ram_store[1];
  if (a1 > 32767) a1-=65536;

  g_Vdd = (((float)a1-(float)Vdd_25)/(float)Kv_Vdd)+(float)Vdd_V0;


#ifdef PRNT_Calc_Vdd
  printf("-VDD_ram %x\n\r",a1);
  printf("-Vdd_25 %f\n\r",Vdd_25);
  printf("-Kv_Vdd %f\n\r",Kv_Vdd);
  printf("-Vdd_V0 %f\n\r",Vdd_V0);
  printf("-VDD-calc %f\n\r",g_Vdd);
#endif
  return g_Vdd;
}


// ****************************************
// Calculate Kgain:
//      Gets:  contents ram addres 0x070e
//      Returns :  Kgain
// *****************************************

float Calc_Kgain()
{

  int a1;
  // get gain compensation
  read_mem( (char) 0x07, (char) 0x0a,(int) 2);
  a1 = ((int)Ram_store[0])*256+(int)Ram_store[1];
  if (a1 > 32767 ) a1-=65536;

  K_gain = GainMeas_25_3v2/(float)a1;
  return K_gain;

} // get_kgain

// *********************
// * Calculate Calc_To *
// *********************

float Calc_To(int x, int y, int v) // row, col, value pixel
{

 float Kv;
 float pixgain = ((float)v)*Kg;
 float pixosref = Pix_os_ref_range_1[x][y];

#ifdef PRNT_Calc_To
 printf("#Calc_To [%d,%d]->%d\n\r",x,y,v);
 printf("#pixgain ->%f\n\r",pixgain);
 printf("#pixosref ->%f\n\r",pixosref);
#endif

 int aa = Get_Odd_Even(x+1,y+1);
 switch (aa){
    case 0:Kv = Kv_odd_odd;break;
    case 1:Kv = Kv_even_odd;break;
    case 2:Kv = Kv_odd_even;break;
    case 3:Kv = Kv_even_even;break;
    default:
        Kv = Kv_odd_odd;
        printf("-Kv- THIS MAY NOT HAPPEN\n\r");
        break;
  } // end switch aa

  float pixos = pixgain-pixosref*(1+Kta_range_1[x][y]*(g_Ta-25.0))*(1+Kv*(g_Vdd-Vdd_V0));

#ifdef PRNT_Calc_To
  printf("#pixos ->%f\n\r",pixos);
  printf("#pix_gain_cp_sp0 ->%f\n\r",pix_gain_cp_sp0);
  printf("#pix_gain_cp_sp1 ->%f\n\r",pix_gain_cp_sp1);
#endif

  float pix_os_cp_sp0=pix_gain_cp_sp0-OFFSET_CP_subpage_0*(1+Kta_cp*(g_Ta-25.0))*(1+KVcp*(g_Vdd-Vdd_V0));
  float pix_os_cp_sp1=pix_gain_cp_sp1-OFFSET_CP_subpage_1*(1+Kta_cp*(g_Ta-25.0))*(1+KVcp*(g_Vdd-Vdd_V0));

#ifdef PRNT_Calc_To
  printf("#pix_os_cp_sp0 ->%f\n\r",pix_os_cp_sp0);
  printf("#pix_os_cp_sp1 ->%f\n\r",pix_os_cp_sp1);
#endif

  int ODD_en,EVEN_en;
  if (x % 2){ // odd row
       ODD_en = 1 ;
       EVEN_en = 0;
  }else{
       ODD_en = 0;
       EVEN_en = 1;
  }

  float vir_compensated = (pixos/emmisivity)-TGC*((float)ODD_en*(float)pix_os_cp_sp0+(float)EVEN_en*(float)pix_os_cp_sp1);

#ifdef PRNT_Calc_To
   printf("#vir_compensated ->%f\n\r",vir_compensated);
   printf("#pixos ->%f\n\r",pixos);
   printf("#TGC ->%f\n\r",TGC);
   printf("#ODD_en ->%d\n\r",ODD_en);
   printf("#EVEN_en ->%d\n\r",EVEN_en);
   printf("#pix_os_cp_sp0 ->%f\n\r",pix_os_cp_sp0);
   printf("#pix_os_cp_sp1 ->%f\n\r",pix_os_cp_sp1);
   printf("#TGC*.... ->%f\n\r",TGC*(ODD_en*pix_os_cp_sp0+EVEN_en*pix_os_cp_sp1));
#endif

  float Alpha_comp=(Alpha[x][y]-TGC*(ODD_en*ALPHA_CP_subpage_0+EVEN_en*ALPHA_CP_subpage_1));
  Alpha_comp=Alpha_comp*(1+KsTa*(g_Ta-25.0));
  float Tak4= (g_Ta+273.15);
  Tak4=Tak4*Tak4*Tak4*Tak4;
  double Sx = (Alpha_comp*Alpha_comp*Alpha_comp*vir_compensated)+Tak4*(Alpha_comp*Alpha_comp*Alpha_comp*Alpha_comp);

#ifdef PRNT_Calc_To
  printf("#Alpha_comp ->%e\n\r",Alpha_comp);
  printf("#Tak4 ->%e\n\r",Tak4);
  printf("#Sx ->%e\n\r",Sx);
#endif

  Sx=sqrt(Sx);
  Sx=sqrt(Sx)*KsTo;
  double TO = vir_compensated/(Alpha_comp*(1-KsTo*273.15)+Sx)+Tak4;
  TO =sqrt(TO);
  TO =sqrt(TO)-273.15;;

#ifdef PRNT_Calc_To
  printf("-Sx page 0 = %e\n\r",Sx);
  printf("pix->%d,%d:To = %e\n\r",x,y,TO);
#endif

  return (float)TO;
}  // end Calc_To

// *************
// * print_ram *
// *************

void print_ram()
{
    int i=0;
    printf("\n\r");
    for (i=0;i<NROWS*NCOLS*2;i++) {
        if (i%32 == 0) printf(" \n\raddr :%x ",i);
        printf("%x",ir_data[i]);
    }
    printf("\n\r");
}

// *************
// * Get_Image *
// *************

// please note :
// this routine needs to be adapted for the application program to capture the image while calculating the results, sync'd with the application


void Get_Image()
{
      Reset_New_Data_In_Ram2();

      // frame 0
      Measurement_done();
      (void)Calc_Vdd();
      g_Ta = Calc_Ta();
      get_array();

      print_ram();
      // if (frame != next_frame)  printf("FRAMES OUT OF SYNC!\r\n");
      if (frame == 0) {x = 0; next_frame = 1;}
      else{
          x=1;
          next_frame = 0;
      }
      // get first frame
      for (;x<NROWS;x=x+2) {
        for (y=0;y<NCOLS;y++) {
           int t=(int)(ir_data[(x*64)+(y*2)])*256+(int)(ir_data[(x*64)+(y*2)+1]);
           if (t > 32767 ) t-=65536;
           IMA[x][y]=(short)(Calc_To(x,y,t)*10.0);
        }
      }

      // get second frame

      Reset_New_Data_In_Ram2();   //re-set already for next frame
      Measurement_done();
      (void)Calc_Vdd();
      g_Ta = Calc_Ta();
      get_array();

      if (frame != next_frame)  printf("*"); //printf("FRAMES OUT OF SYNC!\r\n");
      if (frame == 0) {x = 0; next_frame = 1; }
      else {x=1;next_frame = 0;}

      for (;x<NROWS;x=x+2) {
        for (y=0;y<NCOLS;y++) {
           int t=(int)(ir_data[(x*64)+(y*2)])*256+(int)(ir_data[(x*64)+(y*2)+1]);
           if (t > 32767 ) t-=65536;
           IMA[x][y]=(short)(Calc_To(x,y,t)*10.0);
        }
      }

} // end Get_Image

// **************************
// * Reset_New_Data_In_Ram2 *
// **************************

void Reset_New_Data_In_Ram2()
{
    rd_mlx_status();
    status[1]=status[1]&0xf7;
    wr_mlx_status();
}

// ******************************
// * Set_Sensor_In_Line_mode()  *
// ******************************

void Set_Sensor_In_Line_mode()
{
     rd_mlx_config();
     reg_config[0] = reg_config[0] & 0x0f;
     wr_mlx_config();
} // end Set_Sensor_In_Line_mode

// ************************************
// * send IMAGE data to labview program *
// ************************************
// IMAGE buffer will be displayed

void display_IMAGE()
{
    int i,j;
    for (i=0; i<NROWS; i++) {
        printf("D%2d:",i);
        for (j=0; j<NCOLS; j++) {
            printf("%5d;",IMA[i][j]);
        }
        printf("\n");
    }
    printf("\n");
} // end display_IMAGE

void Prepaire_coeff()
{
    get_eeprom();
    printf("\n\r%s\r\n",SENSOR_ID);
    printf("\t%s\r\n",VERSION);

    set_IR_refresh_rate(refesh_rate);
    rd_mlx_config();
    analyse_config();
    outliers_present =0;

#ifdef CHESS_PAT
    Set_Sensor_In_Chess_mode();
    pattern=CHESS;
#else
    Set_Sensor_In_Line_mode();
    pattern=LINE;
#endif
    // max 16 outliers !, init them as nope
    for (int k=0;k<16;k++) OUTLIERS [k][0]=-1;

    // restore all eeprom parameters using DSversion 3 - feb 7, 2017
    Restore_VDD_param();
    Restore_TA_param();
    Restore_OFFSET_param();
    Restore_SENSITIVITY_param();
    Restore_Kv_param();
    Restore_Kta_param();
    Restore_GAIN_param();
    Restore_KsTa_param();
    Restore_KsTo_param();
    Restore_SENS_ALPHA_CP_param();
    Restore_OFFSET_CP_param();
    Restore_Kv_CP_param();
    Restore_KTa_CP_param();
    Restore_TGC_param();
} // end Prepaire_coeff()


// ******************************
// * Set_Sensor_In_Chess_mode() *
// ******************************

void Set_Sensor_In_Chess_mode()
{
     rd_mlx_config();
     reg_config[0] = reg_config[0] | 0x10;
     wr_mlx_config();
} // end Set_Sensor_In_Chess_mode

// *********************
// * Get_Image_Chess() *
// *********************

void Get_Image_Chess()
{
      int x,y;

      // frame 0
      Reset_New_Data_In_Ram2();
      Measurement_done();
      (void)Calc_Vdd();
      g_Ta = Calc_Ta();
      get_array();

      // frame 0
      int schaak_sw;

      if (frame != next_frame)printf("*");

      if (frame == 0) {next_frame =1; schaak_sw =0;}
      else {next_frame =0; schaak_sw =1;}

      for (x=0;x<NROWS;x=x+1) {
        for (y=schaak_sw;y<NCOLS;y=y+2) {
            int t=(int)(ir_data[(x*64)+(y*2)])*256+(int)(ir_data[(x*64)+(y*2)+1]);
            if (t > 32767 ) t-=65536;
            IMA[x][y]=(short)(Calc_To(x,y,t)*10.0);
        }
        if (schaak_sw ==1)schaak_sw = 0;
        else schaak_sw = 1;
      }

      // just to test the correctness of the sensor data and median filter is skipped
      // frame 1
      Reset_New_Data_In_Ram2();
      Measurement_done();
      (void)Calc_Vdd();
      g_Ta = Calc_Ta();
      get_array();

      if (frame != next_frame) printf("Sync prob\r\n");

      if (frame == 0) {next_frame =1;schaak_sw =0;}
      else  {next_frame =0;schaak_sw =1;}
      for (x=0;x<NROWS;x=x+1) {
         for (y=schaak_sw;y<NCOLS;y=y+2) {
           int t=(int)(ir_data[(x*64)+(y*2)])*256+(int)(ir_data[(x*64)+(y*2)+1]);
           if (t > 32767 ) t-=65536;
           IMA[x][y]=(short)(Calc_To(x,y,t)*10.0);
         }
         if (schaak_sw == 1) schaak_sw = 0;
         else schaak_sw = 1;
      }

} // end Get_Image_Median_Chess


short median(int n, short x[]) {
    float temp;
    int i, j;
    // the following two loops sort the array x in ascending order
    for(i=0; i<n-1; i++) {
        for(j=i+1; j<n; j++) {
            if(x[j] < x[i]) {
                // swap elements
                temp = x[i];
                x[i] = x[j];
                x[j] = temp;
            }
        }
    }
    if(n%2==0) {
        // if there is an even number of elements, return mean of the two elements in the middle
        return(short)((x[n/2] + x[n/2 - 1]) / 2.0);
    } else {
        // else return the element in the middle
        return (short)x[n/2];
    }
}

// ++++++++++++++++++++++++++++++
// + Get_Image_Median
// ++++++++++++++++++++++++++++++

void Get_Image_Median(int full_frame)
{

      short val[3];
      int x,y;
      int a1;
      int t;

      // frame 0
      // printf("Get_Image_Median 1770\r\n");

      Measurement_done();   // checks for new data bit and stes the frame odd or even value
      get_array();
      Kg = Calc_Kgain();
      (void)Calc_Vdd();
      g_Ta = Calc_Ta();


        // page 0
        read_mem( (char) 0x07, (char) 0x09,(int) 2);
        a1 = ((int)Ram_store[0])*256+(int)Ram_store[1];
        if (a1 > 32767 ) a1-=65536; // added on 18/04/2017

        //pix_gain_cp_sp0 = (float)a1*Kg;
        pix_gain_cp_sp0 = (pix_gain_cp_sp0*8+(float)a1*Kg)/9.0;

        //cyclops page 1
        read_mem( (char) 0x07, (char) 0x29,(int) 2);
        a1 = ((int)Ram_store[0])*256+(int)Ram_store[1];
        if (a1 > 32767 ) a1-=65536;
        //pix_gain_cp_sp1 = (float)a1*Kg;
        pix_gain_cp_sp1 = (pix_gain_cp_sp1*8+(float)a1*Kg)/9.0;


      if (frame != next_frame)printf("*");
      if (frame == 0) {x = 0;next_frame =1;}
      else {x=1;next_frame =0;}

      //Reset_New_Data_In_Ram2();  // start already new measurement

      for (;x<NROWS;x=x+2) {
        for (y=0;y<NCOLS;y++) {
           t=(int)(ir_data[(x*64)+(y*2)])*256+(int)(ir_data[(x*64)+(y*2)+1]);
           if (t > 32767 ) t-=65536;
           IMA[x][y]=(short)(Calc_To(x,y,t)*10.0);
        }
      }

      // just to test the correctness of the sensor data and median filter is skipped
      // frame 1
      if (full_frame == 1){
         Reset_New_Data_In_Ram2();
         Measurement_done();   // checks for new data bit and stes the frame odd or even value
         if (frame != next_frame) printf("Sync prob\r\n");
         get_array();
         Kg = Calc_Kgain();(void)Calc_Vdd();

         // page 0
         read_mem( (char) 0x07, (char) 0x09,(int) 2);
         a1 = ((int)Ram_store[0])*256+(int)Ram_store[1];
         if (a1 > 32767 ) a1-=65536;

         pix_gain_cp_sp0 = (float)a1*Kg;
         pix_gain_cp_sp0 = (pix_gain_cp_sp0*8+(float)a1*Kg)/9.0;

         //cyclops page 1
         read_mem( (char) 0x07, (char) 0x29,(int) 2);
         a1 = ((int)Ram_store[0])*256+(int)Ram_store[1];
         if (a1 > 32767 ) a1-=65536;
         //pix_gain_cp_sp1 = (float)a1*Kg;
         pix_gain_cp_sp1 = (pix_gain_cp_sp1*8+(float)a1*Kg)/9.0;

         if (frame == 0) {x = 0;next_frame = 1;}
         else  {x=1;next_frame = 0;}

         for (;x<NROWS;x=x+2) {
           for (y=0;y<NCOLS;y++) {
            t=(int)(ir_data[(x*64)+(y*2)])*256+(int)(ir_data[(x*64)+(y*2)+1]);

            if (t > 32767 ) t-=65536; // added on 18/04/2017
            IMA[x][y]=(short)(Calc_To(x,y,t)*10.0);
           }
         }

       if (outliers_present) {
         Correct_Outliers(0,0);
         Correct_Outliers(1,0);
       }
       return;   // no median is done then!!!
      } // end full_frame

    // end just to test the correctness of the sensor data and median filter is skipped
    // get rid of interleace

    if (frame == 0){ // {x = 1; }
        for (x=1;x<NROWS;x=x+2) {
         for (y=0;y<NCOLS;y++) {

          val[0]=IMA[x-1][y];
          if ((x+1) > NROWS )val[1]=0;
          else val[1]=IMA[x+1][y];
          val[2]= IMA[x][y];
          IMA[x][y]=median(3, val);
         }
        }
      } // if frame =0 // end if x==1
      else {
        for (x=NROWS-2;x>=0;x=x-2) {
         for (y=0;y<NCOLS;y++) {
          if ((x-1) < 0)val[0]= 0;
          else val[0]=IMA[x-1][y];
          val[1]=IMA[x+1][y];
          val[2]= IMA[x][y];
          IMA[x][y]=median(3, val);
         }
        }
      }// end if x ==0

   if (outliers_present) {

      Correct_Outliers(frame,0);
   }
} // end Get_Image_Median


// *************
// * Get_Image *
// *************
// please note :
// this routine needs to be adapted for the application program to capture the image while calculating the results, sync'd with the application

void Get_Image2(int a)
{
      Reset_New_Data_In_Ram2();

      // frame 0

      Measurement_done();
      (void)Calc_Vdd();
      g_Ta = Calc_Ta();
      get_array();
      // Reset_New_Data_In_Ram2();   //re-set already for next frame
      // if (frame != next_frame)  printf("FRAMES OUT OF SYNC!\r\n");
      if (frame == 0) {x = 0; next_frame = 1;}
      else{
          x=1;
          next_frame = 0;
      }
      // get first frame

      for (;x<NROWS;x=x+2) {
        for (y=0;y<NCOLS;y++) {
           int t=(int)(ir_data[(x*64)+(y*2)])*256+(int)(ir_data[(x*64)+(y*2)+1]);
           if (t > 32767 ) t-=65536;
           IMA[x][y]=(short)((x,y,t)*10.0);

        }
      }
      Reset_New_Data_In_Ram2();
      // get second frame
      if (a==0) return;
      Measurement_done();
      (void)Calc_Vdd();
      g_Ta = Calc_Ta();
      get_array();

      if (frame != next_frame)  printf("&");//printf("FRAMES OUT OF SYNC!\r\n");
      if (frame == 0) {x = 0; next_frame = 1; }
      else {x=1;next_frame = 0;}

      for (;x<NROWS;x=x+2) {
        for (y=0;y<NCOLS;y++) {
           int t=(int)(ir_data[(x*64)+(y*2)])*256+(int)(ir_data[(x*64)+(y*2)+1]);
           if (t > 32767 ) t-=65536;
           IMA[x][y]=(short)((x,y,t)*10.0);
        }
      }

} // end Get_Image2


void Get_cyclops_val_init()
{
  Reset_New_Data_In_Ram2();
  Measurement_done();
  Kg = Calc_Kgain();
  (void)Calc_Vdd();
  g_Ta = Calc_Ta();

  //Get cyclops CP0 and CP1



   // page 0
   read_mem( (char) 0x07, (char) 0x09,(int) 2);
   int a1 = ((int)Ram_store[0])*256+(int)Ram_store[1];
   if (a1 > 32767 ) a1-=65536;
   pix_gain_cp_sp0 = (float)a1*Kg;

   //cyclops page 1
   read_mem( (char) 0x07, (char) 0x29,(int) 2);
   a1 = ((int)Ram_store[0])*256+(int)Ram_store[1];
   if (a1 > 32767 ) a1-=65536;
   pix_gain_cp_sp1 = (float)a1*Kg;

} // end Get_cyclops_val_init


short Do_De_inter3(int x,int y,int frame)
{
    short val[6];
    if (frame ==0){
        if (x==0){
          if (y==NCOLS-1){val[0] = IMA[x][y-1];val[1]= IMA[x+1][y];val[2]= IMA[x][y];return median(3, val);}
          else{
              val[0]= IMA[x][y-1];
              val[1]= IMA[x+1][y];
              val[2]= IMA[x][y+1];
              val[3]= IMA[x][y];
              return median(4, val);
          }
        }
        if (x==NROWS-1){
            if (y ==0){val[0]= IMA[x-1][y];val[1]= IMA[x][y+1];val[2]= IMA[x][y+1];return median(3, val);}
            else {
              val[0]= IMA[x][y-1];
              val[1]= IMA[x-1][y];
              val[2]= IMA[x][y+1];
              val[3]= IMA[x][y];
              return median(4, val);
            }
        }
        if ( (y == 0)&& (x%2 )){ val[0]= IMA[x-1][y];val[1]= IMA[x+1][y];val[2]= IMA[x][y+1];val[3]= IMA[x][y];return median(4, val);}
        if ((y == NCOLS-1)&&(x%2)) {
          { val[0]= IMA[x][y-1];val[1]= IMA[x+1][y];val[2]= IMA[x-1][y];val[3]= IMA[x][y];return median(4, val);}
        }
        // all other cases
        {val[0]= IMA[x-1][y];val[1]= IMA[x+1][y];val[2]= IMA[x][y-1];val[3]= IMA[x][y+1];
         val[4]= IMA[x][y];val[5]= IMA[x][y];return median(6, val);}
      }// frame 0

     else{ // frame 1
        if (x==0){
          if (y==0){val[0] = IMA[x+1][y];val[1]= IMA[x][y+1];val[2]= IMA[x][y+1];return median(3, val);}
          else{
              val[0]= IMA[x][y-1];
              val[1]= IMA[x+1][y];
              val[2]= IMA[x][y+1];
              val[3]= IMA[x][y];
              return median(4, val);
          }
        }
        if (x==NROWS-1){
            if (y == NCOLS-1){val[0]= IMA[x-1][y];val[1]= IMA[x][y-1];val[2]= IMA[x][y];return median(3, val);}
            else {
              val[0]= IMA[x][y-1];
              val[1]= IMA[x-1][y];
              val[2]= IMA[x][y+1];
              val[3]= IMA[x][y];
              return median(4, val);
            }
        }
        if ((y == 0) && (x%2==0)){val[0]= IMA[x-1][y];val[1]= IMA[x+1][y];
           val[2]= IMA[x][y+1];val[3]= IMA[x][y];return median(4, val);}
        if ((y == NCOLS-1)&& (x%2==0)){
          {val[0]= IMA[x][y-1];val[1]= IMA[x-1][y];val[2]= IMA[x+1][y];val[3]= IMA[x][y];return median(4, val);}
        }
        // all other cases
        {val[0]= IMA[x-1][y];val[1]= IMA[x+1][y];val[2]= IMA[x][y-1];val[3]= IMA[x][y+1];
        val[4]= IMA[x][y];val[5]= IMA[x][y];
        return median(6, val);}
      }// frame 1
} // end Do_De_inter3

// ***************************************************************************************************************************
// * Get_Image_Median_Chess(a)                                                                                               *
// * if a == 0 : take chess patern images for frame 0 and frame1, results in a full image but with refresh rate divided by 2 *
// * if a == 1 : take chess patern images for frame 0 OR frame1, calculate the median get find the other frame values.       *
// ***************************************************************************************************************************

void Get_Image_Median_Chess(int de_inter)
{
      int x,y;
      short aa;

      Measurement_done();
      Kg = Calc_Kgain();
      (void)Calc_Vdd();
      g_Ta = Calc_Ta();

      get_array();

      //Get cyclops CP0 and CP1
        //page 0
      read_mem( (char) 0x07, (char) 0x09,(int) 2);
      int a1 = ((int)Ram_store[0])*256+(int)Ram_store[1];
      if (a1 > 32767 ) a1-=65536;

      pix_gain_cp_sp0 = (pix_gain_cp_sp0*8+(float)a1*Kg)/9.0;
        //cyclops page 1
      read_mem( (char) 0x07, (char) 0x29,(int) 2);
      a1 = ((int)Ram_store[0])*256+(int)Ram_store[1];
      if (a1 > 32767 ) a1-=65536;

      pix_gain_cp_sp1 = (pix_gain_cp_sp1*8+(float)a1*Kg)/9.0;

      Reset_New_Data_In_Ram2();  // start sensor already for next frame
      // frame 0
      int schaak_sw;

      if (frame != next_frame)printf("*");

      if (frame == 0) {next_frame =1; schaak_sw =0;}
      else {next_frame =0; schaak_sw =1;}

      int t;
      for (x=0;x<NROWS;x=x+1) {
        for (y=schaak_sw;y<NCOLS;y=y+2) {
            t=(int)(ir_data[(x*64)+(y*2)])*256+(int)(ir_data[(x*64)+(y*2)+1]);
            if (t > 32767 ) t-=65536;
           IMA[x][y]=(short)(Calc_To(x,y,t)*10.0);
        }
        if (schaak_sw ==1)schaak_sw = 0;
        else schaak_sw = 1;
      }

      if (de_inter==1){
        {
            if (frame ==1){ // subpage 1
              schaak_sw=0;
              for (x=0;x<NROWS;x=x+1)
                {
                     for (y=schaak_sw;y<NCOLS;y=y+2){
                         aa=Do_De_inter3(x,y,1);
                         if ( abs((float)aa  - (float)IMA [x][y]) > 7)
                            IMA [x][y]= aa;
                 }
                 if (schaak_sw==0) schaak_sw=1; else schaak_sw=0;
                }
           }
            else{ // subpage 0
              schaak_sw=1;
              for (x=0;x<NROWS;x=x+1) {
                  for (y=schaak_sw;y<NCOLS;y=y+2) {
                      aa=Do_De_inter3(x,y,0);
                      if ( abs((float)aa  - (float)IMA [x][y]) > 7)
                         IMA [x][y]=aa;
                     }
                  if (schaak_sw==1) schaak_sw=0; else schaak_sw=1;
              }
            }
        }
        if (outliers_present) {
            Correct_Outliers(frame,1);  // frame, chess
        }
        return;
      }

      // frame 1

      Measurement_done();
      Kg = Calc_Kgain();
      (void)Calc_Vdd();
      g_Ta = Calc_Ta();

      get_array();
       Reset_New_Data_In_Ram2();  // start sensor already for next frame
      //Get cyclops CP0 and CP1
         //page 0
      read_mem( (char) 0x07, (char) 0x09,(int) 2);
      a1 = ((int)Ram_store[0])*256+(int)Ram_store[1];
      if (a1 > 32767 ) a1-=65536;

      pix_gain_cp_sp0 = (pix_gain_cp_sp0*8+(float)a1*Kg)/9.0;
         //cyclops page 1
      read_mem( (char) 0x07, (char) 0x29,(int) 2);
      a1 = ((int)Ram_store[0])*256+(int)Ram_store[1];
      if (a1 > 32767 ) a1-=65536;

      pix_gain_cp_sp1 = (pix_gain_cp_sp1*8+(float)a1*Kg)/9.0;

      if (frame != next_frame) printf("Sync prob\r\n");

      if (frame == 0) {next_frame =1;schaak_sw =0;}
      else  {next_frame =0;schaak_sw =1;}
      for (x=0;x<NROWS;x=x+1) {
         for (y=schaak_sw;y<NCOLS;y=y+2) {
           int t=(int)(ir_data[(x*64)+(y*2)])*256+(int)(ir_data[(x*64)+(y*2)+1]);
           if (t > 32767 ) t-=65536;
           IMA[x][y]=(short)(Calc_To(x,y,t)*10.0);
         }
         if (schaak_sw == 1) schaak_sw = 0;
         else schaak_sw = 1;
      }
      if (outliers_present) {
            Correct_Outliers(0,1);  // frame, chess
            Correct_Outliers(1,1);  // frame, chess
        }
} // end Get_Image_Median_Chess


void Correct_Outliers(int fr,int pat)
{
   int indx=0,x,y;
   short val[4];

   for (indx=0;indx<16;indx++) {
     if (OUTLIERS[indx][2] == fr){
       if (OUTLIERS [indx][0]!= -1){ // update pixel value with neighbors values
        x = OUTLIERS[indx][0]; y = OUTLIERS[indx][1];

        if (pat ==1 ) { // chess
            if ( (y>=1) && (y<=NCOLS-2) && (x>=1) && (x<=NROWS-2) ){ // normal pattern
              val[0]=IMA[x-1][y-1]; val[1]= IMA[x-1][y+1]; val[2]= IMA[x+1][y-1]; val[3]=IMA[x+1][y+1];
              IMA[x][y] = median(4, val);
            }
            else { // boundaries
               IMA[x][y] = Pat_Boundaries(x,y);
            }
        }
        else { // pat ==0 interlaced
           if ((y>=2) && (y<=NCOLS-2)){ // normal pattern
               if (abs((IMA[x][y-2]-IMA[x][y-1])) > abs((IMA[x][y+2]-IMA[x][y+1]))){
                   IMA[x][y] = 2*IMA[x][y+1]-IMA[x][y+2];
               }
               else{
                   IMA[x][y] = 2*IMA[x][y-1]-IMA[x][y-2];
               }
           }
           else { // boundary matters
             if (y<=1){IMA[x][y] = 2*IMA[x][y+1]-IMA[x][y+2];}
             if (y>=NCOLS-2){IMA[x][y] = 2*IMA[x][y-1]-IMA[x][y-2];}
           }
         }
        } // end if outlier
     } // if frame is correct
   } // end for

} // end Correct_Outliers

short Pat_Boundaries(int x,int y)
{
  short val[3];
  if (x==0){
    if (y==0) return IMA[x+1][y+1];
    else if ( y== (NCOLS-1)) return IMA[x+1][y-1];
    else {val[0]=IMA[x+1][y-1];val[1]=IMA[x+1][y+1];return(median(2, val));}
  }
  else if (x == NROWS-1){
    if (y==0) return IMA[x-1][y-1];
    else if ( y== (NCOLS-1)) return IMA[x-1][y-1];
    else {val[0]=IMA[x-1][y-1];val[1]=IMA[x-1][y+1];return(median(2, val));}
  }
  else if (y ==0){
      if (x ==0){return IMA[x+1][y+1];}
      else if (x == NCOLS-1){return IMA[x-1][y+1];}
      else{val[0]=IMA[x-1][y+1];val[1]=IMA[x+1][y+1];return(median(2, val));}
  }
  else if (y == NCOLS-1){
      if (x ==0){return IMA[x+1][y-1];}
      else if (x == NCOLS-1){return IMA[x-1][y-1];}
      else{val[0]=IMA[x-1][y-1];val[1]=IMA[x+1][y-1];return(median(2, val));}
  }
  else return -2500; // should not happen
} // end Pat_Boundaries
