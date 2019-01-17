#define _GLIBCXX_USE_CXX11_ABI 0

#include <dirent.h>
#include <sys/types.h>

#include "math.h"
#include "defines.h"
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <math.h>
#include <signal.h>
#include <bcm2835.h>
#include <errno.h>
#include <string.h>
#include <sys/wait.h>
#include <fstream>
#include <iterator>
#include <vector>
#include <sstream>
#include <iostream>
#include <time.h>
#include <cstdlib>



extern void Prepaire_coeff();
extern void Reset_New_Data_In_Ram2();
extern void Get_Image2(int);
extern void GET_A_KEY();
extern void Get_Image_Median(int);
extern void Get_cyclops_val_init();
extern void Get_Image_Median_Chess(int de_inter);
extern void print_ram();

extern char     SENSOR_ID[];
extern short    IMA[NROWS][NCOLS];
//double          NORMALIZED[NROWS][NCOLS];
extern int      i,j,k;
extern unsigned char       ir_data[NROWS*NCOLS*2+2];   // toke a few bytes more to be sure!

// for file parameter read

char            refesh_rate = '0';


char            mlxFifo[] = "/var/run/mlx90640.sock";

int             num, fd, sleepinsec;

pid_t           pid;


// *************
// * FUNCTIONS *
// *************

// ----------------------------------------------------------------------------
/**
 * Represents a single Pixel in the image. A Pixel has red, green, and blue
 * components that are mixed to form a color. Each of these values can range
 * from 0 to 255
**/
class Pixel
{
public:
	// Stores the individual color components.
	int red, green, blue;

	// Initializes a Pixel with a default black color.
	Pixel() : red(0), green(0), blue(0) { }

	// Initializes a color Pixel with the specified RGB values.
	Pixel(int r, int g, int b) : red(r), green(g), blue(b) { }
};




typedef Pixel pixelArray[NROWS][NCOLS];

typedef double array2d[NROWS][NCOLS];

array2d* NormaliseValue()
{
  double returnValues[NROWS][NCOLS];
  //return (double)(Value - MINTEMP) / (double)(MAXTEMP - MINTEMP);

  for (i=0; i<NROWS; i++) {
      for (j=0; j<NCOLS; j++) {
        returnValues[i][j] = (double)(IMA[i][j] - MINTEMP) / (double)(MAXTEMP - MINTEMP);
      }
  }

  for (i=0; i<NROWS; i++) {
      //pc.printf("D%2d:",i);
      for (j=0; j<NCOLS; j++) {
        printf("%5.3f;", returnValues[i][j]);
      }
       printf("\n\r");
  }
  return &returnValues;
}

void writeBMP(unsigned char **Matrix, std::string filename ){
    FILE *out;
    int ii,jj;
    long pos = 1077;


    out = fopen(filename.c_str(),"wb");


    // Image Signature
    unsigned char signature[2] = {'B','M'};
    fseek(out,0,0);
    fwrite(&signature,2,1,out);


    // Image file size
    uint32_t filesize = 54 + 4*256 + NROWS*NCOLS;
    fseek(out,2,0);
    fwrite(&filesize,4,1,out);


    // Reserved
    uint32_t reserved = 0;
    fseek(out,6,0);
    fwrite(&reserved,4,1,out);


    // Offset
    uint32_t offset = 1078;
    fseek(out,10,0);
    fwrite(&offset,4,1,out);


   // Info header size
    uint32_t ihsize = 40;
    fseek(out,14,0);
    fwrite(&ihsize,4,1,out);


    // Image Width in pixels
    uint32_t width = (uint32_t) NCOLS;
    fseek(out,18,0);
    fwrite(&width,4,1,out);


    // Image Height in pixels
    uint32_t height = (uint32_t) NROWS;
    fseek(out,22,0);
    fwrite(&height,4,1,out);


    // Number of planes
    uint16_t planes = 1;
    fseek(out,26,0);
    fwrite(&planes,2,1,out);


    // Color depth, BPP (bits per pixel)
    uint16_t bpp = 8;
    fseek(out,28,0);
    fwrite(&bpp,2,1,out);


    // Compression type
    uint32_t compression = 0;
    fseek(out,30,0);
    fwrite(&compression,4,1,out);


    // Image size in bytes
    uint32_t imagesize = (uint32_t) NROWS*NCOLS;
    fseek(out,34,0);
    fwrite(&imagesize,4,1,out);


    // Xppm
    uint32_t xppm = 0;
    fseek(out,38,0);
    fwrite(&xppm,4,1,out);


    // Yppm
    uint32_t yppm = 0;
    fseek(out,42,0);
    fwrite(&yppm,4,1,out);


    // Number of color used (NCL)
    uint32_t colours = 256;
    fseek(out,46,0);
    fwrite(&colours,4,1,out);


    // Number of important color (NIC)
    // value = 0 means all collors important
    uint32_t impcolours = 0;
    fseek(out,50,0);
    fwrite(&impcolours,4,1,out);


    // Colour table
    unsigned char bmpcolourtable[1024];
    for(ii=0; ii < 1024; ii++){
        bmpcolourtable[ii] =  0;
    }
    jj=3;
    for(ii=0; ii < 255; ii++){
        bmpcolourtable[jj+1] =  ii+1;
        bmpcolourtable[jj+2] =  ii+1;
        bmpcolourtable[jj+3] =  ii+1;
        jj=jj+4;
    }


    fseek(out,54,0);
    fwrite(&bmpcolourtable,256,4,out);


    for(ii=0;ii< NROWS;ii++){
        for(jj=0;jj<NCOLS;jj++){
            pos+= 1;
            fseek(out,pos,0);
            fwrite(&Matrix[ii][jj],(sizeof(unsigned char)),1,out);
        }
    }


    fflush(out);
    fclose(out);
}

tm operator+ ( tm uct, int span_in_minutes )
{
    uct.tm_min += span_in_minutes ;
    const auto t = mktime(std::addressof(uct) ) ;

    return *localtime(std::addressof(t) ) ;
}

tm operator+ ( int span_in_minutes, tm uct ) { return uct + span_in_minutes ; }
tm operator- ( tm uct, int span_in_minutes ) {  return uct + -span_in_minutes ; }



const std::string currentDateTime() {
    time_t     now = time(0);
    struct tm  tstruct;
    /*tm* localtm = localtime(&now);
    printf("time was: %s\n\r", asctime(localtm));*/
    char       buf[80];
    tstruct = *localtime(&now);
    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
    // for more information about date/time format
    strftime(buf, sizeof(buf), "%Y%m%d%H%M%S", &tstruct);
    printf("time was: %s\n\r", &buf);
    std::string stt(buf);
    return stt;
    //return buf;
}


const std::string OneHoureBackTime() {
    time_t     now = time(0);
    struct tm tstruct, onhback;
    /*tm* localtm = localtime(&now);
    printf("time was: %s\n\r", asctime(localtm));*/
    char       buf[80];
    tstruct = *localtime(&now);
    onhback = tstruct-60 ;
    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
    // for more information about date/time format
    strftime(buf, sizeof(buf), "%Y%m%d%H%M%S", &onhback);
    printf("time was: %s\n\r", &buf);
    std::string stt(buf);
    return stt;
    //return buf;
}

pixelArray* Colorise()
{
  Pixel returnValues[NROWS][NCOLS];

//************************************This saves file to local disk (rgb.txt file to img folder) *****************************************************//
	std::ostringstream RGBfile;
	RGBfile << "./img/" << currentDateTime() << ".rgb.txt";
  std::ofstream myRGBfile (RGBfile.str(), std::ios::out | std::ios::app);

  for (i=0; i<NROWS; i++) {
      for (j=0; j<NCOLS; j++) {
        double x = abs((double)(IMA[i][j] - MINTEMP) / (double)(MAXTEMP - MINTEMP));
				if(x < 0.5)
        {
					int green = ((int)floor((x*2*255)));
					if(green > 255)
						green = 255;
					if(green < 0)
							green = 0;
					int blue = ((int)floor(((1-(x*2))*255)));
					if(blue > 255)
							blue = 255;
					if(blue < 0)
							blue = 0;
	        returnValues[i][j] = Pixel(0, green, blue);
        }
				else if(x >= 0.5)
        {
					int red = ((int)floor(((x-0.5)*2*255)));
					if(red > 255)
						red = 255;
					if(red < 0)
						red = 0;

					int green = ((int)floor(((1-((x-0.5)*2))*255)));
					if(green > 255)
							green = 255;
					if(green < 0)
							green = 0;

          returnValues[i][j] = Pixel(red, green, 0);
        }
      }
  }
	if (myRGBfile.is_open())
	{
		myRGBfile << "[";
		printf("[");
	}
	for (i=0; i<NROWS; i++) {
			//pc.printf("D%2d:",i);
			if (myRGBfile.is_open())
			{
				myRGBfile << "[";
				printf("[");
			}
			for (j=0; j<NCOLS; j++) {
				//out+=IMA[i][j];
				//ss <<"%5d;" << IMA[i][j];
				if (myRGBfile.is_open())
				{
					if(j < NCOLS-1)
					{
						myRGBfile << "("<<returnValues[i][j].red <<","<< returnValues[i][j].green << ","<<returnValues[i][j].blue << "),";
						printf("(%d,%d,%d),", returnValues[i][j].red,returnValues[i][j].green,returnValues[i][j].blue);
					}
					else
					{
						myRGBfile << "("<<returnValues[i][j].red <<","<< returnValues[i][j].green << ","<<returnValues[i][j].blue << ")";
						printf("(%d,%d,%d)", returnValues[i][j].red,returnValues[i][j].green,returnValues[i][j].blue);
					}

				}
			}

				if (myRGBfile.is_open())
				{
					if(i < NROWS-1)
					{
						myRGBfile << "],\n\r";
						printf("],\n\r");
					}
					else
					{
						myRGBfile << "]";
						printf("]");
					}
				}

	}
	if (myRGBfile.is_open())
	{
		myRGBfile << "]";
		printf("]");
		myRGBfile.close();
	}
	printf("\n\r");

	//*************************************************************************************//

  /*for (i=0; i<NROWS; i++) {
      //pc.printf("D%2d:",i);
      for (j=0; j<NCOLS; j++) {
        printf("[%d,%d,%d];", returnValues[i][j].red,returnValues[i][j].green,returnValues[i][j].blue);
      }
       printf("\n\r");
  }*/
  return &returnValues;
}

void display_Ima()
{

  int             fd, sleeper;
  std::ostringstream tempfile, bmpFileName; //rawfile,
  std::string oneHBack = OneHoureBackTime();
  tempfile << "./img/" << currentDateTime() << ".temp.txt";
  bmpFileName << "./img/" << currentDateTime() << ".temp.bmp";
  //rawfile << "./img/" << currentDateTime() << ".raw.txt";

  std::ofstream mytempfile (tempfile.str(), std::ios::out | std::ios::app);
  //std::ofstream myrawfile (rawfile.str(), std::ios::out | std::ios::app);

  /*printf("************RAW DATA************\n\r");
  int i=0;
  printf("\n\r");
  for (i=0;i<NROWS*NCOLS*2;i++) {
      if (i%32 == 0) printf(" \n\raddr :%x ",i);
      printf("%5x",ir_data[i]);
  }
  printf("\n\n\r");*/

  ///std::string out;
  static array2d *NORMALIZED;
  pixelArray *COLORIZED;
  printf("************NORMALIZED Data***********\n\r");
  NORMALIZED = NormaliseValue();
  printf("************NORMALIZED Data: %.3f ***********\n\r", *NORMALIZED+6);

	//************************************This saves file to local disk (temp.txt file to img folder) *****************************************************//

  printf("************Temp Data***********\n\r");
  printf("PRINTING IMAGE FROM DISPALY IMAGE MAIN \n\n\r");
  //currentDateTime();
  for (i=0; i<NROWS; i++) {
      //pc.printf("D%2d:",i);
      for (j=0; j<NCOLS; j++) {
        //out+=IMA[i][j];
        //ss <<"%5d;" << IMA[i][j];
        printf("%5d;", IMA[i][j]);

        if (mytempfile.is_open())
        {
          mytempfile << IMA[i][j] << " ";
          //myfile.close();
        }
        //printf("Print from ss %5d;",ss.str());
      }
       if (mytempfile.is_open())
       {
         mytempfile << "\n\n\r";

       }
       printf("\n\r");
  }
  if (mytempfile.is_open())
    mytempfile.close();
  printf("\n\r");
	//*****************************************************************************************//

  printf("************COLORIZED Data***********\n\r");
  printf("PRINTING IMAGE FROM DISPALY IMAGE MAIN \n\n\r");
  COLORIZED = Colorise();


//********************************************This will remove the data that is 1 h old ***********************************///
#ifdef REMOVE_1H_OLD_FILES
  DIR* dp;
  struct dirent* ep;
  char* path = "/home/pi/90640-v3-driver/img/";
  dp = opendir(path);
  if (dp != NULL)
  {
    printf("Dir content:\n");
    while(ep = readdir(dp))
    {
      std::string name(ep->d_name);
      if((name.substr(name.find_first_of(".") + 1) == "temp.txt")||(name.substr(name.find_first_of(".") + 1) == "raw.txt")||(name.substr(name.find_first_of(".") + 1) == "rgb.txt"))
      {
				std::string::size_type sz;   // alias of size_t
				long long intstamp = std::stoll(name.substr(0,name.find_first_of(".")),&sz);
				long long onehbacktstamp = std::stoll(oneHBack, &sz);
				//printf("bla: %5lld \t", onehbacktstamp);
				if(intstamp < onehbacktstamp)
				{
				  char* filename;
				  filename = (char *)malloc(strlen(path)+strlen(ep->d_name)+1); /* make space for the new string (should check the return value ...) */
				  strcpy(filename, path); /* copy name into the new var */
				  strcat(filename, ep->d_name); /* add the extension */

				  printf("removing %s file name was: %s\n",ep->d_name, filename );
				  if(remove(filename)!=0)
					printf("file remove error\n");
				  else
					printf("file removed\n");
				}
	//printf("%5s\t",name);
      }
    }
  }
  closedir(dp);
#endif
//*******************************************************************************///

  printf("\n\r");
  sleeper = 1000000*sleepinsec;
  if(sleeper > 1500000)
    sleeper-=1500000;
  usleep(sleeper);

} // end display_Ima


// ++++++++++++++++++++++++
// main  application loop
// ++++++++++++++++++++++++


void find_objects()
{


    // get cyclops values to start for averaging them
    Get_cyclops_val_init();

    Reset_New_Data_In_Ram2();

#ifdef CHESS_PAT
    // get the thermal image without de-interlace techniques
    Get_Image_Median_Chess(0);
    printf("chess P\n\r");
#else
    Get_Image_Median(1);       // collect an full image using the odd and even frames wait for both odd and even image
    printf("None chess P\n\r");
#endif

    (void) display_Ima();

    printf("REady to start\n\r");//GET_A_KEY();

    Reset_New_Data_In_Ram2();

    while (1){

#ifdef CHESS_PAT
                Get_Image_Median_Chess(1);
#else
                Get_Image_Median(0);
#endif

               (void) display_Ima();
                //GET_A_KEY();
    } // end while

}  // end find objects


// *****************
// * END FUNCTIONS *
// *****************

// ================
// = main routine =
// ================
const char *defaults[] = { "5"};
int main(int argc =2, const char* argv[] =defaults)
{
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " Sleep in secounds" << std::endl;
        return 1;
    }
    else
    {
      std::istringstream ss(argv[1]);

      if (!(ss >> sleepinsec)){
          std::cerr << "Invalid number " << argv[1] << '\n';
          std::cerr << "Usage: " << argv[0] << " Sleep in secounds <-THIS MEANS NUMBERS ONLY !" << std::endl;
          return 1;
        }
    }
    int x,y;
    if (!bcm2835_init())
    {
      printf("ERROR: CANNOT INIT bcm2835 \n\r");
      return 0;
    }
    bcm2835_i2c_begin();

    bcm2835_i2c_set_baudrate(25000);

    Prepaire_coeff();  // get and prepare all eeprom calibration coeff's
    printf("Note: due to the slow To screen print of each pixel\n\r");
    printf("Image sync is lost, fix speed in your application!\n\r");
    //GET_A_KEY();
    find_objects();     // call to top level routine
    //End i2c
    bcm2835_i2c_end();
} // end main
