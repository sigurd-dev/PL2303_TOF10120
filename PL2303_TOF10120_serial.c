/* 
   Example program to read distances from TOF10120 Laser Range Sensor in Linux.
   If using a PL2303 Serial Port (USB To RS232 TTL PL2303HX Converter link below)
   connect wires like this:
   Red->3V
   Yellow->TDX
   White->RDX
   Black->GND 
   (Blue and Green are not used) 

   Compile with: 
   gcc PL2303_TOF10120_serial.c strrep.c -o PL2303_TOF10120_serial
   or use included makefile.

   The rather cheap hardware can be bought here:
   TOF10120: https://www.aliexpress.com/item/4001120526796.html?spm=a2g0s.9042311.0.0.27424c4dDGURpH
   USB To RS232 TTL PL2303HX: https://www.aliexpress.com/item/4001134803817.html?spm=a2g0s.9042311.0.0.27424c4dDGURpH
   Sigurd Dagestad, 2020
*/

#include <errno.h>
#include <termios.h>
#include <unistd.h>

#include <fcntl.h>

//atof blant annet.
#include <stdlib.h>

//For string replacement.
#include "strrep.h"


int set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                //error_message ("error %d from tcgetattr", errno);
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                //error_message ("error %d from tcsetattr", errno);
                return -1;
        }
        return 0;
}

void set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                //error_message ("error %d from tggetattr", errno);
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
               printf("error %d setting term attributes", errno); 
               //error_message ("error %d setting term attributes", errno);
        }
}


char *portname = "/dev/ttyUSB0";

int main(int argc, char *argv[])
{
  int runs=0; //run forever
  if(argc >1)
    runs=atoi(argv[1]); 

  int fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
  if (fd < 0)
  {
          //error_message ("error %d opening %s: %s", errno, portname, strerror (errno));
          return;
  }

  //B115200, B230400, B9600, B19200, B38400, B57600, B1200, B2400, B4800
  set_interface_attribs (fd, B9600, 0);  // set speed to 115,200 bps, 8n1 (no parity)
  set_blocking (fd, 0);                // set no blocking

  write(fd, "s", 1); 
  write(fd, "5", 1);
  write(fd, "-", 1);
  write(fd, "1", 1);
  write(fd, "#", 1); 
  
  usleep(10);

  unsigned char buf[24], rxbuf[24],rxempty, rxcnt,rxflag,waitflag;
  unsigned short int length_val;
  rxcnt=0;
  int i;

  float avg;
  int high, low;
  high = 0;
  low = 999999; 
  int avgruns = 0;

  while(1) 
  {           

   write(fd, "r", 1); 
   write(fd, "6", 1);
   write(fd, "#", 1);  
   int n = read (fd, rxbuf, sizeof rxbuf);     
   usleep(40000);                 

   //Read ten lines which can have a little bogus data. 
   i++;
   if(i <= 10) continue;

   char * newbuffer;
   newbuffer = strrep(rxbuf, "L=", ""); 
   newbuffer = strrep(newbuffer, "mm", ""); 
   newbuffer = strrep(newbuffer, "\r\n", "");
   newbuffer = strrep(newbuffer, "\n", "");
   if(strcmp(newbuffer, "") == 0) continue;

   avgruns++;
   avg=(avg+atof(newbuffer));
   if(low > atoi(newbuffer)) low=atoi(newbuffer);
   if(high < atoi(newbuffer)) high=atoi(newbuffer);
   
     
   printf("run: %d #, rxbuf: %s mm, avg: %f mm, low: %d mm, high: %d mm\n", avgruns, newbuffer, avg/avgruns, low, high);
   //printf("rxbuf: %s", newbuffer);
   if(avgruns == runs) break;
 
 }
 printf("avg %f\n", avg/avgruns); 

}
