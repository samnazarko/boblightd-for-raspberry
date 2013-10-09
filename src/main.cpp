/*
 * boblight
 * Copyright (C) Bob  2009
 *
 * //////////////
 * Modded By Speedy1985 and Heven (C) 2013
 * //////////////
 *
 * boblight is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * boblight is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#define BCM2708_PERI_BASE        0x20000000
#define UART0_BASE               (BCM2708_PERI_BASE + 0x201000) /* Uart 0 */
#define UART1_BASE               (BCM2708_PERI_BASE + 0x215000) /* Uart 1 */
#define MCORE_BASE               (BCM2708_PERI_BASE + 0x0000)   /* Fake frame buffer device */
#define GPIO_BASE                (BCM2708_PERI_BASE + 0x200000) /* GPIO controller */
#define SPI0_BASE                (BCM2708_PERI_BASE + 0x204000) /* SPI0 controller */

#include <iostream>
#include <string>
#include <signal.h>
#include <unistd.h>
#include <sstream>
#include <stdlib.h>


#include "util/log.h"
#include "util/tcpsocket.h"
#include "util/messagequeue.h"
#include "client.h"
#include "configuration.h"
#include "device/device.h"
#include "config.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <dirent.h>
#include <fcntl.h>
#include <assert.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#define DEFAULTCONF "/etc/boblight.conf"

#define PAGE_SIZE (4*1024)
#define BLOCK_SIZE (4*1024)

int  mem_fd;
char *gpio_mem, *gpio_map;
char *spi0_mem, *spi0_map;


// I/O access
volatile unsigned *gpio;
volatile unsigned *spi0;


// SPI operation

// GPIO setup macros. Always use INP_GPIO(x) before using OUT_GPIO(x) or SET_GPIO_ALT(x,y)
#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))

//
#define SPI0_CNTLSTAT *(spi0 + 0)
#define SPI0_FIFO     *(spi0 + 1)
#define SPI0_CLKSPEED *(spi0 + 2)

// SPI0_CNTLSTAT register bits

#define SPI0_CS_CS2ACTHIGH   0x00800000 // CS2 active high
#define SPI0_CS_CS1ACTHIGH   0x00400000 // CS1 active high
#define SPI0_CS_CS0ACTHIGH   0x00200000 // CS0 active high
#define SPI0_CS_RXFIFOFULL   0x00100000 // Receive FIFO full
#define SPI0_CS_RXFIFO3_4    0x00080000 // Receive FIFO 3/4 full
#define SPI0_CS_TXFIFOSPCE   0x00040000 // Transmit FIFO has space
#define SPI0_CS_RXFIFODATA   0x00020000 // Receive FIFO has data
#define SPI0_CS_DONE         0x00010000 // SPI transfer done. WRT to CLR!
#define SPI0_CS_MOSI_INPUT   0x00001000 // MOSI is input, read from MOSI (BI-dir mode)
#define SPI0_CS_DEASRT_CS    0x00000800 // De-assert CS at end
#define SPI0_CS_RX_IRQ       0x00000400 // Receive irq enable
#define SPI0_CS_DONE_IRQ     0x00000200 // irq when done
#define SPI0_CS_DMA_ENABLE   0x00000100 // Run in DMA mode
#define SPI0_CS_ACTIVATE     0x00000080 // Activate: be high before starting
#define SPI0_CS_CS_POLARIT   0x00000040 // Chip selects active high
#define SPI0_CS_CLRTXFIFO    0x00000020 // Clear TX FIFO    (auto clear bit)
#define SPI0_CS_CLRRXFIFO    0x00000010 // Clear RX FIFO    (auto clear bit)
#define SPI0_CS_CLRFIFOS     0x00000030 // Clear BOTH FIFOs (auto clear bit)
#define SPI0_CS_CLK_IDLHI    0x00000008 // Clock pin is high when idle
#define SPI0_CS_CLKTRANS     0x00000004 // 0=first clock in middle of data bit
                                        // 1=first clock at begin of data bit
#define SPI0_CS_CHIPSEL0     0x00000000 // Use chip select 0
#define SPI0_CS_CHIPSEL1     0x00000001 // Use chip select 1
#define SPI0_CS_CHIPSEL2     0x00000002 // Use chip select 2
#define SPI0_CS_CHIPSELN     0x00000003 // No chip select (e.g. use GPIO pin)

#define SPI0_CS_CLRALL      (SPI0_CS_CLRFIFOS|SPI0_CS_DONE)

#define ISASC(x) ((x)>=0x20 && (x)<=0x7F)

using namespace std;

volatile bool g_stop = false;

void PrintFlags(int argc, char *argv[]);
void SignalHandler(int signum);
void PrintHelpMessage();
bool ParseFlags(int argc, char *argv[], bool& help, string& configfile, bool& fork);
bool usocket;

//
// Set up a memory regions to access GPIO and SPI0
//
void setup_io()
{

   /* open /dev/mem */
   if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
      printf("can't open /dev/mem \n");
      exit (-1);
   }

   /* mmap GPIO */
   if ((gpio_mem = malloc(BLOCK_SIZE + (PAGE_SIZE-1))) == NULL) {
      printf("allocation error \n");
      exit (-1);
   }
   if ((unsigned long)gpio_mem % PAGE_SIZE)
     gpio_mem += PAGE_SIZE - ((unsigned long)gpio_mem % PAGE_SIZE);

   gpio_map = (unsigned char *)mmap(
      (caddr_t)gpio_mem,
      BLOCK_SIZE,
      PROT_READ|PROT_WRITE,
      MAP_SHARED|MAP_FIXED,
      mem_fd,
      GPIO_BASE
   );

   if ((long)gpio_map < 0) {
      printf("mmap error %d\n", (int)gpio_map);
      exit (-1);
   }
   gpio = (volatile unsigned *)gpio_map;

   /* mmap SPI0 */
   if ((spi0_mem = malloc(BLOCK_SIZE + (PAGE_SIZE-1))) == NULL) {
      printf("allocation error \n");
      exit (-1);
   }
   if ((unsigned long)spi0_mem % PAGE_SIZE)
     spi0_mem += PAGE_SIZE - ((unsigned long)spi0_mem % PAGE_SIZE);

   spi0_map = (unsigned char *)mmap(
      (caddr_t)spi0_mem,
      BLOCK_SIZE,
      PROT_READ|PROT_WRITE,
      MAP_SHARED|MAP_FIXED,
      mem_fd,
      SPI0_BASE
   );


   printf("SPI mapped from 0x%p to 0x%p\n",SPI0_BASE,spi0_map);

   if ((long)spi0_map < 0) {
      printf("mmap error %d\n", (int)spi0_map);
      exit (-1);
   }
   spi0 = (volatile unsigned *)spi0_map;

} // setup_io

int main (int argc, char *argv[])
{
// fix pins

int g;

setup_io();  // Set up direct access to I/O for GPIO and SPI

  // Switch GPIO 7..11 to SPI mode (ALT function 0)

 /************************************************************************\
  * You are about to change the GPIO settings of your computer.          *
  * Mess this up and it will stop working!                               *
  * It might be a good idea to 'sync' before running this program        *
  * so at least you still have your code changes written to the SD-card! *
 \************************************************************************/

  for (g=7; g<=11; g++)
  {
    INP_GPIO(g);       // clear bits (= input)
    SET_GPIO_ALT(g,0); // set function 0
  }

  printf("\nBoblightd 2.0 (optimized version for raspberry) (c) 2013 Speedy1985 and Heven)\n");
  
  //read flags
  string configfile;
  bool   help;
  bool   bfork;
  
  
  // get this process pid
  pid_t pid = getpid();

  std::stringstream command;
  command << "ps | grep boblightd | grep -v 'grep boblightd' | grep -v 'gdb boblightd' | grep -v " << pid;
  int isRuning = system(command.str().c_str());
  if (isRuning == 0) {
      cout << "Boblightd already running, exit.." << endl;
      exit(1);
  }

  if (!ParseFlags(argc, argv, help, configfile, bfork) || help)
  {
    PrintHelpMessage();
    return 1;
  }

  if (bfork)
  {
    if (fork())
      return 0;
  }
  
  //init our logfile
  logtostderr = true;
  SetLogFile("boblightd.log");
  PrintFlags(argc, argv);

  //set up signal handlers
  signal(SIGTERM, SignalHandler);
  signal(SIGINT, SignalHandler);

  vector<CDevice*> devices; //where we store devices
  vector<CLight>   lights;  //lights pool
  CClientsHandler  clients(lights);

  { //save some ram by removing CConfig from the stack when it's not needed anymore
    CConfig config;  //class for loading and parsing config
    //load and parse config
    if (!config.LoadConfigFromFile(configfile))
      return 1;
    if (!config.CheckConfig())
      return 1;
    if (!config.BuildConfig(clients, devices, lights))
      return 1;
  }
  
  
  //start the devices
  Log("starting devices");
  for (int i = 0; i < devices.size(); i++)
    devices[i]->StartThread(); // each device has his own thread

  clients.SetSocket(usocket);
  
  //run the clients handler
  while(!g_stop)
    clients.Process();

  //signal that the devices should stop
  Log("signaling devices to stop");
  for (int i = 0; i < devices.size(); i++)
    devices[i]->AsyncStopThread();

  //clean up the clients handler
  clients.Cleanup();

  //stop the devices
  Log("waiting for devices to stop");
  for (int i = 0; i < devices.size(); i++)
    devices[i]->StopThread();

  Log("exiting");
  
  return 0;
}

void PrintFlags(int argc, char *argv[])
{
  string flags = "starting";
  
  for (int i = 0; i < argc; i++)
  {
    flags += " ";
    flags += argv[i];
  }

  Log("%s", flags.c_str());
}

void SignalHandler(int signum)
{
  if (signum == SIGTERM)
  {
    Log("caught SIGTERM");
    g_stop = true;
  }
  else if (signum == SIGINT)
  {
    Log("caught SIGINT");
    g_stop = true;
  }
  else
  {
    Log("caught %i", signum);
  }
}

void PrintHelpMessage()
{
  cout << "Usage: boblightd [OPTION]\n";
  cout << "\n";
  cout << "  options:\n";
  cout << "\n";
  cout << "  -c  set the config file, default is " << DEFAULTCONF << "\n";
  cout << "  -f  fork\n";
  cout << "\n";
}

bool ParseFlags(int argc, char *argv[], bool& help, string& configfile, bool& fork)
{
  help = false;
  fork = false;
  configfile = DEFAULTCONF;

  opterr = 0; //no getopt errors to stdout, we bitch ourselves
  int c;

  while ((c = getopt (argc, argv, "c:hxf")) != -1)
  {
    if (c == 'c')
    {
      configfile = optarg;
    }
    else if (c == 'h')
    {
      help = true;
      return true;
    }
    else if (c == 'f')
    {
      fork = true;
    }
    else if (c == '?')
    {
      if (optopt == 'c')
      {
        PrintError("Option " + ToString((char)optopt) + " requires an argument");
      }
      else
      {
        PrintError("Unknown option " + ToString((char)optopt));
      }
      return false;
    }
  }

  return true;
}
