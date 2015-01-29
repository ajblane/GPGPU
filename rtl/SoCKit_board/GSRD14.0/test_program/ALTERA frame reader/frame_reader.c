#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>


#define PAGE_SIZE 4096 // 2MB
#define LWHPS2FPGA_BRIDGE_BASE 0xff200000
#define FRAME_READER_OFFSET 0x00000100

volatile unsigned char *frame_reader_mem;
void *bridge_map;
void *frame_reader_map;
unsigned int frame0[1024][768];


void wi(unsigned char* address, unsigned int value){
     *address = value & 0x000000ff;
     *(address+1) = (value & 0x0000ff00) >> 8;
     *(address+2) = (value & 0x00ff0000) >> 16;
     *(address+3) = (value & 0xff000000) >> 24;
     
}
int main(int argc, char *argv[])
{
	int fd, ret = EXIT_FAILURE;
	off_t bridge_base = LWHPS2FPGA_BRIDGE_BASE;
        int i,j;
        for(i = 0 ; i < 1024 ; i++ )
        {
           for(j = 0 ; j < 768; j++)
           {
              frame0[i][j] = 0xffffffff;  
           }
        }
	/* open the memory device file */
	fd = open("/dev/mem", O_RDWR|O_SYNC);
	if (fd < 0) {
		perror("open");
		exit(EXIT_FAILURE);
	}
	bridge_map = mmap(NULL, PAGE_SIZE, PROT_WRITE, MAP_SHARED,
				fd, bridge_base);
	if (bridge_map == MAP_FAILED) {
		perror("mmap");
		goto cleanup;
	} 
      frame_reader_map = (unsigned char *)(bridge_map + FRAME_READER_OFFSET); 
      //SETTING      
      //Select frame 0
      wi(frame_reader_map + 3*4, 0);
      
      //Frame 0 base adress
      wi(frame_reader_map + 4*4,(unsigned int)frame0);
      /*
      the number of words the Frame Reader IP core must read from memory
      the width of the word is the same as the Avalon-MM read Master port width parameter.
      */
      wi(frame_reader_map + 4*5,1024*768*8*4/128);
      /*
      must know how many single-cycle color patterns make up the frame. 
      If each single cycle color pattern represents a pixel; 
      the quantity is simply the number of  pixels in the frame. 
      Otherwise, the quantity is the number of pixels in the frame,
      multiplied by the number of single-cycle color patterns required to represent a pixel.
      */
      wi(frame_reader_map + 4*6,1024*768*4);
 
      
      wi(frame_reader_map+ 4*8, 1024);
      wi(frame_reader_map+ 4*9, 768);
      wi(frame_reader_map+ 4*10, 0);
      // start Frame Reader
      wi(frame_reader_map, 1);
      

      while(1); 
      if (munmap(bridge_map, PAGE_SIZE) < 0) {
		perror("munmap");
		goto cleanup;
	}
	ret = 0;
cleanup:
	close(fd);
	return ret;
}
