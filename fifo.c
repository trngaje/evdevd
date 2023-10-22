#include "fifo.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>

extern int g_iLeftAnalog_to_RightAnalog;
extern int g_iDPAD_rotate;

void *threadGetFifoMessages(void *argumentPointer)
{
#ifdef RGB30
	const char* file = "/home/ark/mypipe";
#else
	const char* file = "/home/odroid/mypipe";
#endif
	int fd = open(file, O_RDONLY);
	if (fd < 0) 
	{
		if ( -1 == mkfifo( file, 0666))
		{
			perror( "mkfifo() 실행에러");
			exit(1);
		}

		fd = open(file, O_RDONLY);
		if (fd < 0)
		{
			perror( "open 에러");
			exit(1);			
		}
	}
	unsigned count = 0;
	char strmypipe[256];

	int iExit=0;
	while (iExit==0) {   

		count = read(fd, strmypipe, sizeof(strmypipe));

		if (0 == count)
		{
			sleep(1); 
		}
		else
		{
			strmypipe[count] = '\0'; 	
/*			
			printf("[trngaje] strmypipe = %s (%d)\n", strmypipe, count);
			int i;
			for(i=0; i<count; i++)
				printf("[0x%02x]", strmypipe[i]);
*/
			if (strcmp(strmypipe, "drastic") == 0)
			{
				g_iLeftAnalog_to_RightAnalog = 1;
			}
			else if (strcmp(strmypipe, "drastic_v") == 0)
			{
				g_iLeftAnalog_to_RightAnalog = 1;
				g_iDPAD_rotate = 1;
			}
			else if (strcmp(strmypipe, "0") == 0)
			{
				g_iLeftAnalog_to_RightAnalog = 0;
				g_iDPAD_rotate = 0;
			}
			else if (strcmp(strmypipe, "drastic_v") == 0)
				printf("[trngaje] best\n");

			else if (strcmp(strmypipe, "exit") == 0)
			{
				iExit = 1;
				break;
			}
		}
	}

  close(fd);       /* close pipe from read end */
  unlink(file);    /* unlink from the underlying file */

  return 0;
}