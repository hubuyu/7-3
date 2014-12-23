#include <stdio.h>
#include <fcntl.h>
#include <string.h>

#define DEV_NAME	"/dev/hubuyu"

int main()
{	
	int fd ;
	int err ;
	char data;
	char value = 1 ;	
	int i = 0 ;
	fd = open(DEV_NAME,O_RDWR);
	if(fd<0)
	{
		printf("Open %s failed \n" ,DEV_NAME );	
		return -1 ;
	}
	do{
		err = write(fd , &value , sizeof(value));
		if(err<0){
			printf("Write data to kernel failed\n");
			return -1 ;
		}
		value++;
		printf("Write-0x%x to kernel success\n",value );
		err = read(fd , &data ,sizeof(data));
		if(err<0){
			printf("Read data from kernel failed\n");
			return -1 ;
		}
		printf("Read data-0x%x \n",data);
		sleep(1);
	}while(i++<60);
	close(fd);
}
