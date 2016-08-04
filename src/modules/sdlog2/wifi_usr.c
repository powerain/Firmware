#include "wifi_usr.h"
#include "iniRD.h"

#include <termios.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include<stdio.h>
#include<stdlib.h>
#include<unistd.h>
#include<string.h>

#define  MAX_PATH 260
#include <stdarg.h>
#define WIFI_INI "/fs/microsd/USRwifi.ini"
// int fd,fd1;
// int flagEnAt;
//char *para = GetIniKeyString("WMODE","mode","/fs/microsd/USRwifi.ini");
//if (para != NULL)
	//printf("%s\n", para);
// [SYS]
// id=1
uint8_t	node_id = 0;
int wflag;
bool wifi_init(int fd)
{
	char buf[MAX_PATH];
	char *para,*pbuf;
	pbuf = (char *)&buf;

	if (fd <= 0)
		return false;

	para = GetIniKeyString("SYS","id", WIFI_INI);
	node_id = atoi(para);
	printf("ID is: %d\n", node_id);

	usleep(9000000);
	EnterAT(fd);
	usleep(1000000);

	memset(buf,0,sizeof(buf));
	para = GetIniKeyString("WMODE","mode", WIFI_INI);
	strcat(buf,para);
	WMODE(fd,pbuf);
	usleep(3000000);

	memset(buf,0,sizeof(buf));
	para = GetIniKeyString("WSSSID","ssid", WIFI_INI);
	strcat(buf,para);
	WSSSID(fd, pbuf);
	usleep(2000000);

	memset(buf,0,sizeof(buf));
	para = GetIniKeyString("WSKEY","auth", WIFI_INI);
	strcat(buf,para);
	strcat(buf,",");
	para = GetIniKeyString("WSKEY","encry", WIFI_INI);
	strcat(buf,para);
	strcat(buf,",");
	para = GetIniKeyString("WSKEY","key", WIFI_INI);
	strcat(buf,para);
	WSKEY(fd, pbuf);
	usleep(9000000);

	memset(buf,0,sizeof(buf));
	para = GetIniKeyString("WSKEY","protocol", WIFI_INI);
	strcat(buf,para);
	strcat(buf,",");
	para = GetIniKeyString("WSKEY","CS", WIFI_INI);
	strcat(buf,para);
	strcat(buf,",");
	para = GetIniKeyString("WSKEY","port", WIFI_INI);
	strcat(buf,para);
	strcat(buf,",");
	para = GetIniKeyString("WSKEY","IP", WIFI_INI);
	strcat(buf,para);
	NETP(fd, pbuf);
	usleep(5000000);

	ATZ(fd);
	usleep(5000000);

	return true;
}
/*
bool wifi_aplink_status(void)
{

}

bool wifi_tcp_status(void)
{

}*/

/*void EnterAT(int fd)
{
	//char cmdbuf[16];
	char b;
	// cmdbuf[0]='+';
	// wflag = write(fd, cmdbuf, 1);
	// usleep(3000);
	// wflag = write(fd, cmdbuf, 1);
	// usleep(3000);
	// wflag = write(fd, cmdbuf, 1);
	// usleep(3000);

	wflag = write(fd, "+++", 3);
	while( read(fd, &b, 1) )
	{
		if( b == 'a')
			break;
		else 
			printf("%x\n", b);
	}

	printf("%x\n", b);
	//wflag = write(fd, "a", 1);
}*/

void EnterAT(int fd)
{
	char cmdbuf[8];
	cmdbuf[0]='+';
	wflag = write(fd, cmdbuf, 1);
	usleep(3000);
	wflag = write(fd, cmdbuf, 1);
	usleep(3000);
	wflag = write(fd, cmdbuf, 1);
	usleep(1000000);
	cmdbuf[0]='a';
	wflag = write(fd, cmdbuf, 1);
}

void ENTM(int fd)		//进入透传模式
{
	char cmdbuf[100]="AT+ENTM\r";
	wflag = write(fd, cmdbuf, strlen((char*)cmdbuf) + 1);
	if(wflag > 0 )
	{
		printf("WIFI SET: %s\n",cmdbuf);
	}
}

void WMODE(int fd, char *para)				//wifi模块设置为STA模式
{
	char cmdbuf[100]="AT+WMODE=";	//STA\r";
	strcat(cmdbuf,para);
	strcat(cmdbuf,"\r");
	wflag = write(fd, cmdbuf, strlen((char*)cmdbuf) + 1);
	if(wflag > 0)
	{
		printf("WIFI SET: %s\n",cmdbuf);
	}
}

void WSSSID(int fd, char *para)					//查询或设置wifi模块的SSSID
{
	char cmdbuf[100]="AT+WSSSID=";	//test\r";
	strcat(cmdbuf,para);
	strcat(cmdbuf,"\r");
	wflag = write(fd, cmdbuf, strlen((char*)cmdbuf) + 1);
	if(wflag > 0)
	{
		printf("WIFI SET: %s\n",cmdbuf);
	}
}

void WSKEY(int fd, char *para)					//STA模式下查询或设置wifi模块的加密和连接密码
{
	char cmdbuf[100]="AT+WSKEY=";	//WPA2PSK,AES,abc123456\r";
	strcat(cmdbuf,para);
	strcat(cmdbuf,"\r");
	//wflag = write(fd, cmdbuf, strlen((char*)cmdbuf) + 1);
	for(int i = 0; i < strlen((char*)cmdbuf) + 1; i++)
	{
		wflag = write(fd, &cmdbuf[i], 1);
		usleep(6000);
	}

	if(wflag > 0)
	{
		printf("WIFI SET: %s\n",cmdbuf);
	}
}

void NETP(int fd, char *para)					//设置要连接到的内网服务器IP和端口号
{
	char cmdbuf[100]="AT+NETP=";	//TCP,CLIENT,8899,192.168.1.101\r";
	strcat(cmdbuf,para);
	strcat(cmdbuf,"\r");
	
	for(int i = 0; i < strlen((char*)cmdbuf) + 1; i++)
	{
		wflag = write(fd, &cmdbuf[i], 1);
		usleep(6000);
	}
	if(wflag > 0 )
	{
		printf("WIFI SET: %s\n",cmdbuf);
	}
}

void WANN(int fd)					//查询或设置WAN设置
{
	char cmdbuf[100]="AT+WANN\r";
	wflag = write(fd, cmdbuf, strlen((char*)cmdbuf) + 1);
	if(wflag > 0 )
	{
		printf("WIFI SET: %s\n",cmdbuf);
	}
}

void RELD(int fd)					//恢复出厂设置
{
	char cmdbuf[100]="AT+RELD\r";
	wflag = write(fd, cmdbuf, strlen((char*)cmdbuf) + 1);
	if(wflag > 0)
	{
		printf("WIFI SET: %s\n",cmdbuf);
	}
}

void ATZ(int fd)					//wifi模块重启
{
	char cmdbuf[100]="AT+Z\r";
	wflag = write(fd, cmdbuf, strlen((char*)cmdbuf));
	if(wflag > 0)
	{
		//printf("%d\n", strlen((char*)cmdbuf));
		printf("WIFI SET: %s\n",cmdbuf);
	}
	usleep(7000000);//手册说明重启需要6s
}

void WSLK(int fd)					//STA模式下查询或设置wifi模块的联网状态
{
	char cmdbuf[100]="AT+WSLK\r";
	wflag = write(fd, cmdbuf, strlen((char*)cmdbuf) + 1);
	if(wflag > 0)
	{
		printf("WIFI SET: %s\n",cmdbuf);
	}
}

void WSCAN(int fd)					//wifi模块的查询周边AP
{
	char cmdbuf[100]="AT+WSCAN\r";
	wflag = write(fd, cmdbuf, strlen((char*)cmdbuf) + 1);
	if(wflag > 0)
	{
		printf("WIFI SET: %s\n",cmdbuf);
	}
}

void TCPLK(int fd)					//wifi模块的查询连接TCP状态
{
	char cmdbuf[100]="AT+TCPLK\r";
	wflag = write(fd, cmdbuf, strlen((char*)cmdbuf) + 1);
	if(wflag > 0)
	{
		printf("WIFI SET: %s\n",cmdbuf);
	}
}

void TCPDIS(int fd)					//wifi模块的断开TCP状态
{
	char cmdbuf[100]="AT+TCPDIS\r";
	wflag = write(fd, cmdbuf, strlen((char*)cmdbuf) + 1);
	if(wflag > 0)
	{
		printf("WIFI SET: %s\n",cmdbuf);
	}
}
