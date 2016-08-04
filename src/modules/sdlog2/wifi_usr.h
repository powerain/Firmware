#include <stdio.h>

extern uint8_t	node_id;
#ifdef __cplusplus
extern "C"
{
#endif
bool wifi_init(int fd);
//bool EnterAT(int fd);
void EnterAT(int fd);
void ENTM(int fd);
void WMODE(int fd, char *para);
void WSSSID(int fd, char *para);
void WSKEY(int fd, char *para);
void NETP(int fd, char *para);
void ATZ(int fd);
void RELD(int fd);
void WSLK(int fd);
void WSCAN(int fd);
void TCPLK(int fd);
void TCPDIS(int fd);
void WANN(int fd);
	//void mag_cali(double);
#ifdef __cplusplus
}
#endif
