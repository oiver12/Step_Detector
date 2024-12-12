/*
 * wifi_task.c
 *
 *  Created on: Dec 12, 2024
 *      Author: olive
 */
#include "WiFi/es_wifi_io.h"
#include "WiFi/es_wifi.h"
#include "WiFi/wifi.h"

#define SSID     "YOUR_WIFI_SSID"
#define PASSWORD "YOUR_WIFI_PASSWORD"
uint8_t RemoteIP[] = {192, 0, 2, 1};
#define RemotePORT	8080

#define WIFI_WRITE_TIMEOUT 10000
#define WIFI_READ_TIMEOUT  10000
#define LOG(a) printf a
#define CONNECTION_TRIAL_MAX          10

static  uint8_t  IP_Addr[4];
static uint8_t RxData [500];

static  int wifi_start(void);
static  int wifi_connect(void);

void wifiServerTask(void *arguments)
{
  uint8_t TxData[] = "STM32 : Hello!\n";
  int32_t Socket = -1;
  uint16_t Datalen;
  int16_t Trials = CONNECTION_TRIAL_MAX;
  int32_t ret;

  LOG(("\nRunning HTML Server test\r\n"));
  if (wifi_connect()!=0)
  {
	  printf("Can't connect to WiFi\r\n");
	  vTaskDelete(NULL);
  }

  printf("> Trying to connect to Server: %d.%d.%d.%d:%d ...\r\n",
		  RemoteIP[0],
		  RemoteIP[1],
		  RemoteIP[2],
		  RemoteIP[3],
		  RemotePORT);

  while (Trials--)
  {
	  if( WIFI_OpenClientConnection(0, WIFI_TCP_PROTOCOL, "TCP_CLIENT", RemoteIP, RemotePORT, 0) == WIFI_STATUS_OK)
	  {
		  printf("> TCP Connection opened successfully.\r\n");
		  Socket = 0;
		  break;
	  }
  }
  if(Socket == -1)
  {
	  printf("> ERROR : Cannot open Connection\r\n");
	  vTaskDelete(NULL);
  }
  while(1)
  {
	  if(Socket != -1)
	  {
		  ret = WIFI_ReceiveData(Socket, RxData, sizeof(RxData)-1, &Datalen, WIFI_READ_TIMEOUT);
		  if(ret == WIFI_STATUS_OK)
		  {
			  if(Datalen > 0)
			  {
				  if(RxData[0] == 's')
					  startSaving();
				  else if(RxData[0] == 'r')
					  startReading();
				  RxData[Datalen]=0;
				  printf("Received: %s\r\n",RxData);
				  ret = WIFI_SendData(Socket, TxData, sizeof(TxData), &Datalen, WIFI_WRITE_TIMEOUT);
				  if (ret != WIFI_STATUS_OK)
				  {
					  printf("ERROR : Failed to Send Data, connection closed\r\n");
					  break;
				  }
			  }
		  }
		  else
		  {
			  printf("> ERROR : Failed to Receive Data, connection closed\r\n");
			  break;
		  }
	  }
	  vTaskDelay(pdMS_TO_TICKS(10));
  }
  vTaskDelete(NULL);
}

static int wifi_start(void)
{
  uint8_t  MAC_Addr[6];

 /*Initialize and use WIFI module */
  if(WIFI_Init() ==  WIFI_STATUS_OK)
  {
    LOG(("ES-WIFI Initialized.\r\n"));
    if(WIFI_GetMAC_Address(MAC_Addr, sizeof(MAC_Addr)) == WIFI_STATUS_OK)
    {
      LOG(("> eS-WiFi module MAC Address : %02X:%02X:%02X:%02X:%02X:%02X\r\n",
               MAC_Addr[0],
               MAC_Addr[1],
               MAC_Addr[2],
               MAC_Addr[3],
               MAC_Addr[4],
               MAC_Addr[5]));
    }
    else
    {
      LOG(("> ERROR : CANNOT get MAC address\r\n"));
      return -1;
    }
  }
  else
  {
    return -1;
  }
  return 0;
}

int wifi_connect(void)
{

  wifi_start();

  LOG(("\nConnecting to %s\r\n",SSID));
  if( WIFI_Connect(SSID, PASSWORD, WIFI_ECN_WPA2_PSK) == WIFI_STATUS_OK)
  {
    if(WIFI_GetIP_Address(IP_Addr, sizeof(IP_Addr)) == WIFI_STATUS_OK)
    {
      LOG(("> es-wifi module connected: got IP Address : %d.%d.%d.%d\r\n",
               IP_Addr[0],
               IP_Addr[1],
               IP_Addr[2],
               IP_Addr[3]));
    }
    else
    {
		  LOG((" ERROR : es-wifi module CANNOT get IP address\r\n"));
      return -1;
    }
  }
  else
  {
		 LOG(("ERROR : es-wifi module NOT connected\r\n"));
     return -1;
  }
  return 0;
}
