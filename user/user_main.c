/* main.c -- MQTT client example
*
* Copyright (c) 2014-2015, Tuan PM <tuanpm at live dot com>
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice,
* this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of Redis nor the names of its contributors may be used
* to endorse or promote products derived from this software without
* specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/
#include "ets_sys.h"
#include "driver/uart.h"
#include "driver/sht1x.h"
#include "osapi.h"
#include "mqtt.h"
#include "wifi.h"
#include "config.h"
#include "debug.h"
#include "gpio.h"
#include "user_interface.h"
#include "mem.h"

uint16_t temp=0xffff, humidity=0xffff;
MQTT_Client mqttClient;
char topic[32], id[10];
#define INTERVAL 60*1000*1000


void disconnectAndSleep() {
	unsigned int sleep_time;
	INFO("\r\nGoing sleep\r\n");
	MQTT_Disconnect(&mqttClient);
	sleep_time = INTERVAL - system_get_time();
	if (sleep_time>INTERVAL) sleep_time = INTERVAL;
	system_deep_sleep(sleep_time);
}

void wifiConnectCb(uint8_t status)
{
	if(status == STATION_GOT_IP){
		MQTT_Connect(&mqttClient);
	} else {
		MQTT_Disconnect(&mqttClient);
	}
}
void mqttConnectedCb(uint32_t *args)
{
	char buf[128];
	MQTT_Client* client = (MQTT_Client*)args;
	INFO("MQTT: Connected\r\n");
	MQTT_Subscribe(client, "/esp/time", 0);

	uint16_t len = os_sprintf(buf, "[{\"timestamp\": %%d, \"device\": \"%s\", \"data\": {\"raw\": 1, \"temperature\": %d, \"humidity\": %d}}]", id, temp, humidity);
	MQTT_Publish(client, topic, buf, len, 0, 0);


}

void mqttDisconnectedCb(uint32_t *args)
{
	MQTT_Client* client = (MQTT_Client*)args;
	INFO("MQTT: Disconnected\r\n");
}

#ifdef UPGRADE_OTA
upgrade_states_check_callback ota_cb(void * arg)
{

	INFO("OTA CB!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n");
}
#define user_bin_n 1

struct upgrade_server_info upServer;

void checkUpgrade(struct upgrade_server_info  *server){
	server = (struct upgrade_server_info  *)os_zalloc(sizeof(struct upgrade_server_info));
	os_sprintf(server->pre_version, "v%d.%d", 1, 0);
	os_sprintf(server->upgrade_version, "v%d.%d", 1, 1);
	server->check_cb = (upgrade_states_check_callback)ota_cb;
	server->check_times = 60000;
	server->port = 80;
	uint32_t otaip = ipaddr_addr("192.168.1.2");
	os_memcpy(server->ip, &otaip, 4);
	server->url = (uint8 *) os_zalloc(512);
	os_sprintf(server->url,
			"GET /v1/device/rom/?action=download_rom&version=%s&filename=user%d.bin HTTP/1.1\r\nHost: "IPSTR":%d\r\n\r\n",
			server->upgrade_version,
			user_bin_n,
			IP2STR(server->ip),
			server->port
	);
//	system_upgrade_userbin_check()
	system_upgrade_init();
	system_upgrade_start(server);
}
#endif

void mqttPublishedCb(uint32_t *args)
{
	MQTT_Client* client = (MQTT_Client*)args;
	INFO("MQTT: Published\r\n");
	INFO("System time: %d\r\n", system_get_time());

//	checkUpgrade(&upServer);

	disconnectAndSleep();
}

void mqttDataCb(uint32_t *args, const char* topic, uint32_t topic_len, const char *data, uint32_t data_len)
{
	char *topicBuf = (char*)os_zalloc(topic_len+1),
			*dataBuf = (char*)os_zalloc(data_len+1);

	MQTT_Client* client = (MQTT_Client*)args;

	os_memcpy(topicBuf, topic, topic_len);
	topicBuf[topic_len] = 0;

	os_memcpy(dataBuf, data, data_len);
	dataBuf[data_len] = 0;

	INFO("Receive topic: %s, data: %s \r\n", topicBuf, dataBuf);
	os_free(topicBuf);
	os_free(dataBuf);
}

#define HALFMAC2STR(a) (a)[3], (a)[4], (a)[5]
#define HALFMACSTR "%02x:%02x:%02x"
void get_device_id(char *str) {
	char hwaddr[6];
	wifi_get_macaddr(0, hwaddr);
	os_sprintf(str, HALFMACSTR , HALFMAC2STR(hwaddr));
}


void user_init(void)
{

	int ret;

	uart_init(BIT_RATE_115200, BIT_RATE_115200);
	INFO("System time: %d\r\n", system_get_time());
	os_delay_us(1000000);
	gpio_init();
	sht1x_init();

	ret = sht11_measure(MEASURE_TEMP, &temp);
	ret += sht11_measure(MEASURE_HUMI, &humidity);
	CFG_Load();

	get_device_id(id);
	os_sprintf(topic, "/esp/sht11/%s" , id);
//	os_sprintf(topic_humi, "/esp/%s/humidity" , id);

	MQTT_InitConnection(&mqttClient, sysCfg.mqtt_host, sysCfg.mqtt_port, sysCfg.security);

	MQTT_InitClient(&mqttClient, sysCfg.device_id, sysCfg.mqtt_user, sysCfg.mqtt_pass, sysCfg.mqtt_keepalive, 1);

	MQTT_InitLWT(&mqttClient, "/lwt", "offline", 0, 0);
	MQTT_OnConnected(&mqttClient, mqttConnectedCb);
	MQTT_OnDisconnected(&mqttClient, mqttDisconnectedCb);
	MQTT_OnPublished(&mqttClient, mqttPublishedCb);
	MQTT_OnData(&mqttClient, mqttDataCb);

	WIFI_Connect(sysCfg.sta_ssid, sysCfg.sta_pwd, wifiConnectCb);

	INFO("\r\nSystem started ...\r\n");
}
