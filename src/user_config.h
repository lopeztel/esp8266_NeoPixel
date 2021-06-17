#ifndef _USER_CONFIG_H_
#define _USER_CONFIG_H_

#define INFO_MSG	true

#define WIFI_CLIENTSSID		"<your_wifi_client_SSID>"
#define WIFI_CLIENTPASSWORD	"<your_wifi_client_password>"

#define WIFI_CHECK_DELAY 3000	/* milliseconds */ //TODO:change to check every 5 minutes?
#define FADE_DELAY 2000

/*MQTT stuff*/
#define MQTT_HOST			"<MQTT_broker_host IP>" //Host IP address, Raspberry Pi or "mqtt.yourdomain.com"
#define MQTT_PORT			1883
#define MQTT_BUF_SIZE		1024
#define MQTT_KEEPALIVE		120	 /*second*/

#define MQTT_CLIENT_ID		"ESP_NeoPixel" //Name for your client, no duplicates
#define MQTT_USER			"<MQTT_broker_user>"
#define MQTT_PASS			"<MQTT_broker_password>"
#define MQTT_RECONNECT_TIMEOUT 	5	/*second*/
#define DEFAULT_SECURITY	0
#define QUEUE_BUFFER_SIZE		 		2048
#define PROTOCOL_NAMEv31	/*MQTT version 3.1 compatible with Mosquitto v0.15*/
//PROTOCOL_NAMEv311			/*MQTT version 3.11 compatible with https://eclipse.org/paho/clients/testing/*/
#define CLIENT_SSL_ENABLE

//Adafruit_NeoPixel Stuff
#define PIN	2
#define NUMPIXELS	<number_of_leds>
#define BRIGHTNESS	50
// RGB NeoPixel permutations; white and red offsets are always same
// Offset:         W          R          G          B
#define NEO_GRB	((1 << 6) | (1 << 4) | (0 << 2) | (2))
#define _BV(b) (1UL << (b))
#endif
