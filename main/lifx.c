/*
**  LIFX C Library
**  Copyright 2016 Linard Arquint
*/

#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
//#include <netinet/in.h>
//#include <netinet/ip.h> 
#include <assert.h>
#include <errno.h>

#include "lifx.h"


#define SOCKET_TIMEOUT_US (500000)
/** maximal number of discoverable bulbs */
#define BULB_LIMIT (10)
#define BROADCAST_PORT (56700)
#define RECEIVE_RETRIES (5)

#define PROTOCOL_NUMBER (1024)
#define ADDRESSABLE (1)
#define ORIGIN (0)
#define LIFX_UDP_SERVICE (1)

#pragma pack(push, 1)
typedef struct {
  /* frame */
  uint16_t size;
  uint16_t protocol:12;
  uint8_t  addressable:1;
  uint8_t  tagged:1;
  uint8_t  origin:2;
  uint32_t source;
  /* frame address */
  uint8_t  target[8];
  uint8_t  reserved[6];
  uint8_t  res_required:1;
  uint8_t  ack_required:1;
  uint8_t  :6;
  uint8_t  sequence;
  /* protocol header */
  //uint64_t :64;
  uint64_t reserved1;
  uint16_t type;
  //uint16_t :16;
  uint16_t reserved2;
  /* variable length payload follows */
} lx_protocol_header_t;
#pragma pack(pop)

typedef struct {
    uint16_t payload_size;
    uint8_t *p_payload;
    uint8_t tagged;
    uint8_t ack_required;
    uint8_t res_required;
    uint16_t type;
} packet_config_t;


typedef enum {
	MSG_TYPE_GET_SERVICE = 2,
	MSG_TYPE_STATE_SERVICE,
    MSG_TYPE_GET_LIGHT = 101,
    MSG_TYPE_SET_COLOR,
    MSG_TYPE_LIGHT_STATE = 107,
    MSG_TYPE_GET_POWER = 116,
    MSG_TYPE_SET_POWER = 117,
    MSG_TYPE_STATE_POWER,
} lx_protocol_header_msg_type;


/** a negative number indicates non-existance */
static int udp_socket = -1;

static uint32_t source_id = 0;

static uint8_t p_buffer[512];

int init_lifx_lib() {
	// open socket
	udp_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (udp_socket < 0) {
		printf("opening socket failed\n");
		return -1;
	}
	// set broadcast permission:
	const int broadcastEnable = 1;
	if (setsockopt(udp_socket, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable))) {
		printf("aquiring broadcast permission failed\n");
		return -1;
	}
    // set socket timeout
    struct timeval read_timeout;
    read_timeout.tv_sec = 0;
    read_timeout.tv_usec = SOCKET_TIMEOUT_US;
    if (setsockopt(udp_socket, SOL_SOCKET, SO_RCVTIMEO, &read_timeout, sizeof read_timeout)) {
        printf("enabling socket timeout failed\n");
        return -1;
    }

	// generate random source_id:
	time_t t;
	// init random number generator:
   	srand((unsigned) time(&t));
   	source_id = (uint32_t)rand();

	return 0;
}

int close_lifx_lib() {
	if (udp_socket >= 0) {
		close(udp_socket);
	}
	udp_socket = -1;
	return 0;
}

static int createHeader(bulb_service_t *p_bulb, const packet_config_t *p_config, lx_protocol_header_t *p_header) {
    bzero(p_header, sizeof(lx_protocol_header_t));

	*p_header = (lx_protocol_header_t) {
    	// frame
 		.size = sizeof(lx_protocol_header_t) + p_config->payload_size,
    	.protocol = PROTOCOL_NUMBER,
    	.addressable = ADDRESSABLE,
    	.tagged = p_config->tagged,
    	.origin = ORIGIN,
    	.source = source_id,
    	// frame address
    	.target[0] = (uint8_t)((p_bulb->target >> 0) & 0xFF),
    	.target[1] = (uint8_t)((p_bulb->target >> 8) & 0xFF),
    	.target[2] = (uint8_t)((p_bulb->target >> 16) & 0xFF),
    	.target[3] = (uint8_t)((p_bulb->target >> 24) & 0xFF),
    	.target[4] = (uint8_t)((p_bulb->target >> 32) & 0xFF),
    	.target[5] = (uint8_t)((p_bulb->target >> 40) & 0xFF),
    	.target[6] = (uint8_t)((p_bulb->target >> 48) & 0xFF),
    	.target[7] = (uint8_t)((p_bulb->target >> 56) & 0xFF),
        .ack_required = p_config->ack_required,
        .res_required = p_config->res_required,
		// all set to 0
		// protocol header
		.type = p_config->type,
        .reserved1 = 0,
        .reserved2 = 0,
    };

    return 0;
}

static int getServerAddr(bulb_service_t *p_bulb, struct sockaddr_in *p_server_addr) {
	bzero(p_server_addr, sizeof(*p_server_addr));
	p_server_addr->sin_family = AF_INET;
    p_server_addr->sin_port = htons(p_bulb->port);
    p_server_addr->sin_addr.s_addr = htonl(p_bulb->in_addr);
    return 0;
}

static int sendPacket(bulb_service_t *p_bulb, const packet_config_t *p_config) {
	if (udp_socket < 0) {
		printf("discoverBulbs - socket not open");
		return -1;
	}

	struct sockaddr_in server_addr;
    if (getServerAddr(p_bulb, &server_addr)) {
    	printf("getServerAddr failed\n");
    	return -1;
    }

	lx_protocol_header_t header;
	if (createHeader(p_bulb, p_config, &header)) {
		printf("header creation for packet type %d failed\n", p_config->type);
		return -1;
	}

	uint16_t packet_size = sizeof(header) + p_config->payload_size;
    if (packet_size > sizeof(p_buffer)) {
        printf("p_buffer not large enough to send packet\n");
        return -1;
    }
	// copy header
	memcpy(p_buffer, &header, sizeof(header));
	// copy p_payload
	if (p_config->payload_size > 0) {
		memcpy(p_buffer + sizeof(header), p_config->p_payload, p_config->payload_size);
	}
	assert(((uint16_t *)p_buffer)[0] == packet_size);

    #ifdef DEBUG
	printf("packet [%d]\n", packet_size);
	for (int i = 0; i < packet_size; i++) {
    	if (i == 8 | i == 24 | i == 36) {
    		printf("|");
    	}
    	printf("%02X", p_buffer[i]);
    }
    printf("\n");
    #endif
    

	int res = sendto(udp_socket, p_buffer, packet_size, 0, (struct sockaddr *)&server_addr, sizeof(struct sockaddr_in));
    if (res < 0) {
    	printf("sending packet with type %d failed (err %d (%s))\n", p_config->type, errno, strerror(errno));
    	return -1;
    }
	
	if (res != packet_size) {
		printf("only partial packet sent\n");
		return -1;
	}
	return 0;
}

/** 
 * @p_payload free after use
 * @returns 1 when a timeout occurred 
 */
static int recvPacketWithServerAddr(bulb_service_t *p_bulb, struct sockaddr_in *p_server_addr, lx_protocol_header_t *p_header, uint8_t **pp_payload, uint16_t *p_payload_size) {
	if (udp_socket < 0) {
		printf("discoverBulbs - socket not open");
		return -1;
	}

	unsigned int server_addr_size = sizeof(*p_server_addr);
    if (getServerAddr(p_bulb, p_server_addr)) {
    	printf("getServerAddr failed\n");
    	return -1;
    }

    int res = recvfrom(udp_socket, p_buffer, sizeof(p_buffer), 0, (struct sockaddr *)p_server_addr, &server_addr_size);
    if (res == -1 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        // timeout
        return 1;
    }
    if (res < 0) {
    	printf("receiving packet failed\n");
    	return -1;
    }

    #ifdef DEBUG
    printf("Response buffer[%d]\n", res);
    for (int i = 0; i < res; i++) {
        if (i == 8 | i == 24 | i == 36) {
            printf("|");
        }
        printf("%02X", p_buffer[i]);
    }
    printf("\n");
    #endif

    // check length
    if (res < (int)sizeof(*p_header)) {
    	printf("unexpected response length\n");
    	return -1;
    }

    *p_payload_size = res - sizeof(*p_header);

    // copy header
    memcpy(p_header, p_buffer, sizeof(*p_header));

    // copy payload
    if (*p_payload_size > 0) {
        *pp_payload = malloc(*p_payload_size);
    	memcpy(*pp_payload, p_buffer + sizeof(*p_header), *p_payload_size);
    }

    // check source
    if (p_header->source != source_id) {
    	printf("response source doesn't match: %d instead of %d", p_header->source, source_id);
    	return -1;
    }

    return 0;
}

/** 
 * @p_payload free after use
 * @returns 1 when a timeout occurred 
 */
static int recvPacket(bulb_service_t *p_bulb, lx_protocol_header_t *p_header, uint8_t **pp_payload, uint16_t *p_payload_size) {
	struct sockaddr_in serverAddr;
	return recvPacketWithServerAddr(p_bulb, &serverAddr, p_header, pp_payload, p_payload_size);
}

static int recvPacketWithRetry(bulb_service_t *p_bulb, lx_protocol_header_t *p_header, uint8_t **pp_payload, uint16_t *p_payload_size) {
    int res;
    for (int i = 0; i < RECEIVE_RETRIES; i++) {
        res = recvPacket(p_bulb, p_header, pp_payload, p_payload_size);
        if (res != 1) {
            // error or success
            return res;
        }
        // continue looping in case of timeout (res == 1)
    }
    printf("recvPacketWithRetry max retries reached\n");
    return -1;
}

static int convertToBulbService(struct sockaddr_in *p_server_addr, lx_protocol_header_t *p_response_header, uint8_t *p_payload, const uint16_t payload_size, bulb_service_t *p_bulb) {
    // check payload size:
    if (payload_size < 5) {
        return -1;
    }
    // check response type
    if (p_response_header->type != MSG_TYPE_STATE_SERVICE) {
        return -1;
    }
    // eval response header & response payload
    uint8_t *p_bulb_mac_addr = p_response_header->target;
    p_bulb->in_addr = ntohl(p_server_addr->sin_addr.s_addr);
    p_bulb->target =    ((uint64_t)p_bulb_mac_addr[0] << 0) +
                        ((uint64_t)p_bulb_mac_addr[1] << 8) +
                        ((uint64_t)p_bulb_mac_addr[2] << 16) +
                        ((uint64_t)p_bulb_mac_addr[3] << 24) +
                        ((uint64_t)p_bulb_mac_addr[4] << 32) +
                        ((uint64_t)p_bulb_mac_addr[5] << 40) +
                        ((uint64_t)p_bulb_mac_addr[6] << 48) +
                        ((uint64_t)p_bulb_mac_addr[7] << 56);
    p_bulb->service = p_payload[0];
    p_bulb->port =  ((uint32_t)p_payload[1] << 0) + 
                    ((uint32_t)p_payload[2] << 8) + 
                    ((uint32_t)p_payload[3] << 16) + 
                    ((uint32_t)p_payload[4] << 24);

    if (p_bulb->service != LIFX_UDP_SERVICE) {
        // ignore bulb / service
        return -1;
    }

    return 0;
}

/** NULL terminated array of bulbs, free after use */
int discoverBulbs(bulb_service_t ***ppp_bulbs) {
	
 	// UDP broadcast to port 56700
    bulb_service_t broadcastBulb = {
    	.in_addr = INADDR_BROADCAST,
    	.target = (uint64_t)0, // send to all bulbs
    	.service = 1,
    	.port = BROADCAST_PORT,
    };

    packet_config_t config = {
        .payload_size = 0,
        .p_payload = NULL,
        .tagged = 1,
        .ack_required = 0,
        .res_required = 0,
        .type = MSG_TYPE_GET_SERVICE
    };

    if (sendPacket(&broadcastBulb, &config)) {
    	printf("send discover bulb packet failed\n");
    	return -1;
    }

    struct sockaddr_in serverAddr;
    lx_protocol_header_t response_header;
	uint16_t response_payload_size;
    uint8_t *p_payload;
    int counter = 0;
    int bulb_counter = 0;
    bulb_service_t **pp_bulbs = malloc(sizeof(bulb_service_t *) * (BULB_LIMIT + 1));
    *ppp_bulbs = pp_bulbs;
    while (true) {
        int res = recvPacketWithServerAddr(&broadcastBulb, &serverAddr, &response_header, &p_payload, &response_payload_size);
    	if (res == 1) {
            // timeout 
            printf("bulb discovery timeout\n");
            if (counter >= RECEIVE_RETRIES - 1) {
                break;
            }
        } else if (res) {
            // failure
            printf("receive discover bulb packet failed\n");
            return -1;
        } else {
            // response received
            printf("bulb response received\n");
            bulb_service_t *p_bulbs = malloc(sizeof(bulb_service_t));
            if (!convertToBulbService(&serverAddr, &response_header, p_payload, response_payload_size, p_bulbs)) {
                free(p_payload);
                pp_bulbs[bulb_counter] = p_bulbs;
                bulb_counter++;
                if (bulb_counter >= BULB_LIMIT) {
                    break;
                }
            } else {
                free(p_payload);
                free(p_bulbs);
            }
        }
        counter++;
    }

    // NULL terminate
    pp_bulbs[bulb_counter] = NULL;

    return 0;
}

int freeBulbs(bulb_service_t **pp_bulbs) {
    int bulb_counter = 0;
    while (pp_bulbs[bulb_counter] != NULL) {
        free(pp_bulbs[bulb_counter]);
        pp_bulbs[bulb_counter] = NULL;
    }
    return 0;
}

int getPower(bulb_service_t *p_bulb, bool *p_on) {
    packet_config_t config = {
        .payload_size = 0,
        .p_payload = NULL,
        .tagged = 0, // destination bulb is specified in the bulb_service_t struct
        .ack_required = 0,
        .res_required = 1,
        .type = MSG_TYPE_GET_POWER
    };

    if (sendPacket(p_bulb, &config)) {
        printf("send getPower packet failed\n");
        return -1;
    }

    lx_protocol_header_t res_header;
    uint8_t *p_res_payload = NULL;
    uint16_t res_payload_size;
    if (recvPacketWithRetry(p_bulb, &res_header, &p_res_payload, &res_payload_size)) {
        printf("receive getPower packet failed\n");
        return -1;
    }

    if (res_header.type != MSG_TYPE_STATE_POWER) {
        printf("wrong response type received: %d instead of %d\n", res_header.type, MSG_TYPE_STATE_POWER);
        return -1;
    }

    if (res_payload_size < 2) {
        printf("getPower response too short\n");
        return -1;
    }

    uint16_t level = ((uint16_t)p_res_payload[0] << 0) + 
                        ((uint16_t)p_res_payload[1] << 8);

    *p_on = level == 0 ? false : true;

    free(p_res_payload);
    p_res_payload = NULL;

    return 0;
}

int setPower(bulb_service_t *p_bulb, bool on, uint32_t duration) {
    // create payload
    uint8_t p_payload[6];
    // put level
    uint16_t level = on ? 65535 : 0;
    p_payload[0] = (level >> 0) & 0xFF;
    p_payload[1] = (level >> 8) & 0xFF;
    // put duration
    p_payload[2] = (duration >> 0) & 0xFF;
    p_payload[3] = (duration >> 8) & 0xFF;
    p_payload[4] = (duration >> 16) & 0xFF;
    p_payload[5] = (duration >> 24) & 0xFF;

    packet_config_t config = {
        .payload_size = sizeof(p_payload),
        .p_payload = p_payload,
        .tagged = 0, // destination bulb is specified in the bulb_service_t struct
        .ack_required = 0,
        .res_required = 1,
        .type = MSG_TYPE_SET_POWER
    };

    if (sendPacket(p_bulb, &config)) {
        printf("send setPower packet failed\n");
        return -1;
    }

    lx_protocol_header_t res_header;
    uint8_t *p_res_payload = NULL;
    uint16_t res_payload_size;
    if (recvPacketWithRetry(p_bulb, &res_header, &p_res_payload, &res_payload_size)) {
        printf("receive setPower packet failed\n");
        return -1;
    }

    if (res_header.type != MSG_TYPE_STATE_POWER) {
        printf("wrong response type received: %d instead of %d\n", res_header.type, MSG_TYPE_STATE_POWER);
        return -1;
    }

    free(p_res_payload);
    p_res_payload = NULL;

	return 0;
}

/** @param p_label 32 byte string and 1 null character as terminator */
int getColor(bulb_service_t *p_bulb, bool *p_on, color_t *p_color, char p_label[LIFX_LABEL_LENGTH + 1]) {
    packet_config_t config = {
        .payload_size = 0,
        .p_payload = NULL,
        .tagged = 0, // destination bulb is specified in the bulb_service_t struct
        .ack_required = 0,
        .res_required = 1,
        .type = MSG_TYPE_GET_LIGHT
    };

    if (sendPacket(p_bulb, &config)) {
        printf("send getColor packet failed\n");
        return -1;
    }

    lx_protocol_header_t res_header;
    uint8_t *p_res_payload = NULL;
    uint16_t res_payload_size;
    if (recvPacketWithRetry(p_bulb, &res_header, &p_res_payload, &res_payload_size)) {
        printf("receive getColor packet failed\n");
        return -1;
    }

    if (res_header.type != MSG_TYPE_LIGHT_STATE) {
        printf("wrong response type received: %d instead of %d\n", res_header.type, MSG_TYPE_LIGHT_STATE);
        return -1;
    }

    if (res_payload_size < 52) {
        printf("getColor response too short\n");
        return -1;
    }

    p_color->hue =    ((uint16_t)p_res_payload[0] << 0) + 
                    ((uint16_t)p_res_payload[1] << 8);
    p_color->saturation = ((uint16_t)p_res_payload[2] << 0) + 
                        ((uint16_t)p_res_payload[3] << 8);
    p_color->brightness = ((uint16_t)p_res_payload[4] << 0) + 
                        ((uint16_t)p_res_payload[5] << 8);
    p_color->kelvin = ((uint16_t)p_res_payload[6] << 0) + 
                    ((uint16_t)p_res_payload[7] << 8);

    uint16_t power =    ((uint16_t)p_res_payload[10] << 0) + 
                        ((uint16_t)p_res_payload[11] << 8);

    p_label[LIFX_LABEL_LENGTH] = '\0'; // NULL terminator
    memcpy(p_label, p_res_payload + 12, LIFX_LABEL_LENGTH);

    *p_on = power == 0 ? false : true;

    free(p_res_payload);
    p_res_payload = NULL;

    return 0;
}

int setColor(bulb_service_t *p_bulb, color_t color, uint32_t duration) {
    // create payload
    uint8_t p_payload[13];
    // put color
    p_payload[1] = (color.hue >> 0) & 0xFF;
    p_payload[2] = (color.hue >> 8) & 0xFF;
    p_payload[3] = (color.saturation >> 0) & 0xFF;
    p_payload[4] = (color.saturation >> 8) & 0xFF;
    p_payload[5] = (color.brightness >> 0) & 0xFF;
    p_payload[6] = (color.brightness >> 8) & 0xFF;
    p_payload[7] = (color.kelvin >> 0) & 0xFF;
    p_payload[8] = (color.kelvin >> 8) & 0xFF;
    // put duration
    p_payload[9] = (duration >> 0) & 0xFF;
    p_payload[10] = (duration >> 8) & 0xFF;
    p_payload[11] = (duration >> 16) & 0xFF;
    p_payload[12] = (duration >> 24) & 0xFF;

    packet_config_t config = {
        .payload_size = sizeof(p_payload),
        .p_payload = p_payload,
        .tagged = 0, // destination bulb is specified in the bulb_service_t struct
        .ack_required = 0,
        .res_required = 1,
        .type = MSG_TYPE_SET_COLOR,
    };

    if (sendPacket(p_bulb, &config)) {
        printf("send setColor packet failed\n");
        return -1;
    }
    
    lx_protocol_header_t res_header;
    uint8_t *p_res_payload = NULL;
    uint16_t res_payload_size;
    if (recvPacketWithRetry(p_bulb, &res_header, &p_res_payload, &res_payload_size)) {
        printf("receive setColor packet failed\n");
        return -1;
    }

    if (res_header.type != MSG_TYPE_LIGHT_STATE) {
        printf("wrong response type received: %d instead of %d\n", res_header.type, MSG_TYPE_LIGHT_STATE);
        return -1;
    }

    free(p_res_payload);
    p_res_payload = NULL;
    
    return 0;
}
