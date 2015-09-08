/*
 * main.c
 *
 *  Created on: 07.04.2015
 *      Author: pflaum
 */

#include "global.h"

int main(void) {
	char array[MAX_ARRAYSIZE + 1];
	uint8_t rx_length, tx_length;

	rfm_init();

	while (42) {
		if (rfm_receiving()) {
			rfm_receive(array, &rx_length);
			tx_length = rx_length;
			rfm_transmit(array, tx_length);
			rfm_rxon();
		}
	}
	return 0;
}
