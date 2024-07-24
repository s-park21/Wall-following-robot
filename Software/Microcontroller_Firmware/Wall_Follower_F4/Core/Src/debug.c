/*
 * debug.c
 *
 *  Created on: Apr 20, 2023
 *      Author: Angus McLennan
 */


#include "debug.h"

/* Debug print function */
void debug_print(char *msg, size_t len, enum debug_level dbl) {
#ifdef DEBUG_ENABLED
	if(dbl >= dbg_level) {
		char *log_levels[] = {"DEBUG:   ", "INFO:    ", "WARNING: ", "ERROR:   ", "CRITICAL:"};
		CDC_Transmit_FS((uint8_t*)msg, len);
	}
#elif defined(USB_LOGGING)
	// Print statement must have greater than or equal level to the global debug level set
	if(dbl == LOG_OUTPUT) {
		// Print data to USB
		CDC_Transmit_FS((uint8_t*)msg, len);
		return;
	}
#endif
}
