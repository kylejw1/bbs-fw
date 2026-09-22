/* Host harness: drives the firmware estimator with the same signals the bike
 * would provide, so the fixed point port can be checked against the Python
 * reference. __xdata is SDCC-only, so it is compiled away here. */
#include <stdio.h>
#include <stdint.h>
#include "loadsensor.h"

int main(void)
{
	char line[256];
	long t;
	int cr, cm, pct, truth;

	loadsensor_init(40);	/* must match the simulation's I_max */

	if (!fgets(line, sizeof line, stdin)) return 1;
	while (fgets(line, sizeof line, stdin))
	{
		if (sscanf(line, "%ld,%d,%d,%d,%d", &t, &cr, &cm, &pct, &truth) != 5)
			continue;
		loadsensor_process((uint16_t)cr, (uint16_t)cm, (uint8_t)pct, (uint32_t)t);
		printf("%ld,%d,%d,%d,%d\n", t,
			loadsensor_get_rider_torque_dnm(),
			loadsensor_get_load_bias_dnm(),
			loadsensor_get_flags(), truth);
	}
	return 0;
}
