#ifndef AHT20_H
#define	AHT20_H

#include <stdint.h>

void AHT20_init(void);
void AHT20_read_sensor(void);
float AHT20_get_temperature(void);
float AHT20_get_humidity(void);

#endif //AHT20_H