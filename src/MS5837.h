#ifndef MS5837_H
#define MS5837_H

uint8_t initMS5837(uint16_t calib_data[]);
uint8_t crc4(uint16_t n_prom[]);
uint32_t read_MS5837_depth();

#endif
