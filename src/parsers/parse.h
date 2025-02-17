#ifndef _PARSE_GREMSY_UTILS
#define _PARSE_GREMSY_UTILS

#include <nlohmann/json.hpp>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <fstream>

unsigned int    gremsy_size() ;

bool            gremsy_open(std::string filename);

bool            gremsy_close();

std::vector<unsigned char> gremsy_read();

bool            gremsy_version(uint8_t *x, uint8_t *y, uint8_t *z);

uint32_t        gremsy_crc32();

#endif