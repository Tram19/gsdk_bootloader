#ifndef _PARSE_GREMSY_UTILS
#define _PARSE_GREMSY_UTILS


#include <fstream>
#include <iostream>
#include <vector>
#include <cstring>      // memset, memcpy
#include "json/json.h"

unsigned int    gremsy_size() ;

bool            gremsy_open(const std::string& filename);

void            gremsy_close();

std::vector<unsigned char> gremsy_read();

bool            gremsy_version(uint8_t &x, uint8_t &y, uint8_t &z);

uint32_t        gremsy_crc32();

#endif