#pragma once
#define NUKED_ENABLE_ASIO 0
#define NUKED_ENABLE_DECODER2 1
#define NUKED_VERSION "host"
#define NUKED_SOURCE "host"
#define NUKED_VERSION_MAJOR 0
#define NUKED_VERSION_MINOR 8
#define NUKED_VERSION_PATCH 0
#include <cstdio>
void Cfg_WriteVersionInfo(FILE* file);
