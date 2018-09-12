/*
** DVS GEN2 Package for Linux - Minimal
** modified by CWS
** original file: 08_cybulk.c 
** Samsung Electronics
*/

#include <stdio.h>
#include <string>
#include <unistd.h>
#include <ctype.h>
#include <iostream>
#include <stdlib.h>
#include <getopt.h>
#include <string.h>
#include <pthread.h>
#include <cyusb.h>

static const char * program_name;
static const char *const short_options = "hndst:r:w:v:l:";
static const struct option long_options[] = {
		{ "help",		0,		NULL,		'h'},
		{ "load",		1,		NULL,		'l',},
		{ "stream",		1,		NULL,		's',},
		{ "debug",		1,		NULL,		'd',},
		{ NULL,			0,		NULL,		 0}
};

static int next_option;

static void print_usage(FILE, int);

static FILE *fp = stdout;
static int timeout_provided;
static int timeout = 1000;
static cyusb_handle *h1 = NULL;
static int debug = false;

const int buflen = 1024;
const int I2C_VALUE_LEN = 2;
const int I2C_SLAVE_ADDR = 96;	// 0x60

const int	I2C_SLAVE_ADDR_D2FX = 0x40;	// 7'h20
const int	I2C_SLAVE_ADDR_DVSR = 0x30;	// right(7'h18)
const int	I2C_SLAVE_ADDR_DVSL = 0x20;	// left (7'h10)
const int	I2C_SLAVE_ADDR_M2PR = 0x1A;	// right(7'h0D)
const int	I2C_SLAVE_ADDR_M2PL = 0x1C;	// left (7'h0E)
const int	I2C_VALUE_LEN_D2FX = 1;
const int	I2C_VALUE_LEN_DVSR = 1;
const int	I2C_VALUE_LEN_DVSL = 1;
const int	I2C_VALUE_LEN_M2PR = 2;
const int	I2C_VALUE_LEN_M2PL = 2;

struct Node {
	void *data;
	int len;
	struct Node* next;
};
struct Node* front = NULL;
struct Node* rear = NULL;
pthread_mutex_t queue_mutex;

static void validate_inputs(void);
int I2cSlaveAddr (int);
int I2cValueLen (int);
int readI2cReg(int, int);
int writeI2cReg(int, int, int);
int htoi(char, int);
int parseString(char, int, int, int);
void loadScript(char);
int ahtoi(char);
void Enqueue(void*, int);
void *Dequeue(int*);
void DecodePacket(unsigned char *, int, short int);
static void *reader(void *);
static void *processor(void *);
