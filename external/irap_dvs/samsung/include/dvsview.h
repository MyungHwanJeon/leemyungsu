/*
** DVS GEN2 Package for Linux - Minimal
** KBL & CWS
** Samsung Electronics
*/

#include <stdio.h>
#include <ctime>
#include <cyusb.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace cv;
using namespace std;

#define FRAME_DT 10 // msec
#define PUB_DT 1 // msec

static cyusb_handle *h1 = NULL;
unsigned char *buf;
unsigned int buflen;
const int timeout = 1000;
int transferred = 0;
int posX, posY, posY0, grpAddr, pol;
long long int longTs = 0; // * 10 usec
long long int shortTs = 0; // * 1 msec
long long int timeStamp = 0; // * usec

static Mat image(480,640,CV_8UC1,Scalar(128));
int showImageFlag = 1;
int r;
int n;
