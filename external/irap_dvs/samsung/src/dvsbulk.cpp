#include <dvsbulk.h>

static void print_usage(FILE *stream, int exit_code)
{
	fprintf(stream, "Usage: %s options\n", program_name);
	fprintf(stream, 
		"  -h  --help        Display this usage information.\n"
		"  -l  --load        filename Load I2C script.\n"
		"  -s  --stream      Stream data.\n"
		"  -d  --debug       Print debug information.\n");

	exit(exit_code);
}

static void validate_inputs(void)
{
	if ( (timeout_provided) && (timeout < 0) ) {
		fprintf(stderr,"Must provide a positive value for timeout in seconds\n");
		print_usage(stdout, 1);
	}
}


int I2cSlaveAddr (int index) {
	switch(index){
		case 0 : return(I2C_SLAVE_ADDR_D2FX);
		case 1 : return(I2C_SLAVE_ADDR_DVSL);
		case 2 : return(I2C_SLAVE_ADDR_DVSR);
		case 3 : return(I2C_SLAVE_ADDR_M2PL);
		case 4 : return(I2C_SLAVE_ADDR_M2PR);
		default : return(I2C_SLAVE_ADDR_D2FX);
	}
}

int I2cValueLen (int slvAddr) {
	switch(slvAddr){
		case I2C_SLAVE_ADDR_D2FX : return(I2C_VALUE_LEN_D2FX);
		case I2C_SLAVE_ADDR_DVSL : return(I2C_VALUE_LEN_DVSL);
		case I2C_SLAVE_ADDR_DVSR : return(I2C_VALUE_LEN_DVSR);
		case I2C_SLAVE_ADDR_M2PL : return(I2C_VALUE_LEN_M2PL);
		case I2C_SLAVE_ADDR_M2PR : return(I2C_VALUE_LEN_M2PR);
		default : return(I2C_VALUE_LEN_D2FX);
	}
}

int readI2cReg(int slvAddr, int addr)
{
	int r;
	unsigned char buf[I2C_VALUE_LEN];

	int	I2c_Value_Len = I2cValueLen (slvAddr);

	r = cyusb_control_transfer(h1, 0xC0, 0xBB, slvAddr, addr, buf, I2c_Value_Len, timeout);

	if (r != I2c_Value_Len ) {
		printf("Error reading I2C register\n");
			return -1;
	}

	if (I2c_Value_Len == 1) {
		r = (int) buf[0];
	} else {
		r = (int)(buf[0] << 8) + (int)buf[1];
	}

	if (debug) printf("[I] readI2cReg(%d,%d)=%d\n", slvAddr, addr, r);

	return r;
}

int writeI2cReg(int slvAddr, int addr, int val)
{
	int r;
	unsigned char buf[I2C_VALUE_LEN];

	if (debug) printf("[I] writeI2cReg(%X,%X,%X)\n", slvAddr, addr, val);

	int	I2c_Value_Len = I2cValueLen (slvAddr);

	if (I2c_Value_Len == 1) {
		buf[0] = val & 0xff;
	} else {
		buf[1] = val & 0xff;
		buf[0] = (val >> 8) & 0xff;
	}

	r = cyusb_control_transfer(h1, 0x40, 0xBA, slvAddr, addr, buf, I2c_Value_Len, timeout);

	if (r != I2c_Value_Len ) {
		printf("Error writing I2C register\n");
			return -1;
	}

	return 0;
}

int htoi(char s[], int *i)
{
	int hexdigit;
	int n = 0;
	char c;

	while (true) {
		c = s[*i];

		if(c >='0' && c <='9')
			hexdigit = c - '0';
		else if(c >='a' && c <='f')
			hexdigit = c -'a' + 10;
		else if(c >='A' && c <='F')
			hexdigit = c -'A' + 10;
		else
			return n;

		n = 16 * n + hexdigit;

		*i = *i + 1;
	}
}

int parseString(char *s, int *slvAddr, int *adr, int *val) {
	int i = 0;
	int *dst;
	int mode = 10, ret;

	dst = slvAddr;
	while (true) {
		switch (tolower(s[i])) {
			case 0   : return mode;
			case '\n': return mode;
			case '/' : return mode;

			case ' ' : break;
			case '\t': break;
			case ':' : break;
			case '=' : break;

			case 'w':
				if (tolower(s[i+1]) == 'a' && tolower(s[i+2]) == 'i' && tolower(s[i+3]) == 't') {
					i = i + 3;
					mode = 20;
					dst = val;
					break;
				} else {
					return -1;
				}

			case '0':
			case '1':
			case '2':
			case '3':
			case '4':
			case '5':
			case '6':
			case '7':
			case '8':
			case '9':
			case 'a':
			case 'b':
			case 'c':
			case 'd':
			case 'e':
			case 'f':
				*dst = htoi(s, &i);
				if (debug) printf("[I] Mode(%d): Value(%X) : (%X, %X, %X)\n", mode, *dst, *slvAddr, *adr, *val);
				switch (mode) {
					case 10 :
						mode = 11;
						dst = adr;
						break;
					case 11 :
						mode = 12;
						dst = val;
						break;
					case 12 :
						mode = 1;
						break;
					case 20 :
						mode = 2;
						break;
					default :
						break;
				}
				break;

			default: return -1;
		}
		i++;
	}
}

void loadScript(char *s)
{
	int slvAddr, adr, val;

	if (debug) printf("[I] loadScript(%s)\n", s);

	const int buflen = 1000;
	char buf[buflen];
	FILE * file;

	file = fopen(s , "r");
	if (!file) {
		printf("Error opening file %s\n", s);
		return;
	}

	while (fgets(buf, buflen, file)!=NULL) {

		if (debug) printf("%s",buf);

		switch (parseString(buf, &slvAddr, &adr, &val)) {
			case 1:
				if (debug) printf("[I] I2C (%X,%X,%X)\n", slvAddr, adr, val);
				writeI2cReg(slvAddr, adr, val);
				break;
			case 2:
				if (debug) printf("[I] WAIT (%d)\n", val);
				usleep(val*1000);
				break;
			default: continue;
		}
	}

	fclose(file);
}

int ahtoi(char s[])
{
	int hexdigit;
	int inhex = 1;
	int n = 0;
	int i = 0;

	if (s[i] == '0')
	{
		++i;
		if(s[i] == 'x' || s[i] == 'X') ++i;
		else return atoi(s);
	} else return atoi(s);

	return htoi(s, &i);
}

void Enqueue(void *pkt, int len) {
	pthread_mutex_lock(&queue_mutex);

	struct Node* temp = 
		(struct Node*)malloc(sizeof(struct Node));
	temp->data = pkt;
	temp->len = len;
	temp->next = NULL;

	if(front == NULL && rear == NULL){
		front = rear = temp;
	} else {
		rear->next = temp;
		rear = temp;
	}

	pthread_mutex_unlock(&queue_mutex);

	if (debug) printf("[I] Enqueued packet, len=%d\n", len);
}

void *Dequeue(int *len) {

	*len = 0;
	void *data = NULL;
	pthread_mutex_lock(&queue_mutex);

	struct Node* temp = front;
	if(front == NULL) {
		if (debug) printf("Queue is Empty\n");
	} else {
		if(front == rear) {
			data = front->data;
			*len = front->len;
			front = rear = NULL;
		} else {
			front = front->next;
			data = front->data;
			*len = front->len;
		}
		free(temp);
	}
	pthread_mutex_unlock(&queue_mutex);

	return data;
}


void DecodePacket(unsigned char *pkt, int pktlen, short int MaxEventsPerPixel) {

	if (pktlen < 4) return;	// Unexpected invalid packet
	if (pktlen % 4) pktlen = (pktlen / 4) * 4;

	for (int i = 0; i < pktlen; i += 4) {

		switch (pkt[i]) {

			case (0x66) :
				printf ("T");
				break;
			case (0x99) :
				printf ("G");
				break;
			case (0xcc) :
				printf ("E");
				break;
			default :
				break;

		}

	}
	printf (" %d\n", pktlen/4);

}




static void *reader(void *arg1)
{
	int r;
	void *buf;
	int transferred = 0;
	int nPkt = 0;

	while (1) {
		buf = malloc(buflen);
		r = cyusb_bulk_transfer(h1, 0x81, (unsigned char*)buf, buflen, &transferred, timeout * 1000);
		if ( r == 0 ) {
			if (debug) printf("[I] Received packet %d, len=%d\n", nPkt, buflen);
			Enqueue(buf, buflen);
			nPkt++;
			continue;
		} else {
			cyusb_error(r);
			cyusb_close();
			return NULL;
		}
	}
}

static void *processor(void *arg1)
{
	int r;
	unsigned char *buf;
	int transferred = 0;
	int nPkt = 0;
	int pktlen;

	if (debug) printf("[I] Processor thread started\n");

	while (1) {
		buf = (unsigned char *)Dequeue(&pktlen);
		if (buf) {
			// process
			if (debug) printf("[I] Dequeued packet %d, len=%d\n", nPkt, pktlen);
			DecodePacket(buf, pktlen, 2);
			free(buf);
			nPkt++;
		}
		sleep(0);
	} 
}


int main(int argc, char **argv)
{

	int r;
	char user_input = 'n';
	pthread_t tidStream, tidProcess;
	int slvAddr = 0, addr = 0;
	int val = 0;
	int cmd = 0;
	char *filename;

	program_name = argv[0];

	if (argc < 2) {
		print_usage (stdout, 1);
	}

	while ( (next_option = getopt_long(argc, argv, short_options, long_options, NULL) ) != -1 ) {
		switch ( next_option ) {
			case 'h': /* -h or --help */
				print_usage(stdout, 0);
			case 'l':
				filename = optarg;
				cmd = next_option;
				break;
			case 's':
				cmd = next_option;
				break;
			case 'd':
				debug = true;
				break;
			case '?': /* Invalid option */
				print_usage(stdout, 1);
			default : /* Something else, unexpected */
				abort();
		}
	} 

	validate_inputs();

	r = cyusb_open();
	if ( r < 0 ) {
		printf("Error opening library\n");
		return -1;
	} else if ( r == 0 ) {
		printf("No device found\n");
		return 0;
	}
	if ( r > 1 ) {
		printf("More than 1 devices of interest found. Disconnect unwanted devices\n");
		return 0;
	}
	h1 = cyusb_gethandle(0);
	if ( cyusb_getvendor(h1) != 0x04b4 ) {
		printf("Cypress chipset not detected\n");
		cyusb_close();
		return 0;
	}
	r = cyusb_kernel_driver_active(h1, 0);
	if ( r != 0 ) {
		printf("kernel driver active. Exitting\n");
		cyusb_close();
		return 0;
	}
	r = cyusb_claim_interface(h1, 0);
	if ( r != 0 ) {
		printf("Error in claiming interface\n");
		cyusb_close();
		return 0;
	}
//	else printf("Successfully claimed interface\n");


	switch(cmd) {
		case 'r':
				val = readI2cReg(slvAddr, addr);
				printf("%d\n", val);
				break;
		case 'w':
				writeI2cReg(slvAddr, addr, val);
				break;
		case 'l':
				loadScript(filename);
				break;
		case 's':
				r = pthread_create(&tidStream, NULL, reader, NULL);
				r = pthread_create(&tidProcess, NULL, processor, NULL);
				while (1) {
				pause();
				}
				break;
	}

	cyusb_close();
	return 0;
}
