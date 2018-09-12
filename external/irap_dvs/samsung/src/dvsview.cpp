#include <dvsview.h>

struct timeval structTime;
double start_time, current_time;

int main(int argc, char **argv) {

	r = cyusb_open();
	if ( r < 0 ) {
		printf("Error opening library\n");
		return -1;
	}
	else if ( r == 0 ) {
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


	buflen = cyusb_get_max_packet_size(h1, 0x81);
	printf("buffer size=%d\n", buflen);
	buf = (unsigned char *) malloc (buflen + 1);

	namedWindow("raw", WINDOW_AUTOSIZE);
	image.setTo(cv::Scalar(128));

	start_time = std::clock() / (double)(CLOCKS_PER_SEC / 1000);

	while (true) {

		r = cyusb_bulk_transfer(h1, 0x81, buf, buflen, &transferred, timeout);

		// printf ("%d\t%d\n", buflen, transferred);

		if (transferred % 4) transferred = (transferred / 4) * 4;

		for(int i=0;i<transferred; i+=4){

			switch (buf[i]) {

				case (0x66) :	// 0110 0110 | ---- TTTT | TTTT TTTT | TTTT TTTT	Reference Timestamp
					longTs = ((buf[i+1] & 0x0F) << 16) | ((buf[i+2] & 0xFF) << 8) | (buf[i+3] & 0xFF);
					break;

				case (0x99) :	// 1001 1001 | ---- TTTT | TTTT TTCC | CCCC CCCC	Column Address
					shortTs = ((buf[i+1] & 0x0F) << 6) | ((buf[i+2] & 0xFC) >> 2);
					timeStamp = (longTs << 10) | shortTs;
					posX = (((buf[i+2] & 0x03) << 8) | (buf[i+3] & 0xFF)) - 1;	// Offset
					if ((posX < 0) || (posX >= 640)) continue;
					break;

				case (0xcc) :	// 1100 1100 | --GG GGGG | EEEE EEEE | EEEE EEEE	Group Event (Off / On)
					grpAddr = buf[i+1] & 0x3F;
					if ((grpAddr <= 0) || (grpAddr > 60)) continue;
					posY0 = (grpAddr -1) << 3;
					if (buf[i+2]) {	// Off Event
						pol = 1;
						for (n=0; n<8; n++) {
							if ((buf[i+2] >> n) & 0x01) {
								posY = posY0 + n;
								image.at<char> (posY, posX) = 0;
							}
						}
					}
					if (buf[i+3]) {	// On Event
						pol = 0;
						for (n=0; n<8; n++) {
							if ((buf[i+3] >> n) & 0x01) {
								posY = posY0 + n;
								image.at<char> (posY, posX) = 255;
							}
						}
					}
					break;

				case (0x00) :	// 0000 0000 | Padding
					break;

				default :		// Error - should not happen
					break;

			} // switch:buf[i]

		} // for:i

		gettimeofday(&structTime,NULL);
		current_time = structTime.tv_sec * 1000 + structTime.tv_usec / 1000.0;
		if(current_time-start_time > FRAME_DT){
			start_time = current_time;
			if(showImageFlag==1){
				imshow("raw",image);
			}
			char wk = waitKey(1);
			if(wk==' '){
				showImageFlag = 1-showImageFlag;
			}
			image.setTo(Scalar(128));
		}

	}

	cyusb_close();
	return 0;
}
