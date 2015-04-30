#ifdef WIN32
#include <windows.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#include "hidapi.h"

#define MAX_STR 255

int main(int argc, char* argv[])
{
	int res;
	unsigned char buf[65];
	wchar_t wstr[MAX_STR];
	hid_device *handle;

	// Initialize the hidapi library
	res = hid_init();

	// Open the device using the VID, PID,
	// and optionally the Serial number.
	handle = hid_open(0x4d8, 0x3f, NULL);

	// Read the Manufacturer String
	res = hid_get_manufacturer_string(handle, wstr, MAX_STR);
	wprintf(L"Manufacturer String: %s\n", wstr);

	// Read the Product String
	res = hid_get_product_string(handle, wstr, MAX_STR);
	wprintf(L"Product String: %s\n", wstr);

	// Read the Serial Number String
	res = hid_get_serial_number_string(handle, wstr, MAX_STR);
	wprintf(L"Serial Number String: (%d) %s\n", wstr[0], wstr);

	// Read Indexed String 1
	res = hid_get_indexed_string(handle, 1, wstr, MAX_STR);
	wprintf(L"Indexed String 1: %s\n", wstr);

/*
	// Print stuff (cmd 0x80). The first byte is the report number (0x0).
	printf("enter a row (row<64):\n");
	scanf("%d", &buf[2]);
	printf("enter a string < 25 characters long:\n"); 
	char message[25];
	scanf("%s", message);
	
	buf[0] = 0x0;
	buf[1] = 0x80;
	//buf[2] is the row value
	int i;
	for (i = 0; i < 25; i++)
	{
		if(i<strlen(message)) {
			buf[i+3] = message[i];
		}
		else {
			buf[i+3] = 32;
		}
	}
	
	res = hid_write(handle, buf, 65);
*/

	// Request state (cmd 0x81). The first byte is the report number (0x0).
	int DATAPTS = 10;
	float x[DATAPTS];
	float y[DATAPTS];
	float z[DATAPTS];
	
	int datapts = 0;
	while(datapts < 10)
	{
		buf[0] = 0x0;
		buf[1] = 0x81;
		res = hid_write(handle, buf, 65);

		// Read requested state
		res = hid_read(handle, buf, 65);
		
		if (buf[1] == 0) 
		{
			;
		}
		else if (buf[1] ==1)
		{
			short xaxis = buf[2]<<8 | buf[3];
			short yaxis = buf[4]<<8 | buf[5];
			short zaxis = buf[6]<<8 | buf[7];
			printf("%3.2f,%3.2f,%3.2f\n", xaxis/16000.0, yaxis/16000.0, zaxis/16000.0);
			x[datapts] = xaxis/16000.0; 
			y[datapts] = yaxis/16000.0;
			z[datapts] = zaxis/16000.0;
			datapts++;
		}
	}
	
	FILE *ofp;
	ofp = fopen("accels.txt", "w");
	int i;
	for (i=0; i<DATAPTS; i++) 
	{
		fprintf(ofp,"%3.2f %3.2f %3.2f\r\n",x[i],y[i],z[i]);
	}
	fclose(ofp);
		
	// Finalize the hidapi library
	res = hid_exit();

	return 0;
}