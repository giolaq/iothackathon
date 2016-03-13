

#include "mraa.hpp"

#include <iostream>
#include <unistd.h>
#include <enc03r.h>
#include <buzzer.h>
#include <grovegprs.h>
#include "UdpClient.hpp"
#include <mma7660.h>
#include <math.h>
#include <ublox6.h>
#include "minmea.h"
#include <sstream>

#define INDENT_SPACES "  "

using namespace std;
using namespace upm;

// analog voltage, usually 3.3 or 5.0
#define CALIBRATION_SAMPLES 1000

#define NODE "127.0.0.1"
#define SERVICE "41234"
const size_t bufferLength = 256;
float lati, longi;


void printUsage(char *progname)
{
	cout << "Usage: " << progname << " [AT command]" << endl;
	cout << endl;

	cout << "If an argument is supplied on the command line, that argument is"
			<< endl;
	cout << "sent to the module and the response is printed out." << endl;
	cout << endl;
	cout << "If no argument is used, then the manufacturer and the current"
			<< endl;
	cout << "saved profiles are queried and the results printed out." << endl;
	cout << endl;
	cout << endl;
}

// simple helper function to send a command and wait for a response
void sendCommand(upm::GroveGPRS* sensor, string cmd)
{
	// commands need to be terminated with a carriage return
	cmd += "\r";

	sensor->writeDataStr(cmd);

	// wait up to 1 second
	if (sensor->dataAvailable(15000))
	{
		cout << "Returned: " << sensor->readDataStr(1024) << endl;
	}
	else
	{
		cerr << "Timed out waiting for response" << endl;
	}
}

void sendSOS(upm::GroveGPRS* sensor, char* mes) {
	// query the module manufacturer
	cout << "Querying module manufacturer (AT+CGMI)..." << endl;
	sendCommand(sensor, "AT+CGMI");

	sleep(1);

	// query the saved profiles
	cout << "Querying the saved profiles (AT&V)..." << endl;
	sendCommand(sensor, "AT&V");


	sendCommand(sensor, "AT+CMGF=1");
	sleep(1);
	//sendCommand(sensor, "AT+CMGS=\"+393274678844\"");
	sendCommand(sensor, "AT+CMGS=\"+393394690504\"");
	sleep(1);
	char message[190];
    memset(message, 0, sizeof message);

	sprintf(message, "%s in Lat %f Long %f", mes, lati, longi);
	sendCommand(sensor, message);
	sleep(1);
	sendCommand(sensor, "\032");
}

void decodeNMEA(char* line){

	switch (minmea_sentence_id(line, false)) {
	case MINMEA_SENTENCE_RMC: {
		struct minmea_sentence_rmc frame;
		if (minmea_parse_rmc(&frame, line)) {
			printf(INDENT_SPACES "$xxRMC: raw coordinates and speed: (%d/%d,%d/%d) %d/%d\n",
					frame.latitude.value, frame.latitude.scale,
					frame.longitude.value, frame.longitude.scale,
					frame.speed.value, frame.speed.scale);
			printf(INDENT_SPACES "$xxRMC fixed-point coordinates and speed scaled to three decimal places: (%d,%d) %d\n",
					minmea_rescale(&frame.latitude, 1000),
					minmea_rescale(&frame.longitude, 1000),
					minmea_rescale(&frame.speed, 1000));
			printf(INDENT_SPACES "$xxRMC floating point degree coordinates and speed: (%f,%f) %f\n",
					minmea_tocoord(&frame.latitude),
					minmea_tocoord(&frame.longitude),
					minmea_tofloat(&frame.speed));
				lati = minmea_tocoord(&frame.latitude);
				longi = minmea_tocoord(&frame.longitude);
		}
		else {
			printf(INDENT_SPACES "$xxRMC sentence is not parsed\n");
		}
	} break;

	case MINMEA_SENTENCE_GGA: {
		struct minmea_sentence_gga frame;
		if (minmea_parse_gga(&frame, line)) {
			printf(INDENT_SPACES "$xxGGA: fix quality: %d\n", frame.fix_quality);
		}
		else {
			printf(INDENT_SPACES "$xxGGA sentence is not parsed\n");
		}
	} break;

	case MINMEA_SENTENCE_GST: {
		struct minmea_sentence_gst frame;
		if (minmea_parse_gst(&frame, line)) {
			printf(INDENT_SPACES "$xxGST: raw latitude,longitude and altitude error deviation: (%d/%d,%d/%d,%d/%d)\n",
					frame.latitude_error_deviation.value, frame.latitude_error_deviation.scale,
					frame.longitude_error_deviation.value, frame.longitude_error_deviation.scale,
					frame.altitude_error_deviation.value, frame.altitude_error_deviation.scale);
			printf(INDENT_SPACES "$xxGST fixed point latitude,longitude and altitude error deviation"
					" scaled to one decimal place: (%d,%d,%d)\n",
					minmea_rescale(&frame.latitude_error_deviation, 10),
					minmea_rescale(&frame.longitude_error_deviation, 10),
					minmea_rescale(&frame.altitude_error_deviation, 10));
			printf(INDENT_SPACES "$xxGST floating point degree latitude, longitude and altitude error deviation: (%f,%f,%f)",
					minmea_tofloat(&frame.latitude_error_deviation),
					minmea_tofloat(&frame.longitude_error_deviation),
					minmea_tofloat(&frame.altitude_error_deviation));
		}
		else {
			printf(INDENT_SPACES "$xxGST sentence is not parsed\n");
		}
	} break;

	case MINMEA_SENTENCE_GSV: {
		struct minmea_sentence_gsv frame;
		if (minmea_parse_gsv(&frame, line)) {
			printf(INDENT_SPACES "$xxGSV: message %d of %d\n", frame.msg_nr, frame.total_msgs);
			printf(INDENT_SPACES "$xxGSV: sattelites in view: %d\n", frame.total_sats);
			for (int i = 0; i < 4; i++)
				printf(INDENT_SPACES "$xxGSV: sat nr %d, elevation: %d, azimuth: %d, snr: %d dbm\n",
						frame.sats[i].nr,
						frame.sats[i].elevation,
						frame.sats[i].azimuth,
						frame.sats[i].snr);
		}
		else {
			printf(INDENT_SPACES "$xxGSV sentence is not parsed\n");
		}
	} break;

	case MINMEA_INVALID: {
		printf(INDENT_SPACES "$xxxxx sentence is not valid\n");
	} break;

	default: {
		printf(INDENT_SPACES "$xxxxx sentence is not parsed\n");
	} break;
	}
}
/*
 * On board LED blink C++ example
 *
 * Demonstrate how to blink the on board LED, writing a digital value to an
 * output pin using the MRAA library.
 * No external hardware is needed.
 *
 * - digital out: on board LED
 *
 * Additional linker flags: none
 */

int main() {
	// select onboard LED pin based on the platform type
	// create a GPIO object from MRAA using it
	mraa::Platform platform = mraa::getPlatformType();
	mraa::Gpio* d_pin = NULL;

	// Instantiate a ENC03R on analog pin A0
	ENC03R *gyro = new ENC03R(0);

	bool cw = false;
	bool ccw = true;
	int count = 0;


	// Instantiate a GroveGPRS Module on UART 0
	GroveGPRS* sensor = new GroveGPRS(0);

	// create Buzzer instance
	Buzzer* sound = new Buzzer(5);
	// print sensor name
	std::cout << sound->name() << std::endl;


	MMA7660 *accel = new MMA7660(MMA7660_I2C_BUS,
			MMA7660_DEFAULT_I2C_ADDR);
	// place device in standby mode so we can write registers
	accel->setModeStandby();

	// enable 64 samples per second
	accel->setSampleRate(upm::MMA7660::AUTOSLEEP_64);

	// place device into active mode
	accel->setModeActive();



	// UdpClient class is wrapper for sending UDP data to iotkit-agent
	UdpClient client;
	if (client.connectUdp(NODE, SERVICE) < 0) {
		std::cerr << "Connection to iotkit-agent failed, exiting" << std::endl;
		return mraa::ERROR_UNSPECIFIED;
	}



	// Set the baud rate, 19200 baud is the default.
	if (sensor->setBaudRate(19200) != mraa::SUCCESS)
	{
		cerr << "Failed to set tty baud rate" << endl;
		return 1;
	}


	//! [Interesting]
	// Instantiate a Ublox6 GPS device on uart 0.
	upm::Ublox6* nmea = new upm::Ublox6(0);

	// make sure port is initialized properly.  9600 baud is the default.
	if (!nmea->setupTty(B9600))
	{
		cerr << "Failed to setup tty port parameters" << endl;
		return 1;
	}

	// Collect and output NMEA data.  There are various libraries out on
	// the Internet, such as tinyGPS or tinyGPS++ that can handle
	// decoding NMEA data and presenting it in a more easily accessible
	// format.  This example will just check for, and read raw NMEA data
	// from the device and output it on stdout.

	// This device also supports numerous configuration options, which
	// you can set with writeData().  Please refer to the Ublox-6 data
	// sheet for further information on the formats of the data sent and
	// received, and the various operating modes available.

	char nmeaBuffer[bufferLength];




	switch (platform) {
	case mraa::INTEL_GALILEO_GEN1:
		d_pin = new mraa::Gpio(3, true, true);
		break;
	case mraa::INTEL_GALILEO_GEN2:
		d_pin = new mraa::Gpio(13, true, false);
		break;
	case mraa::INTEL_EDISON_FAB_C:
		d_pin = new mraa::Gpio(13, true, false);
		break;
	default:
		std::cerr << "Unsupported platform, exiting" << std::endl;
		return mraa::ERROR_INVALID_PLATFORM;
	}
	if (d_pin == NULL) {
		std::cerr << "Can't create mraa::Gpio object, exiting" << std::endl;
		return mraa::ERROR_UNSPECIFIED;
	}

	// set the pin as output
	if (d_pin->dir(mraa::DIR_OUT) != mraa::SUCCESS) {
		std::cerr << "Can't set digital pin as output, exiting" << std::endl;
		return MRAA_ERROR_UNSPECIFIED;
	}

	// The first thing we need to do is calibrate the sensor.
	cout << "Please place the sensor in a stable location, and do not" << endl;
	cout << "move it while calibration takes place." << endl;
	cout << "This may take a couple of minutes." << endl;

	gyro->calibrate(CALIBRATION_SAMPLES);
	cout << "Calibration complete.  Reference value: "
			<< gyro->calibrationValue() << endl;
	sound->stopSound();

	// loop forever toggling the on board LED every second
	for (;;) {
		unsigned int val = gyro->value();
		double av = gyro->angularVelocity(val);

		//cout << "Raw value: " << val << ", "
		//<< "angular velocity: " << av << " deg/s" << endl;


		float ax, ay, az;

		accel->getAcceleration(&ax, &ay, &az);
		/*cout << "Acceleration: x = " << ax
				<< "g y = " << ay
				<< "g z = " << az
				<< "g" << endl;*/

		float modAcc = sqrt(ax*ax + ay*ay+ az*az);
		std::stringstream ss;
		cout <<"ACCElerometer in fall "<<modAcc<<endl;
		ss << "{\"n\":\"" << "modAcc" << "\",\"v\":" << modAcc << "}" << std::endl;
		if ( client.writeData(ss) < 0 ) {
			cout << "Error in writedata" << endl;
		}
		if ( modAcc > 1.6 ) {
			cout << "SOS " << endl;
			// play sound (DO, RE, MI, etc...), pausing for 0.1 seconds between notes
			for (int chord_ind = 0; chord_ind < 3; chord_ind++) {
				// play each note for one second
				std::cout << sound->playSound(DO, 100000) << std::endl;
				usleep(10000);
				std::cout << sound->playSound(SOL, 100000) << std::endl;

			}
			sendSOS(sensor, "Man Down");

			sound->stopSound();
		}



		if (ccw) {
			if (av < -1000) {
				count++;
				cw = true;
				ccw = false;
				cout << "CCW" << endl;

			}
		} else if (cw) {
			if (av  > 1000) {
				count++;
				cw = false;
				ccw = true;
				cout << "CW" << endl;

			}
		}

		if (count == 3) {
			count = 0;
			ccw = true;
			cw = false;
			cout << "SOS " << endl;
			// play sound (DO, RE, MI, etc...), pausing for 0.1 seconds between notes
			for (int chord_ind = 0; chord_ind < 3; chord_ind++) {
				// play each note for one second
				std::cout << sound->playSound(SI, 100000) << std::endl;
				usleep(10000);
				std::cout << sound->playSound(FA, 100000) << std::endl;

			}

			sound->stopSound();
            printf("SOS Lat %f Long %f", lati, longi);
			sendSOS(sensor, "SOS Request");
			std::stringstream ss;
			ss << "{\"n\":\"" << "sosB" << "\",\"v\":" << "true" << "}" << std::endl;
			if ( client.writeData(ss) < 0 ) {
				cout << "Error in writedata" << endl;
			}

		}


		char line[MINMEA_MAX_LENGTH];
		char* line2;
		std::stringstream stream;
		string streamString("");

		// we don't want the read to block in this example, so always
		// check to see if data is available first.
		if (nmea->dataAvailable())
		{
			int rv = nmea->readData(nmeaBuffer, bufferLength);

			if (rv > 0)
			{
				line2 = strtok(strdup(nmeaBuffer), "\r\n");
				while (line2 != NULL) {
					//write(1, line2, rv);
					decodeNMEA(line2);

					if (rv < 0) // some sort of read error occured
					{
						cerr << "Port read error." << endl;
						break;
					}
					line2 = strtok(NULL, "\r\n");
				}
			}


		}

		usleep(100000);
	}

	return mraa::SUCCESS;
}
