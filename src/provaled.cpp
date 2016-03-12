

#include "mraa.hpp"

#include <iostream>
#include <unistd.h>
#include <enc03r.h>
#include <buzzer.h>
#include <grovegprs.h>
#include "UdpClient.hpp"

using namespace std;
using namespace upm;

// analog voltage, usually 3.3 or 5.0
#define CALIBRATION_SAMPLES 1000

#define NODE "127.0.0.1"
#define SERVICE "41234"


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

void sendSOS(upm::GroveGPRS* sensor) {
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
		sendCommand(sensor, "AT+CMGS=\"+393406407535\"");
		sleep(1);
		sendCommand(sensor, "SOS");
		sleep(1);
		sendCommand(sensor, "\032");
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
			//sendSOS(sensor);
			std::stringstream ss;
		    ss << "{\"n\":\"" << "sosB" << "\",\"v\":" << "true" << "}" << std::endl;
			if ( client.writeData(ss) < 0 ) {
				cout << "Error in writedata" << endl;
			}

		}
		usleep(100000);
	}

	return mraa::SUCCESS;
}
