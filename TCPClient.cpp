/* 

* TCP Example For KIST Monkey Experiment 

* TCP_HYUSPA.cpp

* Created on: Mar 2, 2020

*     Author: Sunhong Kim

*/

 

#include "Poco/Net/Net.h"

#include "Poco/Net/StreamSocket.h"

#include "Poco/Net/SocketAddress.h"

#include "Poco/Dynamic/Var.h"

#include "Poco/Exception.h"

#include "Poco/Timer.h"

#include "Poco/Stopwatch.h"

#include "Poco/Thread.h"

#include "Poco/DateTime.h"

#include "Poco/Timespan.h"

#include "Poco/NumericString.h"

#include <iostream>

#include <time.h>

 

using namespace Poco;

using namespace Poco::Dynamic;

using Poco::Net::SocketAddress;

using Poco::Net::StreamSocket;

using Poco::Net::Socket;

using Poco::Timer;

using Poco::TimerCallback;

using Poco::Thread;

using Poco::Stopwatch;

using namespace std;

 

const std::string hostname = "127.0.0.1"; //localhost IP Address
//const std::string hostname = "192.168.0.39"; //STEP2 IP Address 
//const std::string hostname = "192.168.0.100"; //STEP2 IP Address Monkey
//const std::string hostname = "192.168.1.18"; //STEP2 IP Address Tensegrity
//const std::string hostname = "192.168.0.122"; //STEP2 IP Address Tensegrity
const Poco::UInt16 PORT = 9911;
enum {
	SIZE_HEADER = 52,
	SIZE_COMMAND = 4,
	SIZE_HEADER_COMMAND = 56,
	SIZE_DATA_MAX = 37,
	SIZE_DATA_ASCII_MAX = 32
};
union Data

{

	unsigned char byte[1];
	float value[9];

};

 

int main()

{

	StreamSocket ss;


	Data data_rev, data;

	unsigned char writeBuff[SIZE_DATA_MAX];
	ss.connect(SocketAddress(hostname, PORT));
	Timespan timeout(1, 0);
	while (ss.poll(timeout, Poco::Net::Socket::SELECT_WRITE) == false){
		cout << "Connecting to server..." << endl;
	}
	Data data_send;
	int count = 0;
	while(true){
		for(int i =0;i<9;i++)
			data.value[i] = 0.0;
		data.value[0] = count++;
		if(count>5) count = 0;
		memcpy(writeBuff, data.byte, SIZE_DATA_MAX);
		ss.sendBytes(writeBuff, SIZE_DATA_MAX);
		usleep(100);
	}
	ss.close();


	return 0;

}

 
