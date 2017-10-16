//============================================================================
// Name        : modbus.cpp
// Author      : ismail baslar
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <modbus/modbus.h>
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <bitset>
#include <thread>

//static const char* return_from_alarm_register(int alarm_code);
#define BUG_REPORT(_cond, _format, _args ...) \
    printf("\nLine %d: assertion error for '%s': " _format "\n", __LINE__, # _cond, ## _args)

#define ASSERT_TRUE(_cond, _format, __args...) {  \
    if (_cond) {                                  \
        printf("Read or write debugging is OK\n");                           \
    } else {                                      \
        BUG_REPORT(_cond, _format, ## __args);    \
    }                                             \
};


#define INPUTREGISTER1 0x40000//40001 aslinda
#define INPUTREGISTER2 0x40001
#define OUTPUTREGISTER1 0x40002
#define RIGHTDOMESLOPE 0x40008
#define LEFTDOMESLOPE 0x40009
#define HYDROLICTEMPERATURE 0x40022
#define DOMETEMPERATURE 0x40023
#define RIGHTDOMEPOSITIONCOMMAND 0x40006
#define LEFTDOMEPOSITIONCOMMAND 0x40007


#define CONTROLWORD1 0x40003

#define EMERGENCY_STOP 0
#define RESET_PLC 1
#define RUN_HYDROLIC 2
#define CLOSE_SHUTTER_LEFT 3
#define OPEN_SHUTTER_LEFT 4
#define CLOSE_SHUTTER_RIGHT 5
#define OPEN_SHUTTER_RIGHT 6
#define DOME_LEFT_POSITION 7
#define DOME_RIGHT_POSITION 8

#define STATUSWORD1 0x40004
#define CLOSING_SHUTTER_LEFT 0
#define CLOSING_SHUTTER_WRIGHT 1
#define OPENING_DOME_LEFT 2
#define OPENING_DOME_WRIGHT 3
#define ALARM 4
#define HEARTBIT 5

#define ALARMWORD1 0x40005
#define PHASE_PROTECTION_ALARM 0
#define HYDROLIC_MOTOR_ERROR 1
#define HYDROLIC_DIRT_ERROR 2
#define HYDROLIC_OIL_LEVEL_WARNING 3
#define HYDROLIC_OIL_LEVEL_ALARM 4
#define RAIN_SENSOR_ALARM 5
#define COMMUNICATION_ERROR 6
#define SHUTTER_TIME_OUT 7
#define ELECTRICITY_POWER_CUT 8
#define RIGHT_SHUTTER_SWITCH_ALARM 9
#define LEFT_SHUTTER_SWITCH_ALARM 10
#define RIGHT_SHUTTER_SLOPE_SENSOR_ALARM 11
#define LEFT_SHUTTER_SLOPE_SENSOR_ALARM 12
using namespace std;




static const char* return_from_alarm_register(int alarm_code)
{

	const char *alarm;

	  switch (alarm_code)
	        {
	        case PHASE_PROTECTION_ALARM:         alarm = "Error with the electricity phases";           break;
	        case HYDROLIC_MOTOR_ERROR:         alarm = "Hydrolic motor problem";                      break;
	        case HYDROLIC_DIRT_ERROR :         alarm = "Hydrolic dirty problem";                      break;
	        case HYDROLIC_OIL_LEVEL_WARNING:         alarm = "Hydrolic oil level warning";                  break;
	        case HYDROLIC_OIL_LEVEL_ALARM:         alarm = "Hydrolic oil level warning";                  break;
	        case RAIN_SENSOR_ALARM:         alarm = "RAIN SENSOR ALARM";                           break;
	        case COMMUNICATION_ERROR:         alarm = "COMMUNICATION ERROR";                         break;
	        case SHUTTER_TIME_OUT:         alarm = "SHUTER TIME OUT ALARM";                       break;
	        case ELECTRICITY_POWER_CUT:         alarm = "ELECTRICITY POWER CUT";                       break;
	        case RIGHT_SHUTTER_SWITCH_ALARM:         alarm = "RIGTH_SHUTTER_SWITCH ALARM";                  break;
	        case LEFT_SHUTTER_SWITCH_ALARM:        alarm = "LEFT SHUTTER SWITCH ALARM";                   break;
	        case RIGHT_SHUTTER_SLOPE_SENSOR_ALARM:        alarm = "RIGHT SHUTTER SLOPE SENSOR ALARM";            break;
	        case LEFT_SHUTTER_SLOPE_SENSOR_ALARM:        alarm = "LEFT SHUTTER SLOPE ALARM";                    break;

	        default:
	                alarm = "Unknown alarm"; break;
	        }

	return alarm;

}
static const char* return_from_status_register(int status_code)
{

	const char *status;

	  switch (status_code)
	        {
	case CLOSING_SHUTTER_LEFT:
		status = "Left shutter is closing";
		break;
	case CLOSING_SHUTTER_WRIGHT:
		status = "Right shutter is closing";
		break;
	case OPENING_DOME_LEFT:
		status = "Left shutter is opening";
		break;
	case OPENING_DOME_WRIGHT:
		status = "Right shutter is opening";
		break;
	case ALARM:
		status = "ALARM";
		break;

	        default:
	                status = "Unknown state"; break;
	        }

	return status;

}



void checkAlarms(uint16_t * tab_rp_registers)
{
	std::bitset<16> IntBits = tab_rp_registers[0];
	bool is_set;
	for (int i = 0; i < 16; i++) {
		is_set = IntBits.test(i);
		if (is_set)
			std::cout << return_from_alarm_register(i) << std::endl;
	}
}

void checkStates(uint16_t * tab_rp_registers)
{
	std::bitset<16> IntBits = tab_rp_registers[0];
	bool is_set;
	for (int i = 0; i < 16; i++) {
		is_set = IntBits.test(i);
		if (is_set)
			std::cout << return_from_status_register(i) << std::endl;
	}
}


int openLeftShutter(uint16_t * tab_rp_registers,modbus_t * mb, uint16_t UT_REGISTERS_ADDRESS)
{
	std::bitset<16> bitSet = tab_rp_registers[0];
	bitSet.set(OPEN_SHUTTER_LEFT);
	int rc = modbus_write_register(mb, UT_REGISTERS_ADDRESS, bitSet.to_ulong());
	printf("Open Left Shutter bit is Set \n");
	ASSERT_TRUE(rc == 1, "");
	return rc;
}

int goLeftShutter(uint16_t * tab_rp_registers,modbus_t * mb, uint16_t UT_REGISTERS_ADDRESS)
{
	std::bitset<16> bitSet = tab_rp_registers[0];
	bitSet.set(DOME_LEFT_POSITION);
	int rc = modbus_write_register(mb, UT_REGISTERS_ADDRESS, bitSet.to_ulong());
	printf("Open Left Shutter bit is Set \n");
	ASSERT_TRUE(rc == 1, "");
	return rc;
}


int closeLeftShutter(uint16_t * tab_rp_registers,modbus_t * mb, uint16_t UT_REGISTERS_ADDRESS)
{
	std::bitset<16> bitSet = tab_rp_registers[0];
	bitSet.set(CLOSE_SHUTTER_LEFT);
	int rc = modbus_write_register(mb, UT_REGISTERS_ADDRESS, bitSet.to_ulong());
	printf("Close Left shutter bit is set\n ");
	ASSERT_TRUE(rc == 1, "");
	return rc;
}


int openRightShutter(uint16_t * tab_rp_registers,modbus_t * mb, uint16_t UT_REGISTERS_ADDRESS)
{
	std::bitset<16> bitSet = tab_rp_registers[0];
	bitSet.set(OPEN_SHUTTER_RIGHT);
	int rc = modbus_write_register(mb, UT_REGISTERS_ADDRESS, bitSet.to_ulong());
	printf("Open right shutter bit is set\n: ");
	ASSERT_TRUE(rc == 1, "");
	return rc;
}

int goRightShutter(uint16_t * tab_rp_registers,modbus_t * mb, uint16_t UT_REGISTERS_ADDRESS)
{
	std::bitset<16> bitSet = tab_rp_registers[0];
	bitSet.set(DOME_RIGHT_POSITION);
	int rc = modbus_write_register(mb, UT_REGISTERS_ADDRESS, bitSet.to_ulong());
	printf("Open Left Shutter bit is Set \n");
	ASSERT_TRUE(rc == 1, "");
	return rc;
}

int closeRightShutter(uint16_t * tab_rp_registers,modbus_t * mb, uint16_t UT_REGISTERS_ADDRESS)
{
	std::bitset<16> bitSet = tab_rp_registers[0];
	bitSet.set(CLOSE_SHUTTER_RIGHT);
	int rc = modbus_write_register(mb, UT_REGISTERS_ADDRESS, bitSet.to_ulong());
	printf("Close right shutter bit is set: \n");
	ASSERT_TRUE(rc == 1, "");
	return rc;
}


int  emergencyStop(uint16_t * tab_rp_registers,modbus_t * mb, uint16_t UT_REGISTERS_ADDRESS)
{
	std::bitset<16> bitSet = tab_rp_registers[0];
	bitSet.set(EMERGENCY_STOP);
	int rc = modbus_write_register(mb, UT_REGISTERS_ADDRESS, bitSet.to_ulong());
	printf("Emergency stop bit is set: \n");
	ASSERT_TRUE(rc == 1, "");
	return rc;
}

int resetButton(uint16_t * tab_rp_registers,modbus_t * mb, uint16_t UT_REGISTERS_ADDRESS)
{
	std::bitset<16> bitSet = tab_rp_registers[0];
	bitSet.set(RESET_PLC);
	int rc = modbus_write_register(mb, UT_REGISTERS_ADDRESS, bitSet.to_ulong());
	printf("Reset button bit is set: \n");
	ASSERT_TRUE(rc == 1, "");
	return rc;
}

int runHydrolic(uint16_t * tab_rp_registers,modbus_t * mb, uint16_t UT_REGISTERS_ADDRESS)
{
	std::bitset<16> bitSet = tab_rp_registers[0];
	bitSet.set(RUN_HYDROLIC);
	int rc = modbus_write_register(mb, UT_REGISTERS_ADDRESS, bitSet.to_ulong());
	printf("run Hydrolic bit is set: \n");
	ASSERT_TRUE(rc == 1, "");
	return rc;
}

int setHearBeat2(uint16_t * tab_rp_registers,modbus_t * mb, uint16_t UT_REGISTERS_ADDRESS)
{
	std::bitset<16> bitSet = tab_rp_registers[0];
	bitSet.set(9);
	int rc = modbus_write_register(mb, UT_REGISTERS_ADDRESS, bitSet.to_ulong());
	printf("reset heartbeat bit is reset: \n");
	ASSERT_TRUE(rc == 1, "");
	return rc;
}

int resetHearBeat2(uint16_t * tab_rp_registers,modbus_t * mb, uint16_t UT_REGISTERS_ADDRESS)
{
	std::bitset<16> bitSet = tab_rp_registers[0];
	bitSet.reset(9);
	int rc = modbus_write_register(mb, UT_REGISTERS_ADDRESS, bitSet.to_ulong());
	printf("reset heartbeat bit is reset: \n");
	ASSERT_TRUE(rc == 1, "");
	return rc;
}



int resetHeartBit(uint16_t * tab_rp_registers,modbus_t * mb, uint16_t UT_REGISTERS_ADDRESS)
{
	std::bitset<16> bitSet = tab_rp_registers[0];
	std::cout<<bitSet.test(HEARTBIT)<<std::endl;
	bitSet.reset(HEARTBIT);
	std::cout<<bitSet.test(HEARTBIT)<<std::endl;
	int rc = modbus_write_register(mb, UT_REGISTERS_ADDRESS, bitSet.to_ulong());
	printf("heartBeat reset \n");
	ASSERT_TRUE(rc == 1, "");
	return rc;

}

int stopHydrolic(uint16_t * tab_rp_registers,modbus_t * mb, uint16_t UT_REGISTERS_ADDRESS)
{
	std::bitset<16> bitSet = tab_rp_registers[0];
	bitSet.reset(RUN_HYDROLIC);
	int rc = modbus_write_register(mb, UT_REGISTERS_ADDRESS, bitSet.to_ulong());
	printf("run Hydrolic bit is reset: \n");
	ASSERT_TRUE(rc == 1, "");
	return rc;
}

int stopAll(uint16_t * tab_rp_registers,modbus_t * mb, uint16_t UT_REGISTERS_ADDRESS)
{

	tab_rp_registers[0]=0x0000;
	int rc = modbus_write_register(mb, UT_REGISTERS_ADDRESS, tab_rp_registers[0]);
	printf("run Stop All: \n");
	ASSERT_TRUE(rc == 1, "");
	return rc;
}




int setRightDomePosition(uint16_t degree,modbus_t * mb, uint16_t UT_REGISTERS_ADDRESS)
{
	int rc = modbus_write_register(mb, UT_REGISTERS_ADDRESS, degree);
	printf("Write to Control register function ... \n");
	ASSERT_TRUE(rc == 1, "");
	return rc;
}

int setLeftDomePosition(uint16_t degree,modbus_t * mb, uint16_t UT_REGISTERS_ADDRESS)
{
	int rc = modbus_write_register(mb, UT_REGISTERS_ADDRESS, degree);
	printf("Write to Control register function ... \n");
	ASSERT_TRUE(rc == 1, "");
	return rc;

}
int writeToControlRegister(int bitNumber,uint16_t * tab_rp_registers,modbus_t * mb, uint16_t UT_REGISTERS_ADDRESS)
{

	  std::bitset<16> bitSet = tab_rp_registers[0];
	  bitSet.set(bitNumber);
	  int rc = modbus_write_register(mb, UT_REGISTERS_ADDRESS, bitSet.to_ulong());
	  printf("Write to Control register function ... \n");
	  ASSERT_TRUE(rc == 1, "");
      return rc;

}

uint16_t * readControlRegister(modbus_t * mb,uint16_t UT_REGISTERS_ADDRESS,uint16_t* tab_rp_registers)
{
			int rc = modbus_read_registers(mb, UT_REGISTERS_ADDRESS,1, tab_rp_registers);
			//printf("2/2 modbus_read_registers: ");
			ASSERT_TRUE(rc == 1, "FAILED (read Control register %d)\n", rc);
			std::cout<<"hex number is "<<std::hex<<tab_rp_registers[0]<<" The address is  "<<UT_REGISTERS_ADDRESS<<std::endl;
			return tab_rp_registers;
}

uint16_t * readStatusRegister(modbus_t * mb,uint16_t UT_REGISTERS_ADDRESS,uint16_t* tab_rp_registers)
{
	int rc = modbus_read_registers(mb, UT_REGISTERS_ADDRESS,1, tab_rp_registers);
	//printf("2/2 modbus_read_registers: ");
	ASSERT_TRUE(rc == 1, "FAILED (read Status register %d)\n", rc);
	// std::cout<<"hex number is "<<std::hex<<tab_rp_registers[0]<<" The address is  "<<UT_REGISTERS_ADDRESS<<std::endl;
	std::cout<<"hex number is "<<std::hex<<tab_rp_registers[0]<<" The address is  "<<UT_REGISTERS_ADDRESS<<" binary value is "<<std::bitset<16>(tab_rp_registers[0])<<std::endl;
	return tab_rp_registers;
}

uint16_t *readAlarmRegister(modbus_t * mb,uint16_t UT_REGISTERS_ADDRESS,uint16_t* tab_rp_registers)
{
	int rc = modbus_read_registers(mb, UT_REGISTERS_ADDRESS,1, tab_rp_registers);
	//printf("2/2 modbus_read_registers: ");
	ASSERT_TRUE(rc == 1, "FAILED (readAlarmRegister %d)\n", rc);
	std::cout<<"hex number is "<<std::hex<<tab_rp_registers[0]<<" The address is  "<<UT_REGISTERS_ADDRESS<<" binary value is "<<std::bitset<16>(tab_rp_registers[0])<<std::endl;
	return tab_rp_registers;
}


uint16_t * readRegister(modbus_t * mb,uint16_t UT_REGISTERS_ADDRESS,uint16_t* tab_rp_registers)
{
	int rc = modbus_read_registers(mb, UT_REGISTERS_ADDRESS,1, tab_rp_registers);
	//printf("2/2 modbus_read_registers: ");
	ASSERT_TRUE(rc == 1, "FAILED (read Status register %d)\n", rc);
	// std::cout<<"hex number is "<<std::hex<<tab_rp_registers[0]<<" The address is  "<<UT_REGISTERS_ADDRESS<<std::endl;
	std::cout<<"non hex number is "<<std::dec<<tab_rp_registers[0]<<" The address is  "<<UT_REGISTERS_ADDRESS<<" binary value is "<<std::bitset<16>(tab_rp_registers[0])<<std::endl;

	std::cout<<"hex number is "<<std::hex<<tab_rp_registers[0]<<" The address is  "<<UT_REGISTERS_ADDRESS<<" binary value is "<<std::bitset<16>(tab_rp_registers[0])<<std::endl;
	return tab_rp_registers;
}


bool isLeftShutterClosing(uint16_t * tab_rp_registers)
{
	bool value = false;
	std::bitset<16> bitSet = tab_rp_registers[0];
	value = bitSet.test(0);
    return value;

}

bool isLeftShutterOpening(uint16_t * tab_rp_registers)
{
	    bool value = false;
		std::bitset<16> bitSet = tab_rp_registers[0];
		value = bitSet.test(2);
	    return value;
}

bool isRightShutterClosing(uint16_t * tab_rp_registers)
{
	    bool value = false;
		std::bitset<16> bitSet = tab_rp_registers[0];
		value = bitSet.test(1);
	    return value;
}

bool isRightShutterOpening(uint16_t * tab_rp_registers)
{
	    bool value = false;
		std::bitset<16> bitSet = tab_rp_registers[0];
		value = bitSet.test(3);
	    return value;
}

bool isThereAlarm(uint16_t * tab_rp_registers)
{
	    bool value = false;
		std::bitset<16> bitSet = tab_rp_registers[0];
		value = bitSet.test(4);
	    return value;
}



modbus_t *mb=NULL;
int main() {

   // modbus_t *mb=NULL;
    int rc;


/*
const uint16_t CONTROL_WORD_REGISTER = CONTROLWORD1;
const uint16_t STATUS_WORD_REGISTER = STATUSWORD1;
const uint16_t ALARM_WORD_REGISTER = ALARMWORD1;
*/

 uint32_t old_response_to_sec;
 uint32_t old_response_to_usec;
 uint32_t new_response_to_sec;
 uint32_t new_response_to_usec;


 mb = modbus_new_tcp("192.168.2.10", 502);
 //mb = modbus_new_tcp("127.0.0.1", 1502);
 if(mb==NULL)
 {
 fprintf(stderr,"unable to allocate libmodbus");
 return -1;
 }

  modbus_set_debug(mb, FALSE);//false yaptım data paketini ekrana yazıyor
  modbus_set_error_recovery(mb, MODBUS_ERROR_RECOVERY_PROTOCOL);
  modbus_set_error_recovery(mb, MODBUS_ERROR_RECOVERY_LINK);

  modbus_get_response_timeout(mb, &old_response_to_sec, &old_response_to_usec);


 if (modbus_connect(mb) == -1)
 {
    fprintf(stderr, "Connection failed: %s\n", modbus_strerror(errno));
    modbus_free(mb);
    return -1;
 }

 modbus_get_response_timeout(mb, &new_response_to_sec, &new_response_to_usec);

 printf("** TESTING IS STARTED **\n");

   printf("1/1 No response timeout modification on connect: ");
   ASSERT_TRUE(old_response_to_sec == new_response_to_sec &&
               old_response_to_usec == new_response_to_usec, "");

/******DUMMY*****/
           uint16_t *anyRegister =  (uint16_t*) malloc(sizeof(uint16_t));
   	        memset(anyRegister, 0, sizeof(uint16_t));

               //write to dummy register
   	   //     rc = modbus_write_register(mb, 0x161, 0xFF);

/*****DUMMY*****/

      int ch;
	  char c=' ';
	  while(c!='x')
	  {
	   int ch;
	   c=' ';

	   printf("\n");



     //	   READ Control Word 40004/////
		printf("READING CONTROL register\n");
		readRegister(mb, CONTROLWORD1, anyRegister);

		printf("----------------------------\n");
		// 		READ Control Word 40004////

		/*get 40000 register INPUTREGISTER1*/
		printf("40000(INPUT REGISTER1) register\n");
		readRegister(mb, INPUTREGISTER1, anyRegister);

		printf("----------------------------\n");
		/*get 40000 register INPUTREGISTER1*/




		 /*get 40001 register INPUTREGISTER2*/
		printf("40001(INPUT REGISTER2) register\n");
		readRegister(mb, INPUTREGISTER2, anyRegister);

		printf("----------------------------\n");
		/*get 40001 register INPUTREGISTER2*/



		 /*get 40002 register OUTPUTREGISTER1*/
		printf("40002(OUTPUT REGISTER1) register\n");
		readRegister(mb, OUTPUTREGISTER1, anyRegister);
		/*****/

		 /*get 40003 register OUTPUTREGISTER1*/
				printf("40003(OUTPUT REGISTER1) register\n");
				readRegister(mb, CONTROLWORD1, anyRegister);
				/*get 40003*/


	    //get right slope
	       printf("READING RIGHT SLOPE register\n");
	     readRegister(mb,RIGHTDOMESLOPE,anyRegister);

	       printf("----------------------------\n");
	    //get right slope


	     //get left slope
	       printf("READING LEFT SLOPE register\n");
	      readRegister(mb,LEFTDOMESLOPE,anyRegister);

	       printf("--------------- ------------\n");



	       //get status register
	     	       printf("READING STATUSWORD1 register\n");
	     	      readRegister(mb,STATUSWORD1,anyRegister);

	     	       printf("--------------- ------------\n");






       printf("\n");

	   printf("0 to open left shutter\n");
	   printf("1 to open right shutter\n");
	   printf("2 to close left shutter\n");
	   printf("3 to close right shutter\n");
	 //  printf("4 for emergency stop\n");
	 //  printf("5 to reset plc\n");
	   printf("6 to run hydrolic  \n");
	   printf("7 Arrange left shutter position\n");
	   printf("8 Arrange right shutter position \n");
	   printf("9 to stop hydrolic  \n");
	   printf("4 to stop All  \n");
	   printf("r to reset HeartBeat  \n");
	   printf("t to reset Alarms  \n");


	   fflush(stdin);

	   for( int x = 0; (x < 2) &&  ((ch = getchar()) != EOF)
	                        && (ch != '\n'); x++ )
		//printf("x is %d c is %d \n",x,c);

	    c=(char)ch;

		if (c == '0') {

			readControlRegister(mb,CONTROLWORD1,anyRegister);
			int err = openLeftShutter(anyRegister,mb, CONTROLWORD1);
			if(err ==-1){printf("error while opening left\n");break;}


		}

		if (c == '1') {
			readControlRegister(mb, CONTROLWORD1, anyRegister);
			int err = openRightShutter(anyRegister, mb, CONTROLWORD1);
			if (err == -1) {
				printf("error while opening right\n");
				break;
			}


		}
		if (c == '2') {
			readControlRegister(mb, CONTROLWORD1, anyRegister);
			int err = closeLeftShutter(anyRegister, mb, CONTROLWORD1);
			if (err == -1) {
				printf("error while closing left\n");
				break;
			}


		}
		if (c == '3') {
			readControlRegister(mb, CONTROLWORD1, anyRegister);
			int err = closeRightShutter(anyRegister, mb, CONTROLWORD1);
			if (err == -1) {
				printf("error while closing left\n");
				break;
			}


		}

		if (c == '5') {
			readControlRegister(mb, CONTROLWORD1, anyRegister);
			int err = resetButton(anyRegister, mb, CONTROLWORD1);
			if (err == -1) {
				printf("error while reset PLC\n");
				break;
			} else {

				printf("Reset PLC is pressed\n");
			}

		}
		if (c == '6') {
			readControlRegister(mb, CONTROLWORD1, anyRegister);
			int err = runHydrolic(anyRegister, mb, CONTROLWORD1);
			if (err == -1) {
				printf("error while running hydrolic \n");
				break;
			} else {

				printf("Hydrolic started\n");
			}

		}

		if (c == '7')
			{
			int position;
			std::cout << "enter the position" << std::endl;
			std::cin >> position;
			int err = setLeftDomePosition(position, mb,LEFTDOMEPOSITIONCOMMAND);
			if (err == -1) {
				printf("Error while set left dome position \n");
				break;
			} else {

					}

			readControlRegister(mb,CONTROLWORD1,anyRegister);
			err = goLeftShutter(anyRegister,mb, CONTROLWORD1);
			if(err ==-1){printf("error while goto left\n");break;}



				}
		if (c == '8') {
			int position;
			std::cout << "enter the position:" << std::endl;
			std::cin >> position;
			int err = setRightDomePosition(position, mb,RIGHTDOMEPOSITIONCOMMAND);
			if (err == -1) {
				printf("error while setting right shutter position\n");
				break;
			}





			readControlRegister(mb,CONTROLWORD1,anyRegister);
			err = goRightShutter(anyRegister,mb, CONTROLWORD1);
			if(err ==-1){printf("error while opening left\n");break;}


				}

		if (c == '9') {
			readControlRegister(mb, CONTROLWORD1, anyRegister);
			int err = stopHydrolic(anyRegister, mb, CONTROLWORD1);
			if (err == -1) {
				printf("error while stopping hydrolic \n");
				break;
			} else {

				printf("Hydrolic stopped\n");
			}

				}

		if (c == '4')
		{
			readControlRegister(mb, CONTROLWORD1, anyRegister);
			int err = stopAll(anyRegister, mb, CONTROLWORD1);
			if (err == -1) {
				printf("error while stopping All \n");
				break;
			} else

			{

				printf("All stopped\n");
			}
		}
			if (c == 'r')
			{



				/*thread kullanmak icin*/

				std::thread t([]()
						{
					while(TRUE)

					{

						uint16_t *anyRegister2 =  (uint16_t*) malloc(sizeof(uint16_t));
						memset(anyRegister2, 0, sizeof(uint16_t));
						std::cout<<"thread started"<<std::endl;
						readControlRegister(mb, CONTROLWORD1, anyRegister2);
						int err = setHearBeat2(anyRegister2, mb, CONTROLWORD1);
						if (err == -1) {
							printf("error while resetting heartbeat\n");


							//break;
						}
						else
						{
							printf("Resetted heartbeat\n");
						}

						readControlRegister(mb, CONTROLWORD1, anyRegister2);
						err = resetHearBeat2(anyRegister2, mb, CONTROLWORD1);

						if (err == -1) {
							printf("error while resetting heartbeat\n");


							//break;
						}
						else
						{
							printf("Resetted heartbeat\n");
						}

						sleep(5);

					}
						}
				);
				std::cout<<"main thread"<<std::endl;
				t.detach();
				/*thread kullanmak icin*/

		}

			if (c == 't')
			{
				readControlRegister(mb, CONTROLWORD1, anyRegister);
				int err = resetButton(anyRegister, mb, CONTROLWORD1);
				if (err == -1) {
					printf("reset alarm butonu \n");
					break;
				} else

				{

					printf("reset alarm butonu\n");
				}
			}


	  }


            modbus_close(mb);
	        modbus_free(mb);

	return 0;
}
