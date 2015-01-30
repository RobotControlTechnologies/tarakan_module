#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <winsock2.h>
#include <ws2bth.h>
#include <map>
#include <vector>

#include "../module_headers/module.h"
#include "../module_headers/robot_module.h"

#include "tarakan_robot_module.h"
#include "SimpleIni.h"

#pragma comment(lib, "Ws2_32.lib")

typedef std::vector< std::pair< int, int > > universalVec;
typedef CSimpleIniA::TNamesDepend::const_iterator ini_i;

#define DEFAULT_SOCKET_BUFLEN 512
#define PATH_TO_CONFIG "robot_modules/tarakan/config.ini"

/* GLOBALS CONFIG */
const int COUNT_FUNCTIONS = 6;
const int COUNT_AXIS = 3;

#define DEFINE_ALL_FUNCTIONS \
	ADD_ROBOT_FUNCTION("moveTo", 2, true)					/*direction, distanse*/\
	ADD_ROBOT_FUNCTION("rotateTo", 2, false)				/*direction, angle*/\
	ADD_ROBOT_FUNCTION("moveToByTime", 2, true)				/*direction, time*/\
	ADD_ROBOT_FUNCTION("rotateToByTime", 2, false)			/*direction, time*/\
	ADD_ROBOT_FUNCTION("changeLightMode", 3, false)			/*type, strength, period*/\
	ADD_ROBOT_FUNCTION("getDistanceObstacle", 1, false)		/*direction*/

#define DEFINE_ALL_AXIS \
	ADD_ROBOT_AXIS("locked", 1, 0)\
	ADD_ROBOT_AXIS("straight", 2, 0)\
	ADD_ROBOT_AXIS("rotation", 2, 0)

universalVec vec_rotate, vec_move;

bool pred(const std::pair<int, int> &a, const std::pair<int, int> &b) {
	return a.first < b.first;
}

long int getParametrsToTime(int parametr, bool what){
	universalVec::const_iterator iter_key;
	universalVec* linkOfaddressMemVec;
	if (what){ // 1 поворот
		linkOfaddressMemVec = &vec_rotate;
	}else{ // 0 движение
		linkOfaddressMemVec = &vec_move;
	}
	int min = 0, max = 0, minValue = 0, maxValue = 0, count = 0;
	long double y = 0;
	bool flagEnd = false;
		for (iter_key = linkOfaddressMemVec->begin(); iter_key != linkOfaddressMemVec->end(); ++iter_key){
			if (parametr >= iter_key->first){
				min = iter_key->first;
				count = iter_key->second;
			}
			else{
				max = iter_key->first;
				maxValue = iter_key->second;
				minValue = count;
				flagEnd = true;
				break;
			}
			count++;
		}
		if (!flagEnd){
			--iter_key;
			max = iter_key->first;
			maxValue = iter_key->second;
			--iter_key;
			min = iter_key->first;
			minValue = iter_key->second;
		}

	y = (((maxValue - minValue)*(parametr - min)) / (max - min)) + minValue;

	return long(std::rint(y));
}

/////////////////////////////////////////////////

TarakanRobotModule::TarakanRobotModule() {
	{
		robot_functions = new FunctionData*[COUNT_FUNCTIONS];
		regval function_id = 0;
		DEFINE_ALL_FUNCTIONS
	}
	{
		robot_axis = new AxisData*[COUNT_AXIS];
		regval axis_id = 0;
		DEFINE_ALL_AXIS
	}
}

int TarakanRobotModule::init() {
	srand(time(NULL));

	CSimpleIniA ini;
	ini.SetMultiKey(true);
	if (ini.LoadFile(PATH_TO_CONFIG) < 0) {
		printf("Can't load '%s' file!\n", PATH_TO_CONFIG);
		return 1;
	}

	CSimpleIniA::TNamesDepend keys;

	vec_rotate.clear();
	vec_move.clear();
		
	ini.GetAllKeys("rotateTo", keys);
	for (ini_i ini_key = keys.begin(); ini_key != keys.end(); ++ini_key) {
		vec_rotate.push_back(
			std::make_pair(
				std::stoi(ini_key->pItem, nullptr, 10), 
				std::stoi(ini.GetValue("rotateTo", ini_key->pItem, NULL), nullptr, 10)
			)
		);
	}
	keys.clear();

	ini.GetAllKeys("moveTo", keys);
	for (ini_i ini_key = keys.begin(); ini_key != keys.end(); ++ini_key) {
		vec_move.push_back(
			std::make_pair(
				std::stoi(ini_key->pItem), 
				std::stoi(ini.GetValue("moveTo", ini_key->pItem, NULL))
			)
		);
	}
	
	//сортируем этот вектор по значению
	std::sort(vec_rotate.begin(), vec_rotate.end(), pred);
	std::sort(vec_move.begin(), vec_move.end(), pred);

	CSimpleIniA::TNamesDepend values;
	ini.GetAllValues("connections", "connection", values);

	WSADATA wsd;
	SOCKET s;
	SOCKADDR_BTH sab;
	
	char recvbuf[DEFAULT_SOCKET_BUFLEN] = "";

	if (WSAStartup(MAKEWORD(2, 2), &wsd) != 0) {
		printf("Unable to load Winsock! Error code is %d\n", WSAGetLastError());
		return 1;
	} else {
		printf("WSAStartup() is OK, Winsock lib loaded!\n");
	}

	for (ini_i ini_value = values.begin(); ini_value != values.end(); ++ini_value) {
		std::string connection(ini_value->pItem);

		try {
			s = socket(AF_BTH, SOCK_STREAM, BTHPROTO_RFCOMM);
			
			if (s == INVALID_SOCKET) {
				printf("Socket creation failed, error %d\n", WSAGetLastError());
			} else{
				printf("socket() looks fine!\n");
			}

			memset(&sab, 0, sizeof(sab));
			sab.addressFamily = AF_BTH;
			sab.btAddr = std::stoll(connection.c_str(), nullptr, 16);
			sab.serviceClassId = SerialPortServiceClass_UUID;
			sab.port = BT_PORT_ANY;

			if (connect(s, (SOCKADDR *)&sab, sizeof(sab)) == SOCKET_ERROR) {
				//This is magic
				if (connect(s, (SOCKADDR *)&sab, sizeof(sab)) == SOCKET_ERROR) {
					printf("connect() failed with error code %d\n", WSAGetLastError());
					closesocket(s);
					throw std::exception();
				}
			}

			printf("connect() should be fine!\n");
			
			TarakanRobot *tarakan_robot = new TarakanRobot(s);
			printf("DLL: connected to %s robot %p\n", connection.c_str(), tarakan_robot);
			aviable_connections.push_back(tarakan_robot);
		} catch (...) {
			printf("Cannot connect to robot with connection: %s\n", connection.c_str());
		}
	}

	return 0;
}

FunctionData** TarakanRobotModule::getFunctions(int *count_functions) {
	(*count_functions) = COUNT_FUNCTIONS;
	return robot_functions;
}

AxisData** TarakanRobotModule::getAxis(int *count_axis) {
	(*count_axis) = COUNT_AXIS;
	return robot_axis;
}

Robot* TarakanRobotModule::robotRequire() {
	printf("DLL: new robot require\n");

	int count_robots = aviable_connections.size();
	if (!count_robots){
		return NULL;
	}

	int index = rand() % count_robots;
	int j = 0;
	for (m_connections_i i = aviable_connections.begin(); i != aviable_connections.end(); ++i) {
		if ((*i)->is_aviable) {
			if (j == index) {
				(*i)->is_aviable = false;
				return (*i);
			}
			++j;
		}
	}

	return NULL;
}

void TarakanRobotModule::robotFree(Robot *robot) {
	TarakanRobot *tarakan_robot = reinterpret_cast<TarakanRobot*>(robot);

	for (m_connections_i i = aviable_connections.begin(); i != aviable_connections.end(); ++i) {
		if ((*i) == tarakan_robot) {
			printf("DLL: free robot: %p\n", tarakan_robot);
			tarakan_robot->is_aviable = true;
			return;
		}
	}
}

void TarakanRobotModule::final() {
	for (m_connections_i i = aviable_connections.begin(); i != aviable_connections.end(); ++i) {
		closesocket((*i)->getSocket());
		delete (*i);
	}
	aviable_connections.clear();

	vec_rotate.clear();
	vec_move.clear();
	
	WSACleanup();
}

void TarakanRobotModule::destroy() {
	for (int j = 0; j < COUNT_FUNCTIONS; ++j) {
		delete robot_functions[j];
	}
	for (int j = 0; j < COUNT_AXIS; ++j) {
		delete robot_axis[j];
	}
	delete[] robot_functions;
	delete[] robot_axis;
	delete this;
}

TarakanRobot::TarakanRobot(SOCKET socket) 
	 : is_aviable(true), is_locked(true), socket(socket) {
	for(regval i = 0; i < COUNT_AXIS; ++i) {
		axis_state.push_back(1);
	}
}

FunctionResult* TarakanRobot::executeFunction(regval command_index, regval *args) {
	if (!command_index) {
		return NULL;
	}

	FunctionResult *fr;
	try {
		std::string command_for_robot = "";
		bool need_result = false;

		if (
			(command_index != ROBOT_COMMAND_HAND_CONTROL_BEGIN)
			&& (command_index != ROBOT_COMMAND_HAND_CONTROL_END)
		) {
			if ((args[0] != 0) && (args[0] != 1)) {
				throw;
			}

			if (
				(command_index >= 1)
				&& (command_index <= 4)
			){
				if (args[1] < 0) {
					throw;
				}
			}

			command_for_robot += std::to_string(command_index);
			command_for_robot += args[0] ? "0" : "1";
		}

		switch (command_index) {
			case ROBOT_COMMAND_HAND_CONTROL_BEGIN: 
				command_for_robot += "B";
				break;
			case ROBOT_COMMAND_HAND_CONTROL_END:
				command_for_robot += "E";
				break;
			case 1:	// moveTo
			case 2: // rotateTo
				command_for_robot += std::to_string(getParametrsToTime(args[1], 0) * 1000);
				break;
			case 3:	// moveToByTime
			case 4: // rotateToByTime
				command_for_robot += std::to_string(args[1]);
				break;
			case 5: { //changeLightMode
				if ((args[1] < 0)||(args[1] > 100)) {
					throw;
				}
				if (args[2] < 0) {
					throw;
				}

				if (
					(args[1] >= 10)
					|| (args[1] < 100)
				) { 
					command_for_robot.append(std::to_string(0)); 
				} else if (args[1] < 10) { 
					command_for_robot.append(std::to_string(00)); 
				}

				command_for_robot.append(std::to_string(args[1]));
				command_for_robot.append(std::to_string(args[2]));
				break;
			}
			case 6: //getDistanceObstacle
				need_result = true;
			default: 
				break;
		}

		command_for_robot += "&";

		fd_set readset;
		FD_ZERO(&readset);
		FD_SET(socket, &readset);

		send(socket, command_for_robot.c_str(), command_for_robot.length(), 0);

		int result = select(socket, &readset, NULL, NULL, NULL);
		if (result < 0 && errno != EINTR){
			printf("Error in select(): %s\n", strerror(errno));
			throw;
		}
		
		char recvbuf[DEFAULT_SOCKET_BUFLEN] = "";
		std::string recvstr = "";
		while (recvstr.find('&') == std::string::npos) {
			int count_recived = recv(socket, recvbuf, DEFAULT_SOCKET_BUFLEN - 1, 0);
			recvbuf[count_recived] = 0;
			recvstr.append(recvbuf);
		}

		if (recvstr[0] != '0') {
			throw;
		}

		fr = new FunctionResult(1, 0);

		if (need_result) {
			recvstr.erase(0, 1);
			recvstr.erase(recvstr.find('&'), 1);
			fr->result = std::stoi(recvstr.c_str());
		}
	} catch (...) {
		fr = new FunctionResult(0);
	}

	return fr;
}

void TarakanRobot::axisControl(regval axis_index, regval value) {
	bool need_send = false;
	
	if (axis_index == 1) {
		if (
			((is_locked) && (!value))
			||((!is_locked) && (value))
		) {
			is_locked = (bool) value;
			need_send = true;
		}
	} else {
		need_send = (!is_locked) && (axis_state[axis_index - 1] != value);
	}

	if (need_send) {
		axis_state[axis_index - 1] = value;
		std::string command_for_robot = "H";
		command_for_robot += std::to_string(axis_index);
		command_for_robot += std::to_string(value);
		command_for_robot += "&";
		send(socket, command_for_robot.c_str(), command_for_robot.length(), 0);
		printf("%s\n",command_for_robot.c_str());
	}
}

SOCKET TarakanRobot::getSocket() {
	return socket;
}

__declspec(dllexport) RobotModule* getRobotModuleObject() {
	return new TarakanRobotModule();
}
