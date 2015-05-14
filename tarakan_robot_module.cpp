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

EXTERN_C IMAGE_DOS_HEADER __ImageBase;

/* GLOBALS CONFIG */
const unsigned int COUNT_FUNCTIONS = 6;
const unsigned int COUNT_AXIS = 3;

#define DEFINE_ALL_AXIS \
	ADD_ROBOT_AXIS("locked", 1, 0)\
	ADD_ROBOT_AXIS("straight", 2, 0)\
	ADD_ROBOT_AXIS("rotation", 2, 0)

universalVec vec_rotate, vec_move;

bool pred(const std::pair<int, int> &a, const std::pair<int, int> &b) {
	return a.first < b.first;
}

long int getParametrsToTime(variable_value parametr, bool what){
	universalVec::const_iterator iter_key;
	universalVec* linkOfaddressMemVec;
	if (what){ 
		linkOfaddressMemVec = &vec_rotate;
	}else{ 
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
		system_value function_id = 0;

		//DEFINE_ALL_FUNCTIONS
		FunctionData::ParamTypes *Params = new FunctionData::ParamTypes[2];
		Params[0] = FunctionData::FLOAT;
		Params[1] = FunctionData::FLOAT;
		robot_functions[function_id] = new FunctionData(function_id + 1, 2, Params, "moveTo");
		function_id++;


		Params = new FunctionData::ParamTypes[2];
		Params[0] = FunctionData::FLOAT;
		Params[1] = FunctionData::FLOAT;
		robot_functions[function_id] = new FunctionData(function_id + 1, 2, Params, "rotateTo");
		function_id++;


		Params= new FunctionData::ParamTypes[2];
		Params[0] = FunctionData::FLOAT;
		Params[1] = FunctionData::FLOAT;
		robot_functions[function_id] = new FunctionData(function_id + 1, 2, Params, "moveToByTime");
		function_id++;


		Params = new FunctionData::ParamTypes[2];
		Params[0] = FunctionData::FLOAT;
		Params[1] = FunctionData::FLOAT;
		robot_functions[function_id] = new FunctionData(function_id + 1, 2, Params, "rotateToByTime");
		function_id++;

		
		Params = new FunctionData::ParamTypes[3];
		Params[0] = FunctionData::FLOAT;
		Params[1] = FunctionData::FLOAT;
		Params[2] = FunctionData::FLOAT;
		robot_functions[function_id] = new FunctionData(function_id + 1, 3, Params, "changeLightMode");
		function_id++;


		FunctionData::ParamTypes *getDistanceObstacleParams = new FunctionData::ParamTypes[1];
		getDistanceObstacleParams[0] = FunctionData::FLOAT;
		robot_functions[function_id] = new FunctionData(function_id + 1, 1, getDistanceObstacleParams, "getDistanceObstacle");

	}
	{
		robot_axis = new AxisData*[COUNT_AXIS];
		system_value axis_id = 0;
		DEFINE_ALL_AXIS
	}
}

const char *TarakanRobotModule::getUID() {
	return "Tarakan robot module 1.00 for presentaion";
}

void TarakanRobotModule::prepare(colorPrintf_t *colorPrintf_p, colorPrintfVA_t *colorPrintfVA_p) {
	colorPrintf = colorPrintf_p;
}

void *TarakanRobotModule::writePC(unsigned int *buffer_length) {
	*buffer_length = 0;
	return NULL;
}

int TarakanRobotModule::init() {
	WSADATA wsd;
	if (WSAStartup(MAKEWORD(2, 2), &wsd) != 0) {
		(*colorPrintf)(this, ConsoleColor(ConsoleColor::red), "Unable to load Winsock! Error code is %d\n", WSAGetLastError());
		return 1;
	}
	
	(*colorPrintf)(this, ConsoleColor(ConsoleColor::green), "WSAStartup() is OK, Winsock lib loaded!\n");

	srand(time(NULL));
	InitializeCriticalSection(&TRM_cs);

	WCHAR DllPath[MAX_PATH] = {0};
	GetModuleFileNameW((HINSTANCE)&__ImageBase, DllPath, _countof(DllPath));

	WCHAR *tmp = wcsrchr(DllPath, L'\\');
	WCHAR ConfigPath[MAX_PATH] = {0};
	
	size_t path_len = tmp - DllPath;

	wcsncpy(ConfigPath, DllPath, path_len);
	wcscat(ConfigPath, L"\\config.ini");

	CSimpleIniA ini;
	ini.SetMultiKey(true);
	if (ini.LoadFile(ConfigPath) < 0) {
		(*colorPrintf)(this, ConsoleColor(ConsoleColor::red), "Can't load '%s' file!\n", ConfigPath);
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
	
	std::sort(vec_rotate.begin(), vec_rotate.end(), pred);
	std::sort(vec_move.begin(), vec_move.end(), pred);

	CSimpleIniA::TNamesDepend values;
	ini.GetAllValues("connections", "connection", values);

	for (ini_i ini_value = values.begin(); ini_value != values.end(); ++ini_value) {
		std::string connection(ini_value->pItem);
		TarakanRobot *tarakan_robot = new TarakanRobot(connection);
		aviable_connections.push_back(tarakan_robot);
	}

	return 0;
}

FunctionData** TarakanRobotModule::getFunctions(unsigned int *count_functions) {
	(*count_functions) = COUNT_FUNCTIONS;
	return robot_functions;
}

AxisData** TarakanRobotModule::getAxis(unsigned int *count_axis) {
	(*count_axis) = COUNT_AXIS;
	return robot_axis;
}

Robot* TarakanRobotModule::robotRequire() {
	EnterCriticalSection(&TRM_cs);
	for (auto i = aviable_connections.begin(); i != aviable_connections.end(); ++i) {
		if ((*i)->require()) {
			LeaveCriticalSection(&TRM_cs);
			return (*i);
		}
	}
	LeaveCriticalSection(&TRM_cs);
	return NULL;
}

void TarakanRobotModule::robotFree(Robot *robot) {
	TarakanRobot *tarakan_robot = reinterpret_cast<TarakanRobot*>(robot);

	EnterCriticalSection(&TRM_cs);
	for (m_connections_i i = aviable_connections.begin(); i != aviable_connections.end(); ++i) {
		if ((*i) == tarakan_robot) {
			tarakan_robot->free();
			LeaveCriticalSection(&TRM_cs);
			return;
		}
	}
	LeaveCriticalSection(&TRM_cs);
}

void TarakanRobotModule::final() {
	for (m_connections_i i = aviable_connections.begin(); i != aviable_connections.end(); ++i) {
		delete (*i);
	}
	aviable_connections.clear();

	vec_rotate.clear();
	vec_move.clear();
	
	WSACleanup();
}

void TarakanRobotModule::destroy() {
	for (unsigned int j = 0; j < COUNT_FUNCTIONS; ++j) {
		delete robot_functions[j];
	}
	for (unsigned int j = 0; j < COUNT_AXIS; ++j) {
		delete robot_axis[j];
	}
	delete[] robot_functions;
	delete[] robot_axis;
	delete this;
}

TarakanRobot::TarakanRobot(std::string connection) :
	is_aviable(true), is_locked(true), connection(connection) {
	for (unsigned int i = 0; i < COUNT_AXIS; ++i) {
		axis_state.push_back(1);
	}
}

bool TarakanRobot::require() {
	if (!is_aviable) {
		return false;
	}

	s = socket(AF_BTH, SOCK_STREAM, BTHPROTO_RFCOMM);

	printf("Try connect with: %s\n", connection.c_str());

	if (s == INVALID_SOCKET) {
		printf("Socket creation failed, error %d\n", WSAGetLastError());
		return false;
	}

	printf("socket() looks fine!\n");

	SOCKADDR_BTH sab;

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
			return false;
		}
	}

	printf("Connected to %s robo\n", connection.c_str());
	is_aviable = false;

	return true;
}

void TarakanRobot::free() {
	if (is_aviable) {
		return;
	}
	is_aviable = true;
	closesocket(s);
}

FunctionResult* TarakanRobot::executeFunction(system_value command_index, void **args) {
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
			variable_value *input1 = (variable_value *)(*args);
			if ((*input1 != 0) && (*input1 != 1)) {
				throw std::exception();
			}

			if (
				(command_index >= 1)
				&& (command_index <= 4)
			){
				variable_value *input2 = (variable_value *)(*(args+1));
				if (*input2 < 0) {
					throw std::exception();
				}
			}
			command_for_robot += std::to_string(command_index);
			command_for_robot += *input1 ? "0" : "1";
		}

		switch (command_index) {
			case ROBOT_COMMAND_HAND_CONTROL_BEGIN: 
				command_for_robot += "B";
				break;
			case ROBOT_COMMAND_HAND_CONTROL_END:
				command_for_robot += "E";
				break;
			case 1:	{// moveTo
				variable_value *input2 = (variable_value *)(*(args + 1));
				command_for_robot += std::to_string(getParametrsToTime(*input2, false) * 1000);
				break;
			}
			case 2: {// rotateTo
				variable_value *input2 = (variable_value *)(*(args + 1));
				command_for_robot += std::to_string(getParametrsToTime(*input2, true) * 1000);
				break;
			}
			case 3:	// moveToByTime
			case 4: {// rotateToByTime
				variable_value *input2 = (variable_value *)(*(args + 1));
				command_for_robot += std::to_string(*input2);
				break;
			}
			case 5: { //changeLightMode
				variable_value *input2 = (variable_value *)(*(args + 1));
				variable_value *input3 = (variable_value *)(*(args + 2));
				if ((*input2 < 0) || (*input2 > 100)) {
					throw std::exception();
				}
				if (*input3 < 0) {
					throw std::exception();
				}

				if (
					(*input2 >= 10)
					|| (*input2 < 100)
				) { 
					command_for_robot.append(std::to_string(0)); 
				}
				else if (*input2 < 10) {
					command_for_robot.append(std::to_string(00)); 
				}

				command_for_robot.append(std::to_string(*input2));
				command_for_robot.append(std::to_string(*input3));
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
		FD_SET(s, &readset);

		send(s, command_for_robot.c_str(), command_for_robot.length(), 0);

		int result = select(s, &readset, NULL, NULL, NULL);
		if (result < 0 && errno != EINTR){
			printf("Error in select(): %s\n", strerror(errno));
			throw std::exception();
		}
		
		char recvbuf[DEFAULT_SOCKET_BUFLEN] = "";
		std::string recvstr = "";
		while (recvstr.find('&') == std::string::npos) {
			int count_recived = recv(s, recvbuf, DEFAULT_SOCKET_BUFLEN - 1, 0);
			recvbuf[count_recived] = 0;
			recvstr.append(recvbuf);
		}

		if (recvstr[0] != '0') {
			throw std::exception();
		}

		fr = new FunctionResult(1);

		if (need_result) {
			recvstr.erase(0, 1);
			recvstr.erase(recvstr.find('&'), 1);
			fr = new FunctionResult(1, std::stoi(recvstr.c_str()));
		} else {
			fr = new FunctionResult(1, 0);
		}
	} catch (...) {
		fr = new FunctionResult(0);
	}

	return fr;
}

void TarakanRobot::axisControl(system_value axis_index, variable_value value) {
	bool need_send = false;
	
	if (axis_index == 1) {
		if (
			((is_locked) && (!value))
			||((!is_locked) && (value))
		) {
			is_locked = !!value;
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
		send(s, command_for_robot.c_str(), command_for_robot.length(), 0);
		printf("%s\n",command_for_robot.c_str());
	}
}

int TarakanRobotModule::startProgram(int uniq_index, void *buffer, unsigned int buffer_length) {
	return 0;
}

int TarakanRobotModule::endProgram(int uniq_index) {
	return 0;
}

PREFIX_FUNC_DLL RobotModule* getRobotModuleObject() {
	return new TarakanRobotModule();
}
