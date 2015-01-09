#include <winsock2.h>
#include <ws2bth.h>
#include <map>
#include <vector>
#include "module.h"
#include "robot_module.h"
#include "tarakan_module.h"
#include "simpleini/SimpleIni.h"

#define DEFAULT_BUFLEN 512
#pragma comment(lib, "Ws2_32.lib")

char *PATH_TO_CONFIG = "robot_modules/tarakan/config.ini";
CSimpleIniA ini;
typedef ULONGLONG bt_addr, *pbt_addr, *PBT_ADDR;
typedef std::vector< std::pair< int, int > > universalVec;
universalVec vec_rotate, vec_move;

bool pred(const std::pair< int, int > &a, const std::pair< int, int > &b)
{
	return a.first < b.first;
}

long getParametrsToTime(int parametr, bool what){
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

int TarakanRobotModule::init() {

	ini.SetMultiKey(true);
	if (ini.LoadFile(PATH_TO_CONFIG) < 0) {
		printf("Can't load '%s' file!\n", PATH_TO_CONFIG);
		return 1;
	}

	std::pair<int, int> Point;
	CSimpleIniA::TNamesDepend keys;
	CSimpleIniA::TNamesDepend::const_iterator ini_key;
	
	ini.GetAllKeys("rotateTo", keys);
	for (ini_key = keys.begin(); ini_key != keys.end(); ++ini_key) {
		Point = std::make_pair(std::stoi(ini_key->pItem, nullptr, 10), std::stoi(ini.GetValue("rotateTo", ini_key->pItem, NULL /*default*/), nullptr, 10));
		vec_rotate.push_back(Point);
	}
	keys.clear();
	ini.GetAllKeys("moveTo", keys);
	for (ini_key = keys.begin(); ini_key != keys.end(); ++ini_key) {
		Point = std::make_pair(std::stoi(ini_key->pItem, nullptr, 10), std::stoi(ini.GetValue("moveTo", ini_key->pItem, NULL /*default*/), nullptr, 10));
		vec_move.push_back(Point);
	}
	
	//сортируем этот вектор по значению
	std::sort(vec_rotate.begin(), vec_rotate.end(), pred);
	std::sort(vec_move.begin(), vec_move.end(), pred);

	CSimpleIniA::TNamesDepend values;
	ini.GetAllValues("connections", "connection", values);

	CSimpleIniA::TNamesDepend::const_iterator ini_value;
	printf("Aviable connections for tarakan robots:\n");

	WSADATA wsd;
	SOCKET s;
	SOCKADDR_BTH sab;
	// Change the type accordingly for non-char data
	char recvbuf[DEFAULT_BUFLEN] = "";

	if (WSAStartup(MAKEWORD(2, 2), &wsd) != 0)
	{
		printf("Unable to load Winsock! Error code is %d\n", WSAGetLastError());
		return 1;
	}
	else
		printf("WSAStartup() is OK, Winsock lib loaded!\n");
	int icc = 0;
	for (ini_value = values.begin(); ini_value != values.end(); ++ini_value) {
		
		std::string connection(ini_value->pItem);
		try {
			s = socket(AF_BTH, SOCK_STREAM, BTHPROTO_RFCOMM);
			if (s == INVALID_SOCKET)
			{
				printf("Socket creation failed, error %d\n", WSAGetLastError());
			}
			else{
				printf("socket() looks fine!\n");
			}
			memset(&sab, 0, sizeof(sab));
			sab.addressFamily = AF_BTH;
			bt_addr BT = std::stoll(connection.c_str(), nullptr, 16);
			sab.btAddr = BT;
			sab.serviceClassId = SerialPortServiceClass_UUID;
			printf("%A\n", BT);
			sab.port = BT_PORT_ANY;
			if (connect(s, (SOCKADDR *)&sab, sizeof(sab)) == SOCKET_ERROR)
			{
				if (connect(s, (SOCKADDR *)&sab, sizeof(sab)) == SOCKET_ERROR)
				{
					printf("connect() failed with error code %d\n", WSAGetLastError());
					closesocket(s);
					throw std::exception();
				}
			}
					printf("connect() should be fine!\n");
					TarakanRobot *tarakan_robot = new TarakanRobot(s);
					printf("DLL: connected to %A robot %p\n", BT, tarakan_robot);
					aviable_connections[icc] = tarakan_robot;
					icc++;
				
	
		}
		catch (...) {
			printf("Cannot connect to robot with connection: %s\n", connection.c_str());
		}
	}
	return 0;
}

FunctionData* TarakanRobotModule::checkAviableFunction(const char *function_name) {
	for (function_list::iterator i = robot_functions.begin(); i != robot_functions.end(); i++) {
		if (!strcmp(i->first.c_str(), function_name)) {
			return i->second;
		}
	}
	return NULL;
}

Robot* TarakanRobotModule::robotRequire() {
	printf("DLL: new robot require\n");

	bool finded = false;
	int jj = 0;
	while (jj <= 10) {
		if (aviable_connections.size() > 0){
			int ind = rand() % aviable_connections.size();
			if (aviable_connections[ind]->isAviable) {
				printf("DLL: finded free robot: %p\n", aviable_connections[ind]);

				TarakanRobot *tarakan_robot = aviable_connections[ind];
				tarakan_robot->isAviable = false;

				Robot *robot = tarakan_robot;
				return robot;
			}
		}
		++jj;
	}
	return NULL;
}

void TarakanRobotModule::robotFree(Robot *robot) {
	TarakanRobot *tarakan_robot = reinterpret_cast<TarakanRobot*>(robot);

	for (m_connections::iterator i = aviable_connections.begin(); i != aviable_connections.end(); ++i) {
		if (i->second == tarakan_robot) {
			printf("DLL: free robot: %p\n", tarakan_robot);
			tarakan_robot->isAviable = true;
		}
	}
}

void TarakanRobotModule::final() {
	for (m_connections::iterator i = aviable_connections.begin(); i != aviable_connections.end(); ++i) {
		closesocket(i->second->getSocket_tarakan());
		delete i->second;
	}
	// чистка векторов
	vec_rotate.clear();
	vec_move.clear();
	// закрыть сокеты
	WSACleanup();
	aviable_connections.clear();
}

void TarakanRobotModule::destroy() {
	for (function_list::iterator i = robot_functions.begin(); i != robot_functions.end(); i++) {
		delete i->second;
	}
	robot_functions.clear();

delete this;
}

TarakanRobotModule::TarakanRobotModule() {
	regval function_id = 1;
	FunctionData *function;

	ADD_ROBOT_FUNCTION("moveTo", 2)
	ADD_ROBOT_FUNCTION("rotateTo", 2)
	ADD_ROBOT_FUNCTION("changeLighMode", 4)
	ADD_ROBOT_FUNCTION("getDistanceObstacle", 1)
}


regval TarakanRobot::executeFunction(regval command_index, regval *args) {
	std::string var = "";
	switch (command_index) {
	case 1: {// moveTo
			timeval time = { 0 };
			int result, iResult;
			char recvbuf[DEFAULT_BUFLEN] = "";
			fd_set readset;
			FD_ZERO(&readset);
			FD_SET(socket_tarakan, &readset);
			var = args[1] ? "W" : "S";
			time.tv_usec = getParametrsToTime(args[0], 0) * 1000;
			time.tv_sec = 0;
			send(socket_tarakan, var.c_str(), var.length(), 0);
			result = select(socket_tarakan, &readset, NULL, NULL, &time);
			if (!result){
				printf("select() time out\n");			
				send(socket_tarakan, "T", (int)strlen("T"), 0);
				//iResult = recv(socket_tarakan, recvbuf, DEFAULT_BUFLEN, 0);
			}
			else if (result < 0 && errno != EINTR){
				printf("Error in select(): %s\n", strerror(errno));
			}else {
				printf("select() bytes received\n");
				send(socket_tarakan, "T", (int)strlen("T"), 0);
				iResult = recv(socket_tarakan, recvbuf, DEFAULT_BUFLEN, 0);
			}
			return 1; 	
		}
		case 2: {//rotateTo
			var = args[1] ? "R" : "L";
			send(socket_tarakan, var.c_str(), var.length(), 0);
			Sleep(getParametrsToTime(args[0], 1));
			send(socket_tarakan, "T", (int)strlen("T"), 0);
			return 1;
		}
		case 3: {//changeLighMode
			if (!args[0]){
				switch (args[1]) {
					case 0: {
						var = "A";	break;
					}
					case 1: {
						var = "G";	break;
					}
					case 2: {
						var = "M";	break;
					}
				}
			}else {
				switch (args[1]) {
					case 0: {
						var = "Q";	break;
					}
					case 1: {
						var = "D";	break;
					}
					case 2: {
						var = "K";	break;
					}
				}
			}
			if (args[1]){
				if (args[2] >= 10)	{ 
					var.append(std::to_string(0)); 
				}else { 
					var.append(std::to_string(00)); 
				}
				var.append(std::to_string(args[2]));
				if (args[1] == 2){
					var.append(std::to_string(args[3]));
				}
				var.append("&");					
			}
			send(socket_tarakan, var.c_str(), var.length(), 0);
			return 1;
		}
		case 4: {//getDistanceObstacle
			var = args[1] ? "B" : "E";
			send(socket_tarakan, var.c_str(), var.length(), 0);
			bool flag=true;
			int iResult;
			char recvbuf[DEFAULT_BUFLEN] = "";
			do {
				iResult = recv(socket_tarakan, recvbuf, DEFAULT_BUFLEN, 0);
				if (iResult > 0){	
					recvbuf[iResult] = 0;
					for (int t = 0; t <= iResult; t++){
						if (recvbuf[t] == '&'){
							recvbuf[t] = 0;
							flag = false;
						}
				
					}
					printf(" %d Bytes received from sender\n", iResult);
				}
				else if (!iResult){
					printf("Connection was closed by peer!\n");
				}else {
					printf("recv() failed with error code %d\n", WSAGetLastError());
				}

			} while (flag);
			return std::atoi(recvbuf);
		}
		default: 
			return 0; 		
	}
}

__declspec(dllexport) RobotModule* getRobotModuleObject() {
	TarakanRobotModule *lrm = new TarakanRobotModule();
	RobotModule *rm = lrm;
	return rm;
}
