#include <winsock2.h>
#ifndef TARAKAN_H
#define	TARAKAN_H

typedef std::map<std::string, FunctionData*> function_list;

class TarakanRobot : public Robot {
public:
	bool isAviable;
	SOCKET socket_tarakan;
	int robot_index;
	HANDLE hSerial;
	TarakanRobot(SOCKET tarakan) : isAviable(true) {
		socket_tarakan = tarakan;
	}
	SOCKET getSocket_tarakan(){
		return socket_tarakan;
	}
	virtual regval executeFunction(regval command_index, regval *args);
	~TarakanRobot() {}
};
typedef std::map<int, TarakanRobot*> m_connections;

class TarakanRobotModule : public RobotModule {
	m_connections aviable_connections;
	function_list robot_functions;

public:
	TarakanRobotModule();
	virtual int init();
	virtual FunctionData* checkAviableFunction(const char *function_name);
	virtual Robot* robotRequire();
	virtual void robotFree(Robot *robot);
	virtual void final();
	virtual void destroy();
	~TarakanRobotModule(){};
};

extern "C" {
	__declspec(dllexport) RobotModule* getRobotModuleObject();
}

#define ADD_ROBOT_FUNCTION(FUNCTION_NAME,COUNT_PARAMS) \
	function = new FunctionData(function_id, COUNT_PARAMS); \
	function_id++; \
	robot_functions[FUNCTION_NAME] = function;

#endif	/* TARAKAN_H */