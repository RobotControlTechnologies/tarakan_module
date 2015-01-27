#ifndef TARAKAN_ROBOT_MODULE_H
#define	TARAKAN_ROBOT_MODULE_H

class TarakanRobot : public Robot {
	public:
		bool is_aviable;
		bool is_locked;
		SOCKET socket;

		std::vector<regval> axis_state;
	
		TarakanRobot(SOCKET socket);
		FunctionResult* executeFunction(regval command_index, regval *args);
		void axisControl(regval axis_index, regval value);

		SOCKET getSocket();
		~TarakanRobot() {}
};
typedef std::vector<TarakanRobot*> m_connections;
typedef m_connections::iterator m_connections_i;

class TarakanRobotModule : public RobotModule {
	m_connections aviable_connections;
	FunctionData *robot_functions;
	AxisData *robot_axis;

	public:
		TarakanRobotModule();
		int init();
		FunctionData* getFunctions(int *count_functions);
		AxisData* getAxis(int *count_axis);
		Robot* robotRequire();
		void robotFree(Robot *robot);
		void final();
		void destroy();
		~TarakanRobotModule(){};
};

#define ADD_ROBOT_FUNCTION(FUNCTION_NAME, COUNT_PARAMS, GIVE_EXCEPTION) \
	robot_functions[function_id].command_index = function_id + 1; \
	robot_functions[function_id].count_params = COUNT_PARAMS; \
	robot_functions[function_id].give_exception = GIVE_EXCEPTION; \
	robot_functions[function_id].name = FUNCTION_NAME; \
	function_id++;

#define ADD_ROBOT_AXIS(AXIS_NAME, UPPER_VALUE, LOWER_VALUE) \
	robot_axis[axis_id].axis_index = axis_id + 1; \
	robot_axis[axis_id].upper_value = UPPER_VALUE; \
	robot_axis[axis_id].lower_value = LOWER_VALUE; \
	robot_axis[axis_id].name = AXIS_NAME; \
	axis_id++;

#endif	/* TARAKAN_ROBOT_MODULE_H */