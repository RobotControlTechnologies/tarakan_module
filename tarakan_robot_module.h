#ifndef TARAKAN_ROBOT_MODULE_H
#define	TARAKAN_ROBOT_MODULE_H

typedef std::vector< std::pair< int, int > > universalVec;

class TarakanRobot : public Robot {
	protected:
		bool is_aviable;
		bool is_locked;

#ifdef _WIN32
	SOCKET s;
#else
	int s;
#endif

		std::vector<variable_value> axis_state;
		std::string connection;
		std::string calibration;
	public:
		universalVec vec_rotate, vec_move;

		std::string TarakanRobot::sendAndRecv(std::string command_for_robot);
		long int getParametrsToTime(variable_value parametr, universalVec *linkOfaddressMemVec);

		TarakanRobot(std::string connection, std::string calibration, universalVec vec_rotate, universalVec vec_move);

		bool require();
		void free();

		FunctionResult* executeFunction(system_value command_index, void **args);
		void axisControl(system_value axis_index, variable_value value);

		~TarakanRobot() {}
};
typedef std::vector<TarakanRobot*> m_connections;
typedef m_connections::iterator m_connections_i;

class TarakanRobotModule : public RobotModule {
#ifdef _WIN32
	CRITICAL_SECTION TRM_cs;
#else
	pthread_mutex_t TRM_mx;
#endif
	
	m_connections aviable_connections;
	FunctionData **robot_functions;
	AxisData **robot_axis;
	colorPrintf_t *colorPrintf;

	public:
		TarakanRobotModule();
		//init
		const char *getUID();
		void prepare(colorPrintf_t *colorPrintf_p, colorPrintfVA_t *colorPrintfVA_p);

		//compiler only
		FunctionData** getFunctions(unsigned int *count_functions);
		AxisData** getAxis(unsigned int *count_axis);
		void *writePC(unsigned int *buffer_length);

		//intepreter - devices
		int init();
		Robot* robotRequire();
		void robotFree(Robot *robot);
		void final();

		//intepreter - program & lib
        void readPC(void *buffer, unsigned int buffer_length);
        
        //intepreter - program
        int startProgram(int uniq_index);
		int endProgram(int uniq_index);

		//destructor
		void destroy();
		~TarakanRobotModule(){};
};

#define ADD_ROBOT_AXIS(AXIS_NAME, UPPER_VALUE, LOWER_VALUE) \
	robot_axis[axis_id] = new AxisData; \
	robot_axis[axis_id]->axis_index = axis_id + 1; \
	robot_axis[axis_id]->upper_value = UPPER_VALUE; \
	robot_axis[axis_id]->lower_value = LOWER_VALUE; \
	robot_axis[axis_id]->name = AXIS_NAME; \
	axis_id++;

#endif	/* TARAKAN_ROBOT_MODULE_H */
