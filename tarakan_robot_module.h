#ifndef TARAKAN_ROBOT_MODULE_H
#define TARAKAN_ROBOT_MODULE_H

#define ROBOT_COMMAND_FREE 0
#define ROBOT_COMMAND_HAND_CONTROL_BEGIN -1
#define ROBOT_COMMAND_HAND_CONTROL_END -2

typedef std::vector<std::pair<int, int> > universalVec;

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

  char *uniq_name;
  colorPrintfRobotVA_t *colorPrintf_p;

 public:
  universalVec vec_rotate, vec_move;

  std::string sendAndRecv(std::string command_for_robot);
  long int getParametrsToTime(variable_value parametr,
                              universalVec *linkOfaddressMemVec);

  TarakanRobot(std::string connection, std::string calibration,
               universalVec vec_rotate, universalVec vec_move,
               unsigned int uniq_index);
  void prepare(colorPrintfRobot_t *colorPrintf_p,
               colorPrintfRobotVA_t *colorPrintfVA_p);
  bool require();
  void free();

  FunctionResult *executeFunction(CommandMode mode, system_value command_index,
                                  void **args);
  void axisControl(system_value axis_index, variable_value value);

  ~TarakanRobot();

  void colorPrintf(ConsoleColor colors, const char *mask, ...);
};
typedef std::vector<TarakanRobot *> m_connections;
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
  colorPrintfModuleVA_t *colorPrintf_p;

 public:
  TarakanRobotModule();
  // init
  const char *getUID();
  void prepare(colorPrintfModule_t *colorPrintf_p,
               colorPrintfModuleVA_t *colorPrintfVA_p);

  // compiler only
  FunctionData **getFunctions(unsigned int *count_functions);
  AxisData **getAxis(unsigned int *count_axis);
  void *writePC(unsigned int *buffer_length);

  // intepreter - devices
  int init();
  Robot *robotRequire();
  void robotFree(Robot *robot);
  void final();

  // intepreter - program & lib
  void readPC(void *buffer, unsigned int buffer_length);

  // intepreter - program
  int startProgram(int uniq_index);
  int endProgram(int uniq_index);

  // destructor
  void destroy();
  ~TarakanRobotModule(){};

  void colorPrintf(ConsoleColor colors, const char *mask, ...);
};

#define ADD_ROBOT_AXIS(AXIS_NAME, UPPER_VALUE, LOWER_VALUE) \
  robot_axis[axis_id] = new AxisData;                       \
  robot_axis[axis_id]->axis_index = axis_id + 1;            \
  robot_axis[axis_id]->upper_value = UPPER_VALUE;           \
  robot_axis[axis_id]->lower_value = LOWER_VALUE;           \
  robot_axis[axis_id]->name = AXIS_NAME;                    \
  axis_id++;

#endif /* TARAKAN_ROBOT_MODULE_H */
