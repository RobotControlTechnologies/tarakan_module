#ifdef _WIN32
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#define _SCL_SECURE_NO_WARNINGS
#endif

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <time.h>
#include <map>
#include <vector>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2bth.h>

#pragma comment(lib, "Ws2_32.lib")
#else
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <dlfcn.h>
#include <pthread.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#endif

#include "SimpleIni.h"

#include "module.h"
#include "robot_module.h"

#include "tarakan_robot_module.h"

typedef std::vector<std::pair<int, int> > universalVec;
typedef CSimpleIniA::TNamesDepend::const_iterator ini_i;

#define DEFAULT_SOCKET_BUFLEN 512

#ifdef _WIN32
EXTERN_C IMAGE_DOS_HEADER __ImageBase;
#endif

/* GLOBALS CONFIG */
const unsigned int COUNT_FUNCTIONS = 9;
const unsigned int COUNT_AXIS = 3;

#define DEFINE_ALL_AXIS              \
  ADD_ROBOT_AXIS("locked", 1, 0)     \
  ADD_ROBOT_AXIS("straight", 200, 0) \
  ADD_ROBOT_AXIS("rotation", 200, 0)

#define CREATE_CALIBRATION_MESSAGE_PART(NAME)                        \
  temp_calibration_value = ini.GetLongValue(tstr.c_str(), NAME, -1); \
  if (temp_calibration_value == -1) {                                \
    colorPrintf(ConsoleColor(ConsoleColor::red),                     \
                "Can't read 'calibration'\n");                       \
    return 1;                                                        \
  }                                                                  \
  temp.assign(std::to_string(temp_calibration_value));               \
  while (temp.length() < 3) {                                        \
    temp.insert(0, "0");                                             \
  }                                                                  \
  initcalibration += temp;

universalVec vec_rotate, vec_move;

bool pred(const std::pair<int, int> &a, const std::pair<int, int> &b) {
  return a.first < b.first;
}

/////////////////////////////////////////////////

// sending and receiving messages
std::string TarakanRobot::sendAndRecv(std::string command_for_robot) {
  fd_set readset;
  FD_ZERO(&readset);
  FD_SET(s, &readset);

  send(s, command_for_robot.c_str(), command_for_robot.length(), 0);

  int result = select(s + 1, &readset, NULL, NULL, NULL);
  if (result < 0 && errno != EINTR) {
    colorPrintf(ConsoleColor(ConsoleColor::red), "Error in select(): %s\n",
                strerror(errno));
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

  return recvstr;
}

long int TarakanRobot::getParametrsToTime(variable_value parametr,
                                          universalVec *linkOfaddressMemVec) {
  universalVec::const_iterator iter_key;
  int min = 0, max = 0, minValue = 0, maxValue = 0, count = 0;
  long double y = 0;
  bool flagEnd = false;
  for (iter_key = linkOfaddressMemVec->begin();
       iter_key != linkOfaddressMemVec->end(); ++iter_key) {
    if (parametr >= iter_key->first) {
      min = iter_key->first;
      count = iter_key->second;
    } else {
      max = iter_key->first;
      maxValue = iter_key->second;
      minValue = count;
      flagEnd = true;
      break;
    }
    count++;
  }
  if (!flagEnd) {
    --iter_key;
    max = iter_key->first;
    maxValue = iter_key->second;
    --iter_key;
    min = iter_key->first;
    minValue = iter_key->second;
  }

  y = (((maxValue - minValue) * (parametr - min)) / (max - min)) + minValue;

  return long(std::rint(y));
}

TarakanRobotModule::TarakanRobotModule() {
  {
    robot_functions = new FunctionData *[COUNT_FUNCTIONS];
    system_value function_id = 0;

    // DEFINE_ALL_FUNCTIONS

    FunctionData::ParamTypes *Params = new FunctionData::ParamTypes[3];
    Params[0] = FunctionData::ParamTypes::FLOAT;
    Params[1] = FunctionData::ParamTypes::FLOAT;
    Params[2] = FunctionData::ParamTypes::FLOAT;
    robot_functions[function_id] =
        new FunctionData(function_id + 1, 3, Params, "moveTo");
    function_id++;

    Params = new FunctionData::ParamTypes[2];
    Params[0] = FunctionData::ParamTypes::FLOAT;
    Params[1] = FunctionData::ParamTypes::FLOAT;
    robot_functions[function_id] =
        new FunctionData(function_id + 1, 2, Params, "rotateTo");
    function_id++;

    Params = new FunctionData::ParamTypes[4];
    Params[0] = FunctionData::ParamTypes::FLOAT;
    Params[1] = FunctionData::ParamTypes::FLOAT;
    Params[2] = FunctionData::ParamTypes::FLOAT;
    Params[3] = FunctionData::ParamTypes::FLOAT;
    robot_functions[function_id] =
        new FunctionData(function_id + 1, 4, Params, "moveToByTime");
    function_id++;

    Params = new FunctionData::ParamTypes[3];
    Params[0] = FunctionData::ParamTypes::FLOAT;
    Params[1] = FunctionData::ParamTypes::FLOAT;
    Params[2] = FunctionData::ParamTypes::FLOAT;
    robot_functions[function_id] =
        new FunctionData(function_id + 1, 3, Params, "rotateToByTime");
    function_id++;

    Params = new FunctionData::ParamTypes[4];
    Params[0] = FunctionData::ParamTypes::FLOAT;
    Params[1] = FunctionData::ParamTypes::FLOAT;
    Params[2] = FunctionData::ParamTypes::FLOAT;
    Params[3] = FunctionData::ParamTypes::FLOAT;
    robot_functions[function_id] =
        new FunctionData(function_id + 1, 4, Params, "changeLightMode");
    function_id++;

    Params = new FunctionData::ParamTypes[1];
    Params[0] = FunctionData::ParamTypes::FLOAT;
    robot_functions[function_id] =
        new FunctionData(function_id + 1, 1, Params, "getDistanceObstacle");
    function_id++;

    robot_functions[function_id] =
        new FunctionData(function_id + 1, 0, NULL, "stop");
    function_id++;

    robot_functions[function_id] =
        new FunctionData(function_id + 1, 0, NULL, "startMotors");
    function_id++;

    robot_functions[function_id] =
        new FunctionData(function_id + 1, 0, NULL, "stopMotors");
    function_id++;
  }
  {
    robot_axis = new AxisData *[COUNT_AXIS];
    system_value axis_id = 0;
    DEFINE_ALL_AXIS
  }
}

const char *TarakanRobotModule::getUID() {
  return "Tarakan robot module 1.00 for presentaion";
}

void TarakanRobotModule::prepare(colorPrintfModule_t *colorPrintf_p,
                                 colorPrintfModuleVA_t *colorPrintfVA_p) {
  this->colorPrintf_p = colorPrintfVA_p;
}
void TarakanRobot::prepare(colorPrintfRobot_t *colorPrintf_p,
                           colorPrintfRobotVA_t *colorPrintfVA_p) {
  this->colorPrintf_p = colorPrintfVA_p;
}
void TarakanRobotModule::colorPrintf(ConsoleColor colors, const char *mask,
                                     ...) {
  va_list args;
  va_start(args, mask);
  (*colorPrintf_p)(this, colors, mask, args);
  va_end(args);
}

void TarakanRobot::colorPrintf(ConsoleColor colors, const char *mask, ...) {
  va_list args;
  va_start(args, mask);
  (*colorPrintf_p)(this, uniq_name, colors, mask, args);
  va_end(args);
}

void *TarakanRobotModule::writePC(unsigned int *buffer_length) {
  *buffer_length = 0;
  return NULL;
}

int TarakanRobotModule::init() {
  srand(time(NULL));

#ifdef _WIN32
  WSADATA wsd;
  if (WSAStartup(MAKEWORD(2, 2), &wsd) != 0) {
    colorPrintf(ConsoleColor(ConsoleColor::red),
                "Unable to load Winsock! Error code is %d\n",
                WSAGetLastError());
    return 1;
  }

  colorPrintf(ConsoleColor(ConsoleColor::green),
              "WSAStartup() is OK, Winsock lib loaded!\n");

  InitializeCriticalSection(&TRM_cs);

  WCHAR DllPath[MAX_PATH] = {0};
  GetModuleFileNameW((HINSTANCE)&__ImageBase, DllPath, _countof(DllPath));

  WCHAR *tmp = wcsrchr(DllPath, L'\\');
  WCHAR ConfigPath[MAX_PATH] = {0};

  size_t path_len = tmp - DllPath;

  wcsncpy(ConfigPath, DllPath, path_len);
  wcscat(ConfigPath, L"\\config.ini");

#else
  pthread_mutex_init(&TRM_mx, NULL);

  Dl_info PathToSharedObject;
  void *pointer = reinterpret_cast<void *>(getRobotModuleObject);
  dladdr(pointer, &PathToSharedObject);
  std::string dltemp(PathToSharedObject.dli_fname);

  int dlfound = dltemp.find_last_of("/");

  dltemp = dltemp.substr(0, dlfound);
  dltemp += "/config.ini";

  const char *ConfigPath = dltemp.c_str();
#endif

  CSimpleIniA ini;
  ini.SetMultiKey(true);
  if (ini.LoadFile(ConfigPath) < 0) {
    colorPrintf(ConsoleColor(ConsoleColor::red), "Can't load '%s' file!\n",
                ConfigPath);
    return 1;
  }

  CSimpleIniA::TNamesDepend keys;

  int tcor = ini.GetLongValue(
      "main", "count_robots",
      -1);  // count of robots // returns 0 if count_robots is absent
  if (tcor == -1) {
    colorPrintf(ConsoleColor(ConsoleColor::red), "Can't read 'count_robots'\n");
    return 1;
  }

  for (int i = 1; i <= tcor; i++) {  // for each robot
    vec_rotate.clear();
    vec_move.clear();

    std::string tstr("tarakan_");
    tstr += std::to_string(i);

    // Create calibration message
    std::string initcalibration("C");
    std::string temp("");
    int temp_calibration_value;
    CREATE_CALIBRATION_MESSAGE_PART("SERV_R_STOP")
    CREATE_CALIBRATION_MESSAGE_PART("SERV_L_STOP")
    CREATE_CALIBRATION_MESSAGE_PART("SERV_R_FORW")
    CREATE_CALIBRATION_MESSAGE_PART("SERV_L_FORW")
    CREATE_CALIBRATION_MESSAGE_PART("SERV_R_BACK")
    CREATE_CALIBRATION_MESSAGE_PART("SERV_L_BACK")
    initcalibration += "&";
    // end calibration message

    int robot_number = ini.GetLongValue(tstr.c_str(), "robot_number", -1);
    if (robot_number == -1) {
      colorPrintf(ConsoleColor(ConsoleColor::red),
                  "Can't read 'robot_number'\n");
      return 1;
    }

    ini.GetAllKeys(tstr.c_str(), keys);
    for (ini_i ini_key = keys.begin(); ini_key != keys.end(); ++ini_key) {
      std::string tIniTime(ini_key->pItem);

      if (tIniTime.find("move") != std::string::npos) {
        tIniTime = tIniTime.substr(10);
        int temp_move_value =
            ini.GetLongValue(tstr.c_str(), ini_key->pItem, -1);
        if (temp_move_value == -1) {
          colorPrintf(ConsoleColor(ConsoleColor::red), "Can't read '%s'\n",
                      ini_key->pItem);
          return 1;
        }
        vec_move.push_back(
            std::make_pair(temp_move_value, std::stoi(tIniTime, nullptr, 10)));
      };
      if (tIniTime.find("rotate") != std::string::npos) {
        tIniTime = tIniTime.substr(12);
        int temp_rotate_value =
            ini.GetLongValue(tstr.c_str(), ini_key->pItem, -1);
        if (temp_rotate_value == -1) {
          colorPrintf(ConsoleColor(ConsoleColor::red), "Can't read '%s'\n",
                      ini_key->pItem);
          return 1;
        }
        vec_rotate.push_back(std::make_pair(temp_rotate_value,
                                            std::stoi(tIniTime, nullptr, 10)));
      };
    }
    keys.clear();

    std::sort(vec_rotate.begin(), vec_rotate.end(), pred);
    std::sort(vec_move.begin(), vec_move.end(), pred);

    // only one field connection in tarakan_x section
    const char *pCon = ini.GetValue(tstr.c_str(), "connection", NULL);
    if (!pCon) {
      colorPrintf(ConsoleColor(ConsoleColor::red), "Can't read 'connection'\n");
      return 1;
    }
    std::string connection(pCon);
    TarakanRobot *tarakan_robot = new TarakanRobot(
        connection, initcalibration, vec_rotate, vec_move, robot_number);
    aviable_connections.push_back(tarakan_robot);
  }

  return 0;
}

FunctionData **TarakanRobotModule::getFunctions(unsigned int *count_functions) {
  (*count_functions) = COUNT_FUNCTIONS;
  return robot_functions;
}

AxisData **TarakanRobotModule::getAxis(unsigned int *count_axis) {
  (*count_axis) = COUNT_AXIS;
  return robot_axis;
}

Robot *TarakanRobotModule::robotRequire() {
#ifdef _WIN32
  EnterCriticalSection(&TRM_cs);
  for (auto i = aviable_connections.begin(); i != aviable_connections.end();
       ++i) {
    if ((*i)->require()) {
      LeaveCriticalSection(&TRM_cs);
      return (*i);
    }
  }
  LeaveCriticalSection(&TRM_cs);
  return NULL;
#else
  pthread_mutex_lock(&TRM_mx);
  for (auto i = aviable_connections.begin(); i != aviable_connections.end();
       ++i) {
    if ((*i)->require()) {
      pthread_mutex_unlock(&TRM_mx);
      return (*i);
    }
  }
  pthread_mutex_unlock(&TRM_mx);
  return NULL;
#endif
}

void TarakanRobotModule::robotFree(Robot *robot) {
  TarakanRobot *tarakan_robot = reinterpret_cast<TarakanRobot *>(robot);

#ifdef _WIN32
  EnterCriticalSection(&TRM_cs);
  for (m_connections_i i = aviable_connections.begin();
       i != aviable_connections.end(); ++i) {
    if ((*i) == tarakan_robot) {
      tarakan_robot->free();
      LeaveCriticalSection(&TRM_cs);
      return;
    }
  }
  LeaveCriticalSection(&TRM_cs);
#else
  pthread_mutex_lock(&TRM_mx);
  for (m_connections_i i = aviable_connections.begin();
       i != aviable_connections.end(); ++i) {
    if ((*i) == tarakan_robot) {
      tarakan_robot->free();
      pthread_mutex_unlock(&TRM_mx);
      return;
    }
  }
  pthread_mutex_unlock(&TRM_mx);
#endif
}

void TarakanRobotModule::final() {
  for (m_connections_i i = aviable_connections.begin();
       i != aviable_connections.end(); ++i) {
    delete (*i);
  }
  aviable_connections.clear();

  vec_rotate.clear();
  vec_move.clear();

#ifdef _WIN32
  DeleteCriticalSection(&TRM_cs);
  WSACleanup();
#else
  pthread_mutex_destroy(&TRM_mx);
#endif
}

void TarakanRobotModule::destroy() {
  for (unsigned int j = 0; j < COUNT_FUNCTIONS; ++j) {
    if (robot_functions[j]->count_params) {
      delete[] robot_functions[j]->params;
    }
    delete robot_functions[j];
  }
  for (unsigned int j = 0; j < COUNT_AXIS; ++j) {
    delete robot_axis[j];
  }
  delete[] robot_functions;
  delete[] robot_axis;
  delete this;
}

TarakanRobot::TarakanRobot(std::string connection, std::string calibration,
                           universalVec vec_rotate, universalVec vec_move,
                           unsigned int uniq_index)
    : is_aviable(true),
      is_locked(true),
      connection(connection),
      calibration(calibration),
      vec_rotate(vec_rotate),
      vec_move(vec_move) {
  uniq_name = new char[40];
  sprintf(uniq_name, "robot-%u", uniq_index);

  for (unsigned int i = 0; i < COUNT_AXIS; ++i) {
    axis_state.push_back(1);
  }
}

TarakanRobot::~TarakanRobot() { delete[] uniq_name; };

bool TarakanRobot::require() {
  if (!is_aviable) {
    return false;
  }

#ifdef _WIN32
  s = socket(AF_BTH, SOCK_STREAM, BTHPROTO_RFCOMM);
  printf("Try connect with: %s\n", connection.c_str());
  if (s == INVALID_SOCKET) {
    printf("Socket creation failed, error %d\n", WSAGetLastError());
    return false;
  }
  printf("socket() looks fine!\n");
#else
  s = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);
#endif

#ifdef _WIN32
  SOCKADDR_BTH sab;

  memset(&sab, 0, sizeof(sab));
  sab.addressFamily = AF_BTH;
  sab.btAddr = std::stoll(connection.c_str(), nullptr, 16);
  sab.serviceClassId = SerialPortServiceClass_UUID;
  sab.port = BT_PORT_ANY;

  if (connect(s, (SOCKADDR *)&sab, sizeof(sab)) == SOCKET_ERROR) {
    // This is magic
    if (connect(s, (SOCKADDR *)&sab, sizeof(sab)) == SOCKET_ERROR) {
      printf("connect() failed with error code %d\n", WSAGetLastError());
      closesocket(s);
      return false;
    }
  }
#else
  std::string tempsubstr = connection.substr(2);
  sockaddr_rc addr = {0};
  addr.rc_family = AF_BLUETOOTH;
  addr.rc_channel = (uint8_t)1;

  for (int i = 5; i > 0; i--) {
    tempsubstr.insert(i * 2, ":");
  }

  str2ba(tempsubstr.c_str(), &addr.rc_bdaddr);
  // connect to server
  int status;
  status = connect(s, (struct sockaddr *)&addr, sizeof(addr));
  if (status) {
    printf("failed to connect the device!\n");
    return false;
  }
#endif

  printf("Connected to %s robo\n", connection.c_str());
  is_aviable = false;

  // Apply calibration
  try {
    sendAndRecv(calibration);
    sendAndRecv("8&");
  } catch (...) {
  }

  return true;
}

void TarakanRobot::free() {
  if (is_aviable) {
    return;
  }
  is_aviable = true;

  // Stop Motors
  try {
    sendAndRecv("9&");
  } catch (...) {
  }

#ifdef _WIN32
  closesocket(s);
#else
  close(s);
#endif
}

FunctionResult *TarakanRobot::executeFunction(CommandMode mode,
                                              system_value command_index,
                                              void **args) {
  if (!command_index) {
    return NULL;
  }

  FunctionResult *fr;
  try {
    std::string command_for_robot = "";
    bool need_result = false;
    if ((command_index != ROBOT_COMMAND_HAND_CONTROL_BEGIN) &&
        (command_index != ROBOT_COMMAND_HAND_CONTROL_END) &&
        (command_index != 7) && (command_index != 8) && (command_index != 9)) {
      variable_value *input1 = (variable_value *)(*args);
      if ((*input1 != 0) && (*input1 != 1)) {
        throw std::exception();
      }
      command_for_robot += std::to_string(command_index);
      command_for_robot += *input1 ? "1" : "0";
    }

    switch (command_index) {
      case ROBOT_COMMAND_HAND_CONTROL_BEGIN:
        command_for_robot += "B";
        break;
      case ROBOT_COMMAND_HAND_CONTROL_END:
        command_for_robot += "E";
        break;
      case 1: {  // moveTo
        variable_value *input2 =
            (variable_value *)(*(args + 1));  // On/Off distance test
        variable_value *input3 = (variable_value *)(*(args + 2));  // distance
        if (*input3 < 0) {
          throw std::exception();
        }

        command_for_robot += *input2 ? "1" : "0";
        command_for_robot +=
            std::to_string(getParametrsToTime(*input3, &vec_move));

        break;
      }
      case 2: {  // rotateTo
        variable_value *input2 = (variable_value *)(*(args + 1));  // distance
        if (*input2 < 0) {
          throw std::exception();
        }

        command_for_robot +=
            std::to_string(getParametrsToTime(*input2, &vec_rotate));
        break;
      }
      case 3: {  // moveToByTime
        variable_value *input2 =
            (variable_value *)(*(args + 1));  // On/Off distance test
        variable_value *input3 = (variable_value *)(*(args + 2));  // speed
        if ((*input3 < 0) || (*input3 > 100)) {
          throw std::exception();
        }
        variable_value *input4 = (variable_value *)(*(args + 3));  // time
        if (*input4 < 0) {
          throw std::exception();
        }

        std::string temp(std::to_string((int)*input3));
        while (temp.length() < 3) {
          temp.insert(0, "0");
        }

        command_for_robot += *input2 ? "1" : "0";
        command_for_robot += temp;
        command_for_robot += std::to_string((int)*input4);
        break;
      }
      case 4: {  // rotateToByTime
        variable_value *input2 = (variable_value *)(*(args + 1));  // speed
        if ((*input2 < 0) || (*input2 > 100)) {
          throw std::exception();
        }
        variable_value *input3 = (variable_value *)(*(args + 2));  // time
        if (*input3 < 0) {
          throw std::exception();
        }

        std::string temp(std::to_string((int)*input2));
        while (temp.length() < 3) {
          temp.insert(0, "0");
        }

        command_for_robot += temp;
        command_for_robot += std::to_string((int)*input3);
        break;
      }
      case 5: {  // changeLightMode
        variable_value *input2 = (variable_value *)(*(args + 1));  // LED number
        if (*input2 < 0 || *input2 > 4) {
          throw std::exception();
        }
        variable_value *input3 = (variable_value *)(*(args + 2));  // On/Off
        variable_value *input4 = (variable_value *)(*(args + 3));  // Period
        if (*input4 < 0) {
          throw std::exception();
        }

        command_for_robot += std::to_string((int)*input2);
        command_for_robot += *input3 ? "1" : "0";
        command_for_robot += std::to_string((int)*input4);
        break;
      }
      case 6: {  // getDistanceObstacle
        need_result = true;
        break;
      }
      case 7:    // Stop
      case 8:    // Motors On
      case 9: {  // Motors Off/Stop
        command_for_robot += std::to_string(command_index);
        break;
      }
      default:
        break;
    }
    command_for_robot += "&";

    std::string recvstr = sendAndRecv(command_for_robot);

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
    if (((is_locked) && (!value)) || ((!is_locked) && (value))) {
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

    // create three-digit number
    if (axis_index != 1) {
      std::string temp(std::to_string((int)value));
      while (temp.length() < 3) {
        temp.insert(0, "0");
      }
      command_for_robot += temp;
    } else {
      command_for_robot += std::to_string((int)value);
    }

    command_for_robot += "&";
    send(s, command_for_robot.c_str(), command_for_robot.length(), 0);
    colorPrintf(ConsoleColor(ConsoleColor::green), "%s\n",
                command_for_robot.c_str());
  }
}

int TarakanRobotModule::startProgram(int uniq_index) { return 0; }

void TarakanRobotModule::readPC(void *buffer, unsigned int buffer_length) {}

int TarakanRobotModule::endProgram(int uniq_index) { return 0; }

PREFIX_FUNC_DLL RobotModule *getRobotModuleObject() {
  return new TarakanRobotModule();
}
