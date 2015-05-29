#include <Servo.h>

#define LED_PIN_FORWARD_1 48 //слева направо
#define LED_PIN_FORWARD_2 49
#define LED_PIN_BACKWARD_1 50
#define LED_PIN_BACKWARD_2 51

#define LED_PIN_GABARIT_1 46 //по часовой стрелке начиная с переднего левого
#define LED_PIN_GABARIT_2 47
#define LED_PIN_GABARIT_3 52
#define LED_PIN_GABARIT_4 53

#define TRIG_BEGIN 44
#define ECHO_BEGIN 45
#define TRIG_END 42
#define ECHO_END 43
#define PIN_MOTOR_LEFT 5
#define PIN_MOTOR_RIGHT 3

// start calibration
int SERV_R_STOP = 80, SERV_L_STOP = 88, SERV_R_FORW = 180, SERV_L_FORW = 0, SERV_R_BACK = 0, SERV_L_BACK = 180;

String input_buffer = "";

unsigned long moving_time = 0;
unsigned long previous_millis_moving = 0;
unsigned long last_time_obstacle_check = 0;

bool is_moving = false;
bool is_rotation = false;
bool is_forward = false;
bool is_motor_start = false;
bool is_distance_test_on = false;

const int light_pins[2][4] = {
  {LED_PIN_GABARIT_1, LED_PIN_GABARIT_2, LED_PIN_GABARIT_3, LED_PIN_GABARIT_4},
  {LED_PIN_FORWARD_1, LED_PIN_FORWARD_2, LED_PIN_BACKWARD_1, LED_PIN_BACKWARD_2}
};
unsigned long light_intervals[2][4] = {
  {0, 0, 0, 0}, //gabarit
  {0, 0, 0, 0}  //direction
};
int light_states[2][4] = {
  {LOW, LOW, LOW, LOW},
  {LOW, LOW, LOW, LOW}
};
unsigned long light_previous_time[2][4] = {
  {0, 0, 0, 0},
  {0, 0, 0, 0}
};

unsigned long current_millis;
bool is_hand_control = false;

Servo servo_left;
Servo servo_right;

void setup() {
  Serial.begin(9600);
  
  pinMode(TRIG_BEGIN, OUTPUT); 
  pinMode(ECHO_BEGIN, INPUT);
  pinMode(TRIG_END, OUTPUT); 
  pinMode(ECHO_END, INPUT);
  
  pinMode(LED_PIN_FORWARD_1, OUTPUT);
  pinMode(LED_PIN_FORWARD_2, OUTPUT);
  pinMode(LED_PIN_BACKWARD_1, OUTPUT);
  pinMode(LED_PIN_BACKWARD_2, OUTPUT);
  
  pinMode(LED_PIN_GABARIT_1, OUTPUT);
  pinMode(LED_PIN_GABARIT_2, OUTPUT);
  pinMode(LED_PIN_GABARIT_3, OUTPUT);
  pinMode(LED_PIN_GABARIT_4, OUTPUT);
  
  digitalWrite(TRIG_BEGIN, LOW);
  digitalWrite(ECHO_BEGIN, LOW);
  digitalWrite(TRIG_END, LOW);
  digitalWrite(ECHO_END, LOW);
  
  digitalWrite(LED_PIN_FORWARD_1, LOW);
  digitalWrite(LED_PIN_FORWARD_2, LOW);
  digitalWrite(LED_PIN_BACKWARD_1, LOW);
  digitalWrite(LED_PIN_BACKWARD_2, LOW);
  
  digitalWrite(LED_PIN_GABARIT_1, LOW);
  digitalWrite(LED_PIN_GABARIT_2, LOW);
  digitalWrite(LED_PIN_GABARIT_3, LOW);
  digitalWrite(LED_PIN_GABARIT_4, LOW);
}

void loop() {
  unsigned int count_bytes = Serial.available();
  if (count_bytes) {
    while(count_bytes--){
       input_buffer = input_buffer + (char)(Serial.read());
    }
    
    if (input_buffer.length()) {
      int last_char = input_buffer.indexOf('&');
      while (last_char != -1) {
        switch (input_buffer[0]) {
          case '1': //moveTo
            {
              if (is_motor_start) {
                is_distance_test_on = (input_buffer[2] == '1') ? true : false; // On/Off distance test
                moving_time = input_buffer.substring(3, last_char).toInt();
                robotMove(input_buffer[1] == '1', 100);
              }
              else { sendShortAnswer(false); };
            }
            break;
          case '3': //moveToByTime
            {
              if (is_motor_start) {
                is_distance_test_on = (input_buffer[2] == '1') ? true : false; // On/Off distance test
                int speed_percent = input_buffer.substring(3, 6).toInt();
                moving_time = input_buffer.substring(6, last_char).toInt();
                robotMove(input_buffer[1] == '1', speed_percent);
              }
              else { sendShortAnswer(false);};
            }
            break;
          case '2': //rotateTo
            {
              if (is_motor_start) {
                moving_time = input_buffer.substring(2, last_char).toInt();
                robotRotate(input_buffer[1] == '1', 100);
              }
              else { sendShortAnswer(false); };
            }
            break;
          case '4': //rotateToByTime
            {
              if (is_motor_start) {
                int speed_percent = input_buffer.substring(2, 5).toInt();
                moving_time = input_buffer.substring(5, last_char).toInt();
                robotRotate(input_buffer[1] == '1', speed_percent);
              }
              else { sendShortAnswer(false); };
            }
            break;
          case '5': //changeLightMode
            {
              int led_state;
              int period;
              if (input_buffer[3] == '1') {
                led_state = HIGH;
                period = input_buffer.substring(4, last_char).toInt();
              } else {
                led_state = LOW;
                period = 0;
              }
        
              int group_index;
              if (input_buffer[1] == '1') {
                group_index = 1; 
              } else {
                group_index = 0;
              }
              
              int led_index = input_buffer[2] - 48;
              if ((led_index >= 0) && (led_index <= 4)) {
                if (led_index) {
                  led_index--;
                  changeOneLedState(group_index, led_index, led_state, period);
                } else {
                  changeGroupLedState(group_index, led_state, period);
                }
                sendShortAnswer(true);
              } else {
                sendShortAnswer(false);
              }
            }
            break;
          case '6': //getDistanceObstacle
            {
              sendNormalAnswer(String(distanceIK(input_buffer[1] == '1')));
            }
            break;
          case '7':
            {
              robotStop();
              sendShortAnswer(true);
            }
            break;
          case 'B': //ROBOT_COMMAND_HAND_CONTROL_BEGIN
            {
              is_hand_control = true;

              changeGroupLedState(0, HIGH, 0);
              changeGroupLedState(1, LOW, 0);
              sendShortAnswer(true);
            }
            break;
          case 'E': //ROBOT_COMMAND_HAND_CONTROL_END
            {
              is_hand_control = false;
              robotStop();
              
              changeGroupLedState(0, LOW, 0);
              changeGroupLedState(1, LOW, 0);
              
              sendShortAnswer(true);
            }
            break;
          case 'H': //axisControl call
            {
              switch (input_buffer[1]) {
                case '1':
                  {
                    if (input_buffer[2] == '0') {
                      changeGroupLedState(0, HIGH, 0);
                    } else {
                      changeGroupLedState(0, LOW, 0);
                      robotStop();
                    }
                  }
                  break;
                case '2':
                  {
                    int speed_percent = input_buffer.substring(2, 4).toInt();
                    int speed = map(speed_percent, 0, 200, SERV_L_BACK, SERV_L_FORW);
                    servo_left.write(speed);
                    speed = map(speed_percent, 0, 200, SERV_R_BACK, SERV_R_FORW);
                    servo_right.write(speed);
                  }
                  break;
                case '3':
                  {
                    int speed_percent = input_buffer.substring(2, 4).toInt();
                    int speed = map(speed_percent, 0, 200, SERV_L_BACK, SERV_L_FORW);
                    servo_left.write(speed);
                    speed = map(speed_percent, 0, 200, SERV_R_FORW, SERV_R_BACK);
                    servo_right.write(speed);
                  }
                  break;
              }
            }
          case 'C': // calibration call
            {
              SERV_R_STOP = input_buffer.substring(1, 4).toInt();
              SERV_L_STOP = input_buffer.substring(4, 7).toInt();
              SERV_R_FORW = input_buffer.substring(7, 10).toInt();
              SERV_L_FORW = input_buffer.substring(10, 13).toInt();
              SERV_R_BACK = input_buffer.substring(13, 16).toInt();
              SERV_L_BACK = input_buffer.substring(16, 19).toInt();
              sendShortAnswer(true);  
              break;
            }
          case '8': // Include Motors
            { // Motors On
              if (!is_motor_start){
                servo_left.attach(PIN_MOTOR_LEFT);
                servo_left.write(SERV_L_STOP);

                servo_right.attach(PIN_MOTOR_RIGHT);
                servo_right.write(SERV_R_STOP);

                is_motor_start = true;
              }
              sendShortAnswer(true);
              break;
            }
          case '9': 
            { // Stop Motors
              if (is_motor_start) {
                servo_left.detach();
                servo_right.detach();

                is_motor_start = false;
              } 
              sendShortAnswer(true);
            }
          break;
        }
        input_buffer.remove(0, last_char + 1);
        last_char = input_buffer.indexOf('&');
      }
    }
  }
  
  if (is_moving) {
    if (!is_hand_control) {
      current_millis = millis();
      if (current_millis - previous_millis_moving > moving_time) {
        robotStop();
        sendShortAnswer(true);
      } else if (!is_rotation && is_distance_test_on) {

        if (distanceIK(is_forward) < 15) { 
          robotStop();
          sendShortAnswer(false);
        }

      }
    }
  }
  
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 4; j++) {
      if (light_intervals[i][j]) {
        current_millis = millis();
        if (current_millis - light_previous_time[i][j] > light_intervals[i][j]) {
          light_previous_time[i][j] = current_millis;
          if (light_states[i][j]) {
            light_states[i][j] = LOW;
          } else {
            light_states[i][j] = HIGH;
          }
          digitalWrite(light_pins[i][j], light_states[i][j]);
        }
      }
    }
  }
}

void changeOneLedState(int group, int index, int state, int period) {
  digitalWrite(light_pins[group][index], state);
  light_states[group][index] = state;
  light_intervals[group][index] = period;
}

void changeGroupLedState(int group, int state, int period) {
  for (int i = 0; i < 4; i++) {
    changeOneLedState(group, i, state, period);
  }
}

void robotMove(bool forward, int speed_percent){
  if (forward) {
    motorForwardLeft(speed_percent);
    motorForwardRight(speed_percent);
  } else {
    motorBackwardLeft(speed_percent);
    motorBackwardRight(speed_percent);
  }
  
  is_moving = true;
  is_rotation = false;
  is_forward = forward;
  previous_millis_moving = millis();
}
void robotRotate(bool right, int speed_percent){
  if (right) {
    motorBackwardRight(speed_percent);
    motorForwardLeft(speed_percent);
  } else {
    motorForwardRight(speed_percent);
    motorBackwardLeft(speed_percent);
  }

  is_moving = true;
  is_rotation = true;
  is_forward = right;
  previous_millis_moving = millis();
}
void robotStop(){ 
  servo_left.write(SERV_L_STOP);
  servo_right.write(SERV_R_STOP);
  is_moving = false;
  delay(500);
}
void motorForwardLeft(int speed_percent){
  int speed = map(speed_percent, 0, 100, SERV_L_STOP, SERV_L_FORW);
  servo_left.write(speed);
}
void motorForwardRight(int speed_percent){
  int speed = map(speed_percent, 0, 100, SERV_R_STOP, SERV_R_FORW);
  servo_right.write(speed);
}
void motorBackwardLeft(int speed_percent){
  int speed = map(speed_percent, 0, 100, SERV_L_STOP, SERV_L_BACK);
  servo_left.write(speed);
}
void motorBackwardRight(int speed_percent){
  int speed = map(speed_percent, 0, 100, SERV_R_STOP, SERV_R_BACK);
  servo_right.write(speed);
}
int distanceIK(boolean check_forward){ 
  unsigned int delay_time = millis() - last_time_obstacle_check;
  if (delay_time < 50) {
    delay(50 - delay_time);
  }
  last_time_obstacle_check = millis();
  
  int trig, echo;  
  if(check_forward){
    trig = TRIG_BEGIN;
    echo = ECHO_BEGIN;
  }else{
    trig = TRIG_END;
    echo = ECHO_END;
  }
  
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  
  delay_time = pulseIn(echo, HIGH);
  unsigned int distance_sm = delay_time / 58;
  
  return distance_sm;
}
void sendNormalAnswer(String answer) {
  String output_buffer = "0";
  output_buffer += answer;
  output_buffer += '&';
  Serial.print(output_buffer);
}
void sendShortAnswer(bool isNormal) {
  String output_buffer = "";
  if (isNormal) {
    output_buffer += '0';
  } else {
    output_buffer += '1';
  }
  output_buffer += '&';
  Serial.print(output_buffer);
}
