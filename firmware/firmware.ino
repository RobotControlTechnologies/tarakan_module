#define LED_PIN_DIRECTION 2
#define LED_PIN_GABARIT 3
#define TRIG_BEGIN 6
#define ECHO_BEGIN 7
#define TRIG_END 8
#define ECHO_END 9
#define PIN_MOTOR_A_ENABLE 4 //LEFT
#define PIN_MOTOR_B_ENABLE 5 //RIGHT
#define PIN_MOTOR_A_INPUT_1 28
#define PIN_MOTOR_A_INPUT_2 26
#define PIN_MOTOR_B_INPUT_1 30
#define PIN_MOTOR_B_INPUT_2 32

const int motor_speed_max = 200;
int motor_speed_now = 0;
String input_buffer = "";

unsigned long moving_time = 0;
unsigned long previous_millis_moving = 0;
unsigned long previous_millis_accelarting = 0;
unsigned long last_time_obstacle_check = 0;

bool is_moving = false;
bool is_acceleration = false;
bool is_rotation = false;
bool is_forward = false;

int interval_gabarit = 0;
int interval_direction = 0;
int intensity_gabarit = 0;
int intensity_direction = 0;

unsigned long current_millis;
unsigned long previous_millis_gabarit = 0;
unsigned long previous_millis_direction = 0;
int led_state_gabarit = LOW;
int led_state_direction = LOW;

bool is_hand_control = false;

void setup() {
  Serial.begin(9600);
  
  pinMode(TRIG_BEGIN, OUTPUT); 
  pinMode(ECHO_BEGIN, INPUT);
  pinMode(TRIG_END, OUTPUT); 
  pinMode(ECHO_END, INPUT);
  
  pinMode(LED_PIN_DIRECTION, OUTPUT);
  pinMode(LED_PIN_GABARIT, OUTPUT);
  
  pinMode(PIN_MOTOR_A_ENABLE, OUTPUT);
  pinMode(PIN_MOTOR_B_ENABLE, OUTPUT);
  pinMode(PIN_MOTOR_A_INPUT_1, OUTPUT);
  pinMode(PIN_MOTOR_A_INPUT_2, OUTPUT); 
  pinMode(PIN_MOTOR_B_INPUT_1, OUTPUT);
  pinMode(PIN_MOTOR_B_INPUT_2, OUTPUT);  
  
  digitalWrite(TRIG_BEGIN, LOW);
  digitalWrite(ECHO_BEGIN, LOW);
  digitalWrite(TRIG_END, LOW);
  digitalWrite(ECHO_END, LOW);
  
  digitalWrite(LED_PIN_DIRECTION, LOW);
  digitalWrite(LED_PIN_GABARIT, LOW);
  
  digitalWrite(PIN_MOTOR_A_INPUT_2, LOW);
  digitalWrite(PIN_MOTOR_B_INPUT_2, LOW);
  digitalWrite(PIN_MOTOR_A_INPUT_1, LOW); 
  digitalWrite(PIN_MOTOR_B_INPUT_1, LOW);
  analogWrite(PIN_MOTOR_A_ENABLE, 0);
  analogWrite(PIN_MOTOR_B_ENABLE, 0);
}

void loop() {
  unsigned int count_bytes = Serial.available();
  if (count_bytes) {
    while(count_bytes--){
       input_buffer = input_buffer + (char)(Serial.read());
    }
    
    if (input_buffer.length()) {
      int last_char = input_buffer.indexOf('&');
      if (last_char != -1) {
        switch (input_buffer[0]) {
          case '1': //moveTo
          case '3': //moveToByTime
            {
              moving_time = input_buffer.substring(2, last_char - 1).toInt();
              robotMove(input_buffer[1] == '1');
            }
            break;
          case '2': //rotateTo
          case '4': //rotateToByTime
            {
              moving_time = input_buffer.substring(2, last_char - 1).toInt();
              robotRotate(input_buffer[1] == '1');
            }
            break;
          case '5': //changeLightMode
            {
              int intensity = map(input_buffer.substring(2, 4).toInt(), 0, 100, 0, 255);
              int period;
              if (intensity) {
                period = input_buffer.substring(5, last_char - 1).toInt();
              } else {
                period = 0; 
              }
              if (input_buffer[1] == '1') {
                interval_direction = period;
                intensity_direction = intensity;
                
                if (!period) {
                  analogWrite(LED_PIN_DIRECTION, intensity);
                }
              } else {
                interval_gabarit = period;
                intensity_gabarit = intensity;
                
                if (!period) {
                  analogWrite(LED_PIN_GABARIT, intensity);
                }
              }
              sendShortAnswer(true);
            }
            break;
          case '6': //getDistanceObstacle
            {
              sendNormalAnswer(String(distanceIK(input_buffer[1] == '1')));
            }
            break;
          case 'B': //ROBOT_COMMAND_HAND_CONTROL_BEGIN
            {
              is_hand_control = true;

              interval_direction = 0;
              interval_gabarit = 0;
              analogWrite(LED_PIN_GABARIT, 0);
              analogWrite(LED_PIN_DIRECTION, 0);
              
              sendShortAnswer(true);
            }
            break;
          case 'E': //ROBOT_COMMAND_HAND_CONTROL_END
            {
              is_hand_control = false;
              robotStop();
              
              interval_direction = 0;
              interval_gabarit = 0;
              analogWrite(LED_PIN_GABARIT, 0);
              analogWrite(LED_PIN_DIRECTION, 0);
              
              sendShortAnswer(true);
            }
            break;
          case 'H': //axisControl call
            {
              switch (input_buffer[1]) {
                case '1':
                  {
                    if (input_buffer[2] == '0') {
                      analogWrite(LED_PIN_GABARIT, 255);
                      analogWrite(LED_PIN_DIRECTION, 255);
                    } else {
                      analogWrite(LED_PIN_GABARIT, 0);
                      analogWrite(LED_PIN_DIRECTION, 0);
                      robotStop();
                    }
                  }
                  break;
                case '2':
                  {
                    switch (input_buffer[2]) {
                      case '0': { robotMove(false); } break; //backward
                      case '1': { robotStop(); } break; //stop
                      case '2': { robotMove(true); } break; //forward                      
                    }
                  }
                  break;
                case '3':
                  {
                    switch (input_buffer[2]) {
                      case '0': { robotRotate(true); } break; //right
                      case '1': { robotStop(); } break; //stop
                      case '2': { robotRotate(false); } break; //left                      
                    }
                  }
                  break;
              }
            }
            break;
        }
        input_buffer.remove(0, last_char + 1);  
      }
    }
  }
  
  if (is_moving) {
    if (is_acceleration) {
      current_millis = millis();
      if (current_millis - previous_millis_accelarting > 30) {
        previous_millis_accelarting = current_millis; 

        motor_speed_now++;
        analogWrite(PIN_MOTOR_A_ENABLE, motor_speed_now);
        analogWrite(PIN_MOTOR_B_ENABLE, motor_speed_now);
        
        if (motor_speed_now >= motor_speed_max) {
          is_acceleration = false;
        }
      }
    }
    
    if (!is_hand_control) {
      current_millis = millis();
      if (current_millis - previous_millis_moving > moving_time) {
        previous_millis_moving = current_millis;
        robotStop();
        sendShortAnswer(true);
      } else if (!is_rotation) {
        if (distanceIK(is_forward) < 15) { 
          robotStop();
          sendShortAnswer(false);
        }
      }
    }
  }
  
  if (interval_gabarit) {
    current_millis = millis();
    if (current_millis - previous_millis_gabarit > interval_gabarit) {
      previous_millis_gabarit = current_millis; 
      if (led_state_gabarit) {
        led_state_gabarit = 0;
      } else {
        led_state_gabarit = intensity_gabarit;
      }
      analogWrite(LED_PIN_GABARIT, led_state_gabarit);
    }  
  }
  
  if (interval_direction){
    current_millis = millis();
    if(current_millis - previous_millis_direction > interval_direction) {
      previous_millis_direction = current_millis; 
      if (led_state_direction) {
        led_state_direction = 0;
      } else {
        led_state_direction = intensity_direction;
      }
      analogWrite(LED_PIN_DIRECTION, led_state_direction);
    }
  }
}

void robotMove(bool forward){
  if (forward) {
    motorForwardA();
    motorForwardB();
  } else {
    motorBackwardA();
    motorBackwardB();
  }
  
  motor_speed_now = 0;
  is_acceleration = true;
  is_moving = true;
  is_rotation = false;
  is_forward = forward;
}
void robotRotate(bool right){
  if (right) {
    motorBackwardA();
    motorForwardB();
  } else {
    motorForwardA();
    motorBackwardB();
  }

  motor_speed_now = 0;
  is_acceleration = true;
  is_moving = true;
  is_rotation = true;
  is_forward = right;
}
void robotStop(){ 
  analogWrite(PIN_MOTOR_A_ENABLE, 0); 
  digitalWrite(PIN_MOTOR_A_INPUT_2, LOW);
  digitalWrite(PIN_MOTOR_A_INPUT_1, LOW);
  analogWrite(PIN_MOTOR_B_ENABLE, 0); 
  digitalWrite(PIN_MOTOR_B_INPUT_2, LOW);
  digitalWrite(PIN_MOTOR_B_INPUT_1, LOW);
  is_moving = false;
  is_acceleration = false;
  motor_speed_now = 0;

  delay(500);
}

void motorForwardA(){
  digitalWrite(PIN_MOTOR_A_INPUT_1, LOW); 
  digitalWrite(PIN_MOTOR_A_INPUT_2, HIGH);
}
void motorForwardB(){
  digitalWrite(PIN_MOTOR_B_INPUT_1, LOW);
  digitalWrite(PIN_MOTOR_B_INPUT_2, HIGH);
}
void motorBackwardA(){
  digitalWrite(PIN_MOTOR_A_INPUT_1, HIGH);
  digitalWrite(PIN_MOTOR_A_INPUT_2, LOW);
}
void motorBackwardB(){
  digitalWrite(PIN_MOTOR_B_INPUT_1, HIGH);
  digitalWrite(PIN_MOTOR_B_INPUT_2, LOW);
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
