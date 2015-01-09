#define LED_PIN_DIRECTION 2
#define LED_PIN_GABARIT 3
#define TRIG_BEGIN 6
#define ECHO_BEGIN 7
#define TRIG_END 8
#define ECHO_END 9
#define PIN_MOTOR_1_ENABLE 4
#define PIN_MOTOR_2_ENABLE 5
#define PIN_MOTOR_1_INPUT_1 28
#define PIN_MOTOR_1_INPUT_2 26
#define PIN_MOTOR_2_INPUT_1 30
#define PIN_MOTOR_2_INPUT_2 32

int motor_max=200; //ширина импульса  ШИМ по умолчанию 
int motor_rotation=200; //ширина импульса  ШИМ по умолчанию50%
unsigned int motor_start=0; // флаг инициализации
unsigned int motor_speed=0; // флаг состояние для разгона
unsigned int ultrasound_begin=0; // флаг ультразвука begin
unsigned int ultrasound_end=0; // флаг ультразвука end
unsigned int blink_gabarit=0; // флаг мигания габаритами
unsigned int blink_direction=0; // флаг мигания направляющимми
int led_state_gabarit = LOW;             // этой переменной устанавливаем состояние светодиода
int led_state_direction = LOW;             // этой переменной устанавливаем состояние светодиода
long previous_millis_gabarit = 0;        // храним время последнего переключения светодиода
long previous_millis_direction = 0;        // храним время последнего переключения светодиода 
long interval_gabarit = 0;           // интервал между включение/выключением светодиода (1 секунда)
long interval_direction = 0;           // интервал между включение/выключением светодиода (1 секунда)
long intensity_gabarit =0;                // интенсивность свечения габаритов
long intensity_direction =0;              // интенсивность свечения направляющих
String val, val_two, s1="";
unsigned long current_millis, temp_millis=0;

void setup() {
  Serial.begin(9600);
  pinMode(TRIG_BEGIN, OUTPUT); 
  pinMode(ECHO_BEGIN, INPUT);
  pinMode(TRIG_END, OUTPUT); 
  pinMode(ECHO_END, INPUT);
  pinMode(LED_PIN_DIRECTION, OUTPUT);
  pinMode(LED_PIN_GABARIT, OUTPUT);
  pinMode(PIN_MOTOR_1_ENABLE, OUTPUT);
  pinMode(PIN_MOTOR_2_ENABLE, OUTPUT);
  pinMode(PIN_MOTOR_1_INPUT_1, OUTPUT);
  pinMode(PIN_MOTOR_1_INPUT_2, OUTPUT); 
  pinMode(PIN_MOTOR_2_INPUT_1, OUTPUT);
  pinMode(PIN_MOTOR_2_INPUT_2, OUTPUT);  
  
  digitalWrite(PIN_MOTOR_1_INPUT_2, LOW);
  digitalWrite(PIN_MOTOR_2_INPUT_2, LOW);
  digitalWrite(PIN_MOTOR_1_INPUT_1, LOW); 
  digitalWrite(PIN_MOTOR_2_INPUT_1, LOW);
  digitalWrite(PIN_MOTOR_1_ENABLE, LOW);
  digitalWrite(PIN_MOTOR_2_ENABLE, LOW);
  digitalWrite(TRIG_BEGIN, LOW);
  digitalWrite(ECHO_BEGIN, LOW);
  digitalWrite(TRIG_END, LOW);
  digitalWrite(ECHO_END, LOW);
  digitalWrite(LED_PIN_DIRECTION, LOW);
  digitalWrite(LED_PIN_GABARIT, LOW);
}

void loop() {
  while(Serial.available()){
     s1= s1+(char)(Serial.read());
  }              
  if (s1.length()){
    switch (s1[0]) {
     case 'W': forward(motor_max); break;
     case 'S': backward(motor_max); break;
     case 'T': motorStop(); break;
     case 'L': sLeft(motor_rotation); break;
     case 'R': sRight(motor_rotation); break;
     case 'G': 
              if(correctionErrors(false)){             
                intensity_gabarit=val.toInt(); 
                onGabarit();
              }                          
              break;
     case 'D':
              if(correctionErrors(false)){             
                intensity_direction=val.toInt(); 
                onDirection();
              }                          
              break;
     case 'Q': offDirection(); break;
     case 'A': offGabarit(); break;
     case 'H': onAll(); break;
     case 'J': offAll(); break;
     case 'B': Serial.print(String(distanceIK(true)) + "&"); break;
     case 'E': Serial.print(String(distanceIK(false)) + "&"); break; 
     case 'M':
              if(correctionErrors(true)){           
                 intensity_gabarit=val.toInt(); 
                 interval_gabarit=val_two.toInt(); 
                 blink_gabarit=1;                  
              }
              break;
     case 'K':
              if(correctionErrors(true)){
                intensity_direction=val.toInt(); 
                interval_direction=val_two.toInt();
                blink_direction=1;  
              }               
              break;
    }
    if((s1[0] != 'G') && (s1[0] != 'D') && (s1[0] != 'M') && (s1[0] != 'K')){
      s1.remove(0,1);
    }
  }
//***************************************************
/********** измерение в прямом направлении ********/  
  if(ultrasound_begin){
    if (distanceIK(true) < 15){ 
      Serial.print("V");
      motorStop();
    }
  }
//*****************************************************
/********** измерение в обратном направлении ********/
  if(ultrasound_end){  
    if (distanceIK(false) < 15){
      Serial.print("V");
      motorStop();
    }
  }
//***************************************************
/********** Включить мсигание габаритов ********/
  if (blink_gabarit){
    current_millis = millis();
     //проверяем не прошел ли нужный интервал, если прошел то
    if(current_millis - previous_millis_gabarit > interval_gabarit) {
      // сохраняем время последнего переключения
      previous_millis_gabarit = current_millis; 
      // если светодиод не горит, то зажигаем, и наоборот
      if (led_state_gabarit == LOW)
        led_state_gabarit = (255/100)*intensity_gabarit;
      else
        led_state_gabarit = LOW;   
      // устанавливаем состояния выхода, чтобы включить или выключить светодиод
      analogWrite(LED_PIN_GABARIT, led_state_gabarit);
    }  
  }
//***************************************************
/********** Включить мсигание направляющих ********/
  if (blink_direction){
    current_millis = millis();
     //проверяем не прошел ли нужный интервал, если прошел то
    if(current_millis - previous_millis_direction > interval_direction) {
      // сохраняем время последнего переключения
      previous_millis_direction = current_millis; 
      // если светодиод не горит, то зажигаем, и наоборот
      if (led_state_direction == LOW)
        led_state_direction = (255/100)*intensity_direction;
      else
        led_state_direction = LOW;   
      // устанавливаем состояния выхода, чтобы включить или выключить светодиод
      analogWrite(LED_PIN_DIRECTION, led_state_direction);
    }  
  } 
}
//****************************************************
//********* Управление шириной импульса *******************
//****************************************************
void changeDuty(int speeds){
  if (!motor_speed){
    motor_speed=1; // Сохранить текущее значение скорости
    for (int i = 199; i <= speeds; ++i){
      analogWrite(PIN_MOTOR_1_ENABLE, i);
      analogWrite(PIN_MOTOR_2_ENABLE, i);
      delay(30);
    }   
  }
}
//****************************************************
/********** Двигатель A команда ВПЕРЕД ********/
void motorForwardA(){
  digitalWrite(PIN_MOTOR_1_INPUT_2, LOW);
  digitalWrite(PIN_MOTOR_1_INPUT_1, HIGH); 
}
/************************************/
/****** Двигатель B команда ВПЕРЕД ****/
void motorForwardB(){
 digitalWrite(PIN_MOTOR_2_INPUT_2, LOW);
 digitalWrite(PIN_MOTOR_2_INPUT_1, HIGH);
}
/************************************/
/******** Двигатель A команда НАЗАД*****/
void motorBackA(){
  digitalWrite(PIN_MOTOR_1_INPUT_1, LOW);
  digitalWrite(PIN_MOTOR_1_INPUT_2, HIGH);
}
/************************************/
/******** Двигатель B команда НАЗАД *****/
void motorBackB(){
  digitalWrite(PIN_MOTOR_2_INPUT_1, LOW);
  digitalWrite(PIN_MOTOR_2_INPUT_2, HIGH);
}
/************************************/
/******* Двигатель A выключен **********/
void motorOffA(){
  digitalWrite(PIN_MOTOR_1_ENABLE, LOW); 
  digitalWrite(PIN_MOTOR_1_INPUT_2, LOW);
  digitalWrite(PIN_MOTOR_1_INPUT_1, LOW);
}
/************************************/
/********** Двигатель B выключен *******/
void motorOffB(){
  digitalWrite(PIN_MOTOR_2_ENABLE, LOW); 
  digitalWrite(PIN_MOTOR_2_INPUT_2, LOW);
  digitalWrite(PIN_MOTOR_2_INPUT_1, LOW);
}
/************************************/
/******* Команда ДВИЖЕНИЕ ВПЕРЕД ******/
void forward(int speed){
  motorBackA();
  motorBackB();
  changeDuty(speed);
  ultrasound_begin=1;
}
/************************************/
/****** Команда ДВИЖЕНИЕ НАЗАД ********/
void backward(int speed){ 
  motorForwardA();
  motorForwardB();
  changeDuty(speed);
  ultrasound_end=1;
}
/************************************/
/***** Команда ПОВОРОТ НАПРАВО ** ******/
void sRight(int speed){
  motorForwardA();
  motorBackB();
  changeDuty(speed);
}
/************************************/
/******** Команда ПОВОРОТ НАЛЕВО ******/
void sLeft(int speed){ 
  motorBackA();
  motorForwardB();
  changeDuty(speed);
}
/************************************/
/********** Команда СТОП ************/
void motorStop(){ 
  analogWrite(PIN_MOTOR_1_ENABLE, LOW);
  analogWrite(PIN_MOTOR_2_ENABLE, LOW);
  motorOffA();
  motorOffB();
  motor_speed=0;
  ultrasound_begin=0;
  ultrasound_end=0;
  delay(500);
}
//****************************************************
/********** Включить габариты ********/
void onGabarit(){
  led_state_gabarit = (255/100)*intensity_gabarit;
  analogWrite(LED_PIN_GABARIT, led_state_gabarit);
}
//****************************************************
/********** Включить ходовые ********/
void onDirection(){
  led_state_direction = (255/100)*intensity_direction;
  analogWrite(LED_PIN_DIRECTION, led_state_direction); 
}
//****************************************************
/********** Выключить габариты  ********/
void offGabarit(){
  digitalWrite(LED_PIN_GABARIT, LOW);
  blink_gabarit=0;
}
//****************************************************
/********** Выключить ходовые ********/
void offDirection(){
  digitalWrite(LED_PIN_DIRECTION, LOW); 
  blink_direction=0;
}
//****************************************************
/********** Включить все светодиоды ********/
void onAll(){
   onGabarit();
   onDirection();
}
//***************************************************
/********** Выключить все светодиоды ********/
void offAll(){
  offGabarit();
  offDirection();
  blink_gabarit=0;
  blink_direction=0;
}
//*****************************************************
/********** Проверка передаваемых параметров ********/
boolean correctionErrors(boolean flag){
   int index_end=s1.indexOf('&');
   if(flag){
     val=s1.substring(1,3);
   }else{
     if(index_end == 4){
       val=s1.substring(1,3);
     }else{
       return false;
     }
   }
   val_two="";
   if(flag){ 
     if(index_end != -1){ 
       val_two=s1.substring(4,(index_end));
     }else{
       return false;
     }
   }
  s1.remove(0,(4+val_two.length()+1));
  return true;
}
//*******************************************************
/********** Возвращает растояние до препятствия ********/ 
int distanceIK(boolean flag){ 
  unsigned int time=0;
  unsigned long residual;
  unsigned int distance_sm=0; //возвращет растояние от датчика прямого направления
  int trig, echo; 
  residual=millis() - temp_millis;
  temp_millis=millis();
  if(residual < 50) {
    delay(50 - residual);
  }
  if(flag){
    trig=TRIG_BEGIN;
    echo=ECHO_BEGIN;
  }else{
    trig=TRIG_END;
    echo=ECHO_END;
  }
  digitalWrite(trig, HIGH); // Подаем сигнал на выход микроконтроллера  
  delayMicroseconds(10); // Удерживаем 10 микросекунд 
  digitalWrite(trig, LOW); // Затем убираем
  time=pulseIn(echo, HIGH); // Замеряем длину импульса 
  distance_sm=time/58; // Пересчитываем в сантиметры   
  return distance_sm;
}
//***************************************************
