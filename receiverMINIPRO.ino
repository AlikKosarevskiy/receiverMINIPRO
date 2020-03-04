#include <SPI.h>                                          // Подключаем библиотеку  для работы с шиной SPI
#include <nRF24L01.h>                                     // Подключаем файл настроек из библиотеки RF24
#include <RF24.h>                                         // Подключаем библиотеку  для работы с nRF24L01+
//#include <iarduino_4LED.h>                                // Подключаем библиотеку  для работы с четырёхразрядным LED индикатором
#include <Servo.h>                                        // Подключаем библиотеку  для работы с сервоприводами
RF24           radio(9, 10);                              // Создаём объект radio   для работы с библиотекой RF24, указывая номера выводов nRF24L01+ (CE, CSN)
//iarduino_4LED  dispLED(2,3);                              // Создаём объект dispLED для работы с функциями библиотеки iarduino_4LED, с указанием выводов дисплея ( CLK , DIO ) 
Servo          myservo;                                   // Создаём объект myservo для работы с функциями библиотеки Servo
int            data[2];                                   // Создаём массив для приёма данных
int val;

#define enA 5   // PWM for M1
#define enB 6   // PWM for M2
#define in1 7   // PIN forward
#define in2 8   // PIN backward

int motorSpeedA = 0;
int motorSpeedB = 0;

void setup(){
    pinMode(enA, OUTPUT);
    pinMode(enB, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
  
    delay(1000);
    Serial.begin(9600);
    myservo.attach(4);                                    // Подключаем объект myservo к 4 выводу Arduino
  //  dispLED.begin();                                      // Инициируем работу индикатора
    radio.begin();                                        // Инициируем работу nRF24L01+
    radio.setChannel(5);                                  // Указываем канал приёма данных (от 0 до 127), 5 - значит приём данных осуществляется на частоте 2,405 ГГц (на одном канале может быть только 1 приёмник и до 6 передатчиков)
    radio.setDataRate     (RF24_250KBPS);                   // Указываем скорость передачи данных (RF24_250KBPS, RF24_1MBPS, RF24_2MBPS), RF24_1MBPS - 1Мбит/сек
    radio.setPALevel      (RF24_PA_MIN);                 // Указываем мощность передатчика (RF24_PA_MIN=-18dBm, RF24_PA_LOW=-12dBm, RF24_PA_HIGH=-6dBm, RF24_PA_MAX=0dBm)
    radio.openReadingPipe (1, 0x1234567890LL);            // Открываем 1 трубу с идентификатором 0x1234567890 для приема данных (на ожном канале может быть открыто до 6 разных труб, которые должны отличаться только последним байтом идентификатора)
    radio.startListening  ();                             // Включаем приемник, начинаем прослушивать открытую трубу
//  radio.stopListening   ();                             // Выключаем приёмник, если потребуется передать данные
}
void loop(){
    if(radio.available()){                                // Если в буфере имеются принятые данные
        radio.read(&data, sizeof(data));                  // Читаем данные в массив data и указываем сколько байт читать
    //    dispLED.print(data[0]);                           // Выводим показания Trema слайдера на индикатор
// myservo.write(90); 
//delay(50);
//myservo.write(45); 
//delay(50);
//myservo.write(127); 
//delay(50);
//  Serial.println(map(data[1],0,676,0,180));
//Serial.print(data[0]);
  // Serial.println(data[1]);
// delay(5);
  val = data[1];
  myservo.write(val);
//myservo.write(map(data[1],0,1023,0,180));         // Поворачиваем сервопривод на угол заданный Trema потенциометром
   
      int yAxis = data[0]; // Read Joysticks y-axis
 // int yAxis = analogRead(A1); // Read Joysticks Y-axis
  // Y-axis used for forward and backward control
  if (yAxis < 470) {
    // Set Motor A backward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    // Set Motor B backward
  //  digitalWrite(in3, HIGH);
  //  digitalWrite(in4, LOW);
    // Convert the declining Y-axis readings for going backward from 470 to 0 into 0 to 255 value for the PWM signal for increasing the motor speed
    motorSpeedA = map(yAxis, 470, 0, 0, 255);
    motorSpeedB = map(yAxis, 470, 0, 0, 255);
  }
  else if (yAxis > 550) {
    // Set Motor A forward
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    // Set Motor B forward
    //digitalWrite(in3, LOW);
    //digitalWrite(in4, HIGH);
    // Convert the increasing Y-axis readings for going forward from 550 to 1023 into 0 to 255 value for the PWM signal for increasing the motor speed
    motorSpeedA = map(yAxis, 550, 1023, 0, 255);
    motorSpeedB = map(yAxis, 550, 1023, 0, 255);
  }
  // If joystick stays in middle the motors are not moving
  else {
    motorSpeedA = 0;
    motorSpeedB = 0;
  }
    
  analogWrite(enA, motorSpeedA); // Send PWM signal to motor A
  analogWrite(enB, motorSpeedB); // Send PWM signal to motor B
    }
}
