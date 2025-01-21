#include <Servo.h>
#include <WiFiNINA.h>

#define MOTORLATCH 12
#define MOTORCLK 4
#define MOTORENABLE 7
#define MOTORDATA 8
#define MOTOR1_A 2
#define MOTOR1_B 3
#define MOTOR2_A 1
#define MOTOR2_B 4
#define MOTOR3_A 5
#define MOTOR3_B 7
#define MOTOR4_A 0
#define MOTOR4_B 6
#define MOTOR1_PWM 11
#define MOTOR2_PWM 3
#define MOTOR3_PWM 6
#define MOTOR4_PWM 5
#define SERVO1_PWM 10
#define SERVO2_PWM 9
#define FORWARD 1
#define BACKWARD 2
#define BRAKE 3
#define RELEASE 4
Servo servo_1;
Servo servo_2;

char ssid[] = "Participant_Wi-Fi";             //  your network SSID (name) between the " "
char pass[] = "R0y@lM3lbW1F1#";      // your network password between the " "
int keyIndex = 0;                 // your network key Index number (needed only for WEP)
int status = WL_IDLE_STATUS;      //connection status
WiFiServer server(80);            //server socket

WiFiClient client = server.available();

void setup()
{
  Serial.begin(9600);
  //pinMode(ledPin, OUTPUT);
  while (!Serial);
  
  enable_WiFi();
  connect_WiFi();

  server.begin();
  printWifiStatus();

	//Serial.begin(9600);
	Serial.println("Simple Adafruit Motor Shield sketch");
	servo_1.attach(SERVO1_PWM);
	servo_2.attach(SERVO2_PWM);
}
void loop()
{
  client = server.available();

  if (client) {
    printWEB();
  }

/*
	motor(1, FORWARD, 255);
	motor(2, FORWARD, 255);
	motor(3, FORWARD, 255);
	motor(4, FORWARD, 255);
	delay(2000);
	// Be friendly to the motor: stop it before reverse.
	motor(1, RELEASE, 0);
	motor(2, RELEASE, 0);
	motor(3, RELEASE, 0);
	motor(4, RELEASE, 0);
	delay(100);
	motor(1, BACKWARD, 128);
	motor(2, BACKWARD, 128);
	motor(3, BACKWARD, 128);
	motor(4, BACKWARD, 128);
	delay(2000);
	motor(1, RELEASE, 0);
	motor(2, RELEASE, 0);
	motor(3, RELEASE, 0);
	motor(4, RELEASE, 0);
	delay(100);

  // */

}
void motor(int nMotor, int command, int speed)
{
	int motorA, motorB;
	if (nMotor >= 1 && nMotor <= 4)
	{
    switch (nMotor)
    {
      case 1:
        motorA = MOTOR1_A;
        motorB = MOTOR1_B;
      break;
      case 2:
        motorA = MOTOR2_A;
        motorB = MOTOR2_B;
      break;
      case 3:
        motorA = MOTOR3_A;
        motorB = MOTOR3_B;
      break;
      case 4:
        motorA = MOTOR4_A;
        motorB = MOTOR4_B;
      break;
      default:
      break;
    }
    switch (command)
    {
      case FORWARD:
        motor_output (motorA, HIGH, speed);
        motor_output (motorB, LOW, -1); // -1: no PWM set
      break;
      case BACKWARD:
        motor_output (motorA, LOW, speed);
        motor_output (motorB, HIGH, -1); // -1: no PWM set
      break;
      case BRAKE:
        motor_output (motorA, LOW, 255); // 255: fully on.
        motor_output (motorB, LOW, -1); // -1: no PWM set
      break;
      case RELEASE:
        motor_output (motorA, LOW, 0); // 0: output floating.
        motor_output (motorB, LOW, -1); // -1: no PWM set
      break;
      default:
      break;
    }
	}
}
void motor_output (int output, int high_low, int speed)
{
	int motorPWM;
	switch (output)
	{
    case MOTOR1_A:
    case MOTOR1_B:
      motorPWM = MOTOR1_PWM;
    break;
    case MOTOR2_A:
    case MOTOR2_B:
      motorPWM = MOTOR2_PWM;
    break;
    case MOTOR3_A:
    case MOTOR3_B:
      motorPWM = MOTOR3_PWM;
    break;
    case MOTOR4_A:
    case MOTOR4_B:
      motorPWM = MOTOR4_PWM;
    break;
    default:
      speed = -3333;
    break;
	}
	if (speed != -3333)
	{
	  shiftWrite(output, high_low);
	  // set PWM only if it is valid
    if (speed >= 0 && speed <= 255)
    {
      analogWrite(motorPWM, speed);
    }
	}
}
void shiftWrite(int output, int high_low)
{
	static int latch_copy;
	static int shift_register_initialized = false;
	// Do the initialization on the fly,
	// at the first time it is used.
	if (!shift_register_initialized)
	{
    // Set pins for shift register to output
    pinMode(MOTORLATCH, OUTPUT);
    pinMode(MOTORENABLE, OUTPUT);
    pinMode(MOTORDATA, OUTPUT);
    pinMode(MOTORCLK, OUTPUT);
    // Set pins for shift register to default value (low);
    digitalWrite(MOTORDATA, LOW);
    digitalWrite(MOTORLATCH, LOW);
    digitalWrite(MOTORCLK, LOW);
    // Enable the shift register, set Enable pin Low.
    digitalWrite(MOTORENABLE, LOW);
    // start with all outputs (ofthe shift register) low
    latch_copy = 0;
    shift_register_initialized = true;
	}
	// The defines HIGH and LOW are 1 and 0.
	// So this is valid.
	bitWrite(latch_copy, output, high_low);
	shiftOut(MOTORDATA, MOTORCLK, MSBFIRST, latch_copy);
	delayMicroseconds(5); // For safety, not really needed.
	digitalWrite(MOTORLATCH, HIGH);
	delayMicroseconds(5); // For safety, not really needed.
	digitalWrite(MOTORLATCH, LOW);
}

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");

  Serial.print("To see this page in action, open a browser to http://");
  Serial.println(ip);
}

void enable_WiFi() {
  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < "1.0.0") {
    Serial.println("Please upgrade the firmware");
  }
}

void connect_WiFi() {
  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(2000);
  }
}

void printWEB() {

  if (client) {                             // if you get a client,
    Serial.println("new client");           // print a message out the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        if (c == '\n') {                    // if the byte is a newline character

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {

            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();
           
            //create the buttons
            client.print("Click <a href=\"/H\">here</a> move forward<br>");
            client.print("Click <a href=\"/L\">here</a> move backward<br><br>");

            client.print("Click <a href=\"/Right\">here</a> move left<br>");
            client.print("Click <a href=\"/Left\">here</a> move right<br><br>");

            client.print("Click <a href=\"/S\">here</a> stop moving<br><br>");

            int randomReading = analogRead(A1);
            client.print("Random reading from analog pin: ");
            client.print(randomReading);
           
            
            

            // The HTTP response ends with another blank line:
            client.println();
            // break out of the while loop:
            break;
          }
          else {      // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        }
        else if (c != '\r') {    // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }

        if (currentLine.endsWith("GET /Left")) {
          //digitalWrite(ledPin, HIGH); 
          motor(1, RELEASE, 0);
          motor(2, RELEASE, 0);
          motor(3, RELEASE, 0);
          motor(4, RELEASE, 0);
          delay(100);
        	motor(1, FORWARD, 128);
          motor(2, FORWARD, 128);
          motor(3, BACKWARD, 128);
          motor(4, BACKWARD, 128);
          //delay(2000);       
        }
        if (currentLine.endsWith("GET /Right")) {
          //digitalWrite(ledPin, HIGH); 
          motor(1, RELEASE, 0);
          motor(2, RELEASE, 0);
          motor(3, RELEASE, 0);
          motor(4, RELEASE, 0);
          delay(100);
        	motor(1, BACKWARD, 128);
          motor(2, BACKWARD, 128);
          motor(3, FORWARD, 128);
          motor(4, FORWARD, 128);
          //delay(2000);       
        }
        if (currentLine.endsWith("GET /H")) {
          //digitalWrite(ledPin, HIGH); 
          motor(1, RELEASE, 0);
          motor(2, RELEASE, 0);
          motor(3, RELEASE, 0);
          motor(4, RELEASE, 0);
          delay(100);
        	motor(1, FORWARD, 128);
          motor(2, FORWARD, 128);
          motor(3, FORWARD, 128);
          motor(4, FORWARD, 128);
          //delay(2000);       
        }
        if (currentLine.endsWith("GET /L")) {
          //digitalWrite(ledPin, LOW);  
        	// Be friendly to the motor: stop it before reverse.
          motor(1, RELEASE, 0);
          motor(2, RELEASE, 0);
          motor(3, RELEASE, 0);
          motor(4, RELEASE, 0);
          delay(100);
          motor(1, BACKWARD, 128);
          motor(2, BACKWARD, 128);
          motor(3, BACKWARD, 128);
          motor(4, BACKWARD, 128);
          //delay(2000);     
        }
        if (currentLine.endsWith("GET /S")) {
          //digitalWrite(ledPin, LOW);  
          motor(1, RELEASE, 0);
          motor(2, RELEASE, 0);
          motor(3, RELEASE, 0);
          motor(4, RELEASE, 0);
          //delay(2000);    
        }
      }
    }
    // close the connection:
    client.stop();
    Serial.println("client disconnected");
  }
}