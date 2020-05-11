#include <DHT.h>
#define DHTTYPE DHT22
#include <U8x8lib.h>
#include <Servo.h>
#define DHT22OUTDOOR 8
#define DHT22inDOOR 10
#define RELAYFAN 12
#define RELAYLIGH1 5
#define RELAYLIGH2 4
#define RELAYSERVO 2
#define DISPLAYSCK 6
#define DISPLAYSDA 7
#define MQ135 A5
#define SUNSENSOR A4
#define HUMANSENSOR A3
#define SERVODATA 9
#define SERVOPOSOPEN 10
#define SERVOPOSCLOSE 60
#define SERVOTIMETOOPERATE 2000
#define HUMADETECTEDRESET 600000 // 300000 time span for man detection
#define HOURNIGHT 23
#define HOURDAY  8
#define TASK_SENSORREAD 2000
#define TASK_DATASEND 10000
#define TASK_ESPUPDATE 30000
#define TEMPTHRESHOLD 25
#define HUMIDTHRESHOLD 90
#define FANTIMETOWORK  3600000//3600000  60000
#define FANPAUSE  1200000//  20000
#define BLINDTHRESHOLD 10 //iner temp to open blind
#define DHTRESETRELAY 3
#define DHTRESETTIME 1000
#define SUNTOONLIGHT 600
#define SUNDIFFERENCE 500 //sun sensor mesures les then detected sun is high. the difference SUNTOONLIGHT and SUNDIFFERENCE 
							//has to be not less than 0

float tempDifoff = 0;
float tempDifon = 0;
float humidon = 0;
float humidoff =0;
float airDifoff = 0;
float airDifon = 0;
bool ServerConected;
int hour, min;
U8X8_SH1106_128X64_NONAME_SW_I2C u8x8(DISPLAYSCK, DISPLAYSDA);
DHT indoorDHT22(DHT22OUTDOOR, DHTTYPE);
DHT outdoorDHT22(DHT22inDOOR, DHTTYPE);

Servo myservo;  
class task;
class Fan;
class Light;
class ventionBlind;
void display(int);
void readsensors();
void sendDataToServer(int index = 1);
void ReadDataSerial();
void resetDHT(bool action = false);
struct sensor {
	bool displayUpdate;
	bool invalidData;
	static bool overallDisplUpdate;
	static int dataReadCounter;
	
};
bool sensor::overallDisplUpdate = false;
int sensor::dataReadCounter;
struct DHTSensor:public sensor {
	float temp;
	float tempBuf;
	float humid;
	float humidBuf;
	int state;
	char index;
	DHTSensor(char index) {
		this->index = index;
	}
	void store(float temp, float humid) {
		tempBuf += temp;
		humidBuf += humid;
	}
	void update(float temp, float humid) {
		if (this->temp != temp || this->humid != humid) {
			if (isnan(temp) || isnan(humid)) {
				invalidData = true;
			} else {
				display(4);//reset indication if OK
				this->temp = temp;
				this->humid = humid;
				sensor::overallDisplUpdate = false;
				displayUpdate = false;
				if (invalidData) {
					invalidData = false;
					display(4);//reset indication if OK
				}
			}			
		}
		tempBuf += temp;
		humidBuf += humid;
	}
	void updateBuf() {
		tempBuf = tempBuf / sensor::dataReadCounter;
		humidBuf = humidBuf / sensor::dataReadCounter;
	}
	void resetBuf() {
		tempBuf = 0;
		humidBuf = 0;
		sensor::dataReadCounter = 0;
	}
	operator const String() {
		String valueToDisplay = String(index)+"T" + String(temp) + " " + String(index) +"H" + String(humid);
		return valueToDisplay;
	}
	
};
DHTSensor DHTInDoor('I');
DHTSensor DHTOutDoor('O');
struct sunAndAir :public sensor {
	int value;
	int valueBuf;
	void update(int value) {
		if (this->value != value) {
			this->value = value;
			sensor::overallDisplUpdate = false;
			displayUpdate = false;
			invalidData = false;
			if (value == 0) {
				invalidData = true;
			}

		}
		valueBuf += value;
	}
	void updateBuf() {
		valueBuf = valueBuf / sensor::dataReadCounter;
	}
	void resetBuf() {
		valueBuf = 0;
		sensor::dataReadCounter = 0;
	}
}sun, air;
struct Man :public sensor {
	bool detected;
	bool active;
	unsigned long lastDetected;
	void update(bool detected) {
		if (this->detected != detected) {
			this->detected = detected;
			sensor::overallDisplUpdate = false;
			displayUpdate = false;
			lastDetected = millis();
			invalidData = false;
		}else{ 
			if(!detected && lastDetected+ HUMADETECTEDRESET < millis())	invalidData = true; // used for light activation
		}
	}
}man;
class ventionBlind {
	unsigned long timeBegin;
	bool startOpen;
	bool startClose;
public:
	bool open=true;
	bool forceClose;
	bool blindTempToOpen;
	bool automatedOperation = true;
	void set(int index) {// set 0 to force close, 1-force open, 2-automated oparation
		switch (index) {
		case 0: forceClose = true; automatedOperation = false; break;
		case 1: forceClose = false; automatedOperation = false; break;
		case 2: automatedOperation = true; break;
		default:break;
		}
	}
	void setToValue(int index) {
		digitalWrite(RELAYSERVO, LOW);
		myservo.write(index);
		delay(1000);
		digitalWrite(RELAYSERVO, HIGH);
	}
	void check() {
		if (!automatedOperation) {
			if (forceClose)closeBlind();
			else openBlind();
		}
		else {
			if (blindTempToOpen) openBlind();
			else closeBlind();
		}
	}
	bool openBlind() {
		if (!open || startClose) {
			if (!startOpen) {
				digitalWrite(RELAYSERVO, LOW);
				startOpen = true;
				timeBegin = millis();
				myservo.write(SERVOPOSOPEN);
				open = false; //reset open in case if while open execution close function is called
				display(7);
				startClose = false;
				return false;
			}
			else {
				if (millis() > timeBegin + SERVOTIMETOOPERATE) {
					digitalWrite(RELAYSERVO, HIGH);
					startOpen = false;
					open = true;
					display(7);
					return true;
				}
				else return false;
			}
		}
		else return true;
	}
	bool closeBlind() {
		if (open || startOpen) {
			if (!startClose) {
				digitalWrite(RELAYSERVO, LOW);
				startClose = true;
				timeBegin = millis();
				myservo.write(SERVOPOSCLOSE);
				open = true; //reset open in case if while close execution open function is called
				display(7);
				startOpen = false;
				return false;
			}
			else {
				if (millis() > timeBegin + SERVOTIMETOOPERATE) {
					digitalWrite(RELAYSERVO, HIGH);
					startClose = false;

					open = false;
					display(7);
					return true;
				}
				else return false;
			}
		}
		else return true;
	}
} blind;

class task {
public:
	unsigned long period;
	bool ignor = false;
	void reLoop() {
		taskLoop = millis();
	};
	bool check() {
		if (!ignor) {
			if (millis() - taskLoop > period) {
				taskLoop = millis();
				return true;
			}
		}
		return false;
	}
	void StartLoop(unsigned long shift) {
		taskLoop = millis() + shift;
	}
	task(unsigned long t) {
		period = t;
	}
	task() {
		period = 1000;
	}
private:
	unsigned long taskLoop;
};
task task_SensorRead(TASK_SENSORREAD);
task task_dataSend(TASK_DATASEND);
task task_CheckRespons(60000);
task task_ESPupdate(TASK_ESPUPDATE);
task task_resetDHT(DHTRESETTIME);


struct Light
{
	bool light1;
	bool light2;
	bool night;
	bool automatedOperation=true;
	void check(int detectedSun) {
		if (automatedOperation) {
			if (!night) { //day routine
				if (light1 && light2) { 
					if (detectedSun < SUNTOONLIGHT - SUNDIFFERENCE) { // idealy shoule never get here
						light1 = false; light2 = false;
					}
				}
				else { 
					if (detectedSun > SUNTOONLIGHT) { //switch on in the evening
						light1 = true; light2 = true;
					}
				}
			}
			else { // night routene
				if (man.invalidData) {
					if (light1 && light2) {
						light1 = false; light2 = false; // night time switch off
					}
				}
				else {
					if (!light1 && !light2) {
						light1 = true; light2 = true; // if human detected light on
					}
				}
			}
		}
		if (hour >= HOURNIGHT || hour<= HOURDAY) night = true;
		else night = false;
		if (light1) digitalWrite(RELAYLIGH1, LOW);
		else digitalWrite(RELAYLIGH1, HIGH);
		if (light2) digitalWrite(RELAYLIGH2, LOW);
		else digitalWrite(RELAYLIGH2, HIGH);
	}
	
	void set(int index) { //set 0 to switch both off, 1: 1-on 2-off, 2: 1-off, 1-on, 3: both on, 3- automated loght oparation
							//4-automated operation, 5-night mode; 6-day mode
		switch (index) {
		case 0: light1 = false; light2 = false; automatedOperation = false; break;
		case 1: light1 = true; light2 = false; automatedOperation = false; break;
		case 2: light1 = false; light2 = true; automatedOperation = false; break;
		case 3: light1 = true; light2 = true; automatedOperation = false; break;
		case 4: automatedOperation = true; break;
		case 5: night = true; automatedOperation = true; break;
		case 6: night = false;  automatedOperation = true; break;
		default:break;
		}
		
	}
} light;

class Fan {
	bool oneTimeStart;
public:
	bool on;
	bool switchedOnTemp;
	bool switchedOnAir;
	bool switchedOnHumid;
	bool pauseOn;
	unsigned long setTimeToWork;
	unsigned long pauseStart;
	unsigned long lastCheck;
	void stop() {
		on = false;
		pauseOn = false;
		oneTimeStart = false;
		display(7);
		setTimeToWork = 0;
		pauseStart = 0;
		lastCheck = 0;
		digitalWrite(RELAYFAN, LOW);
		blind.set(2);//set automated operation
	}
	bool start(unsigned long time = FANTIMETOWORK) {
		if (!on) {
			if (!switchedOnTemp && !switchedOnAir && !switchedOnHumid) oneTimeStart = true;//set TRUE in case if the start() is called by telegram
			on = true;
			pauseOn = false;
			setTimeToWork = time;
			lastCheck = millis();
			display(7);
			blind.set(1);//focre open
			digitalWrite(RELAYFAN, HIGH);
			return true;
		}
		return false;
	}
	void pauseActivate() {
		pauseStart = millis();
		pauseOn = true;
		on = false;
		digitalWrite(RELAYFAN, LOW);
		setTimeToWork = 0;
		display(7);
	}
	void check() {
		if (on) {
			if (millis() - lastCheck > setTimeToWork) {
				if (!switchedOnTemp && !switchedOnAir && !switchedOnHumid) stop();
				else pauseActivate();
			}
		}
		if (pauseOn) {
			if (millis() - pauseStart > FANPAUSE)start();
		}
		if (pauseOn && !switchedOnTemp && !switchedOnAir && !switchedOnHumid) stop();//complite stop if while pause fan is deactivated
		if (!pauseOn) {
			if (switchedOnTemp || switchedOnAir || switchedOnHumid) start();//start by switchedOn
		}
		if (on && !oneTimeStart && !switchedOnTemp && !switchedOnAir && !switchedOnHumid) stop();
	}
} fan;
void display(int index=0) {
	String line;
	if (!man.displayUpdate || index == 7) {
		if (fan.on) line = "WORK ";
		else if(fan.pauseOn) line = "PAUS ";
		else line = "STOP ";
		if (blind.open) line.concat("OP ");
		else line.concat("CL ");
		if (man.detected && !man.invalidData) line.concat("MAN");
		else line.concat("NON");
		const char* lineptr = line.c_str();
		u8x8.drawString(0, 6, lineptr);
		man.displayUpdate = true;
	}
	if (!DHTInDoor.displayUpdate) {
		line = DHTInDoor;
		const char* lineptr = line.c_str();
		u8x8.drawString(0, 0, lineptr);
		DHTInDoor.displayUpdate = true;
	}
	if (!DHTOutDoor.displayUpdate) {
		line = DHTOutDoor;
		const char* lineptr = line.c_str();
		DHTOutDoor.displayUpdate = true;
		u8x8.drawString(0, 1, lineptr);
		}
	if (!air.displayUpdate || !sun.displayUpdate) {
		line = "ai:" + String(air.value) + " su:" + String(sun.value)+"  ";
		air.displayUpdate = true;
		sun.displayUpdate = true;
		const char* lineptr = line.c_str();
		u8x8.drawString(0, 2, lineptr);
	}
	if (index == 3) {
		if (ServerConected) line = "Server OK ";
		else line = "Server NOK";
		const char* lineptr = line.c_str();
		u8x8.drawString(0, 3, lineptr);
	}
	if (index == 4) {
		if (DHTInDoor.invalidData) {
			line = "tIN NaN";
			if (DHTOutDoor.invalidData) line.concat("tOUTNaN");
			} 
		else if(DHTOutDoor.invalidData) line="tOUTNaN       ";
		if (!DHTOutDoor.invalidData && !DHTInDoor.invalidData) line = "              ";
		const char* lineptr = line.c_str();
		u8x8.drawString(0, 4, lineptr);
	}
	if (index == 6) {
		line = String(hour)+":"+String(min);
		if (light.night)line += " NIGHT"; 
		else line += " DAY  ";
		const char* lineptr = line.c_str();
		u8x8.drawString(0, 5, lineptr);
	}
	sensor::overallDisplUpdate = true;

}

void readsensors() {
	indoorDHT22.read();
	outdoorDHT22.read();
	DHTInDoor.update(indoorDHT22.readTemperature(), indoorDHT22.readHumidity());
	DHTOutDoor.update(outdoorDHT22.readTemperature(), outdoorDHT22.readHumidity());
	if (DHTInDoor.invalidData || DHTOutDoor.invalidData)resetDHT();
	sun.update( analogRead(SUNSENSOR));
	air.update(analogRead(MQ135));
	++sensor::dataReadCounter;
}
void resetDHT(bool action) {
	if (!action) {
		digitalWrite(DHTRESETRELAY, LOW);
		task_resetDHT.ignor = false;
	}
	else {
		digitalWrite(DHTRESETRELAY, HIGH);
		task_resetDHT.ignor = true;
	}
	display(4);
}
void sendDataToServer(int index) {
	if (!index) {
		String LogString = "Device:1;get:3;tempInDoor,humidInDoor,tempOutDoor,humidOurDoor,sunlight,air,state;";
		Serial.println(LogString);
		return;
	}
	byte state = B00000000;// construction of state
	if (!man.invalidData)bitWrite(state, 0, 1);
	if (fan.on)bitWrite(state, 1, 1);
	if (blind.open)bitWrite(state, 2, 1);
	if (blind.automatedOperation)bitWrite(state, 3, 1);
	if (light.light1)bitWrite(state, 4, 1);
	if (light.light2)bitWrite(state, 5, 1);
	if (light.light1)bitWrite(state, 6, 1);
	if (light.light1)bitWrite(state, 7, 1);
	if (index == 2) Serial.println("temp:" + String(DHTInDoor.temp) + ";humid:" + String(DHTInDoor.humid) +
		+";air:" + String(air.value) + ";status:" + String(state) + ";"); //send data to ESP
	else {
		DHTInDoor.updateBuf();
		DHTOutDoor.updateBuf();
		sun.updateBuf();
		air.updateBuf();
		String data2Send = "Device:1;get:2;";
		Serial.println(data2Send + String(DHTInDoor.tempBuf) + "," + String(DHTInDoor.humidBuf) + "," +
			String(DHTOutDoor.tempBuf) + "," + String(DHTOutDoor.humidBuf) + "," + String(sun.valueBuf) + "," +
			String(air.valueBuf) + "," + String(state) + ";"); //to server

		DHTInDoor.resetBuf();
		DHTOutDoor.resetBuf();
		sun.resetBuf();
		air.resetBuf();
	}
}
void ReadDataSerial() { 
	unsigned long temp;
	int  index = 0;
	String message;
	while (Serial.available()) {
		message = Serial.readStringUntil('\r');
		Serial.println(message);
		if (get_field_value(message, "connected:", &temp, &index)) {
			task_CheckRespons.reLoop();
			ServerConected = true;
			display(3);
		}
		else if (get_field_value(message, "hour:", &temp, &index)) {
			hour = (int)temp;
			get_field_value(message, "min:", &temp, &index);
			min = (int)temp;
			display(6);
		}
		else if (get_field_value(message, "tempDifoff:", &temp, &index))tempDifoff = int(temp);
		else if (get_field_value(message, "tempDifon:", &temp, &index))tempDifon = int(temp);
		else if (get_field_value(message, "humidon:", &temp, &index))humidon = int(temp);
		else if (get_field_value(message, "humidoff:", &temp, &index))humidoff = int(temp);
		else if (get_field_value(message, "airDifoff:", &temp, &index))airDifoff = int(temp);
		else if (get_field_value(message, "airDifon:", &temp, &index))airDifon = int(temp);
		else if (get_field_value(message, "getsetting:", &temp, &index))sendSettings();
		else if (get_field_value(message, "fanstart:", &temp, &index))fan.start(temp);//temp set time to work miliss
		else if (get_field_value(message, "lightset:", &temp, &index))light.set(int(temp)); //set 0 to switch both off, 1: 1-on 2-off, 2: 1-off, 1-on, 3: both on, 3- automated loght oparation
																		//4-automated operation, 5-night mode; 6-day mode
		else if (get_field_value(message, "venetionBlind:", &temp, &index)) blind.set(int(temp));// set 0 to force close, 1-force open, 2-automated oparation
		else if (get_field_value(message, "blindSetValue:", &temp, &index)) blind.setToValue(int(temp));// 
		else if (get_field_value(message, "getLogString:", &temp, &index))sendDataToServer(0);//send LogString
	
	}
}
void sendSettings() {
	String data2Send = "Device:1;settings:;tempDifoff:";
	data2Send.concat(tempDifoff);
	data2Send.concat(";tempDifon:");
	data2Send.concat(tempDifon);
	data2Send.concat(";humidon:");
	data2Send.concat(humidon);
	data2Send.concat(";humidoff:");
	data2Send.concat(humidoff);
	data2Send.concat(";airDifoff:");
	data2Send.concat(airDifoff);
	data2Send.concat(";airDifon:");
	data2Send.concat(airDifon);
	data2Send.concat(";");
	Serial.println(data2Send);
}
bool get_field_value(String Message, String field, unsigned long* value, int* index) {
	int fieldBegin = Message.indexOf(field) + field.length();
	int check_field = Message.indexOf(field);
	int ii = 0;
	*value = 0;
	*index = 0;
	bool indFloat = false;
	if (check_field != -1) {
		int filedEnd = Message.indexOf(';', fieldBegin);
		if (filedEnd == -1) { return false; }
		int i = 1;
		char ch = Message[filedEnd - i];
		while (ch != ' ' && ch != ':') {
			if (isDigit(ch)) {
				int val = ch - 48;
				if (!indFloat)ii = i - 1;
				else ii = i - 2;
				*value = *value + ((val * pow(10, ii)));
			}
			else if (ch == '.') { *index = i - 1; indFloat = true; }
			i++;
			if (i > (filedEnd - fieldBegin + 1) || i > 10)break;
			ch = Message[filedEnd - i];
		}

	}
	else return false;
	return true;
}
void setup() {
	Serial.begin(9600);
	indoorDHT22.begin();
	outdoorDHT22.begin();
	Serial.println("Start");
	myservo.attach(SERVODATA);
	pinMode(RELAYFAN, OUTPUT);
	pinMode(RELAYLIGH1, OUTPUT);
	pinMode(RELAYLIGH2, OUTPUT);
	pinMode(RELAYSERVO, OUTPUT);
	pinMode(DHTRESETRELAY, OUTPUT);
	digitalWrite(DHTRESETRELAY, HIGH);
	digitalWrite(RELAYFAN, LOW);
	digitalWrite(RELAYLIGH1, HIGH);
	digitalWrite(RELAYLIGH2, HIGH);
	digitalWrite(RELAYSERVO, HIGH);
	pinMode(HUMANSENSOR, INPUT);
	pinMode(SUNSENSOR, INPUT);
	pinMode(MQ135, INPUT);
	u8x8.begin();
	u8x8.setFont(u8x8_font_chroma48medium8_r);
	blind.closeBlind();
	task_resetDHT.ignor = true;
}

void loop() {
	if (!sensor::overallDisplUpdate) display();
	if (task_SensorRead.check()) {
		readsensors();
		light.check(sun.value);

	}
	if(task_ESPupdate.check())   sendDataToServer(2);
	if (task_dataSend.check()) sendDataToServer(1);
	if (Serial.available()>0)ReadDataSerial();
	man.update(digitalRead(HUMANSENSOR));
	blind.check();
	fan.check();
	if (DHTInDoor.temp > DHTOutDoor.temp) {
		if (DHTInDoor.temp > TEMPTHRESHOLD+ tempDifon)fan.switchedOnTemp = true;
	}
	if (fan.switchedOnTemp) {
		if(DHTInDoor.temp < TEMPTHRESHOLD + tempDifoff) fan.switchedOnTemp = false;
	}
	if (DHTInDoor.humid > DHTOutDoor.humid) {
		if (DHTInDoor.humid > HUMIDTHRESHOLD + humidon)fan.switchedOnHumid = true;
	}
	if (fan.switchedOnHumid) {
		if (DHTInDoor.humid < HUMIDTHRESHOLD + humidoff) fan.switchedOnHumid = false;
	}
	if (DHTInDoor.temp > BLINDTHRESHOLD+1)blind.blindTempToOpen=true;
	else if (DHTInDoor.temp < BLINDTHRESHOLD )blind.blindTempToOpen = false;
	if (task_CheckRespons.check()) {
		ServerConected = false;
		display(3);
	}
	if (task_resetDHT.check())resetDHT(true);
}

