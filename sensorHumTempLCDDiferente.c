/**
 *
 * Recordatorio
 * SENSOR           RASPBERRY
 * VDD(1)    -->    3V3 (NumPin: 17)
 * VSS(2)    -->    GND (NumPin: 9)
 * SCL(3)    -->    GPIO3 (NumPin: 5)
 * SDA(4)    -->	GPIO2 (NumPin: 3)
 *
 * BAUDIOS   -->    115200
 *
 */

 //Sensores
#include <stdio.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <time.h>
#include <wiringPi.h>
#include <softPwm.h>
#include <math.h>

//LCD
#include <ctype.h>
#include <errno.h>
#include <unistd.h>
#include <pthread.h>

#include <geniePi.h>  //the ViSi-Genie-RaspPi library

#define BAUDURATE 115200 // Must be the same as the program

#define HUMMIN 40.0
#define HUMMAX 70.0

#define LRED 1 // GPIO18 (NumPin: 12)

#define RGBGREEN 4 // GPIO23 (NumPin: 16)
#define RGBBLUE 5 // GPIO24 (NumPin: 18)
#define RGBRED 6 // GPIO25 (NumPin: 22)

int fileDescriptor; // File descriptor
const char *fileNamePort = "/dev/i2c-1"; // Name of the port we will be using. On Raspberry 2 this is i2c-1, on an older Raspberry Pi 1 this might be i2c-0.
int  address = 0x27; // Address of Honeywell sensor shifted right 1 bit
unsigned char buffer[4]; // Buffer for data read/written on the i2c bus

//Time
time_t t;
struct tm *tm;
char dateTime;
int timeConcatenated;

//Genie
char geineData;

//Temp and Hum
int tempThreshold = 27;
int reading_temp;
int reading_hum;
double temperature;
double humidity;


//Concatena dos enteros
int concatenate(int x, int y){
    int pow = 10;

    while (y >= pow) pow *= 10;
    return x * pow + y;

}


//This a thread for writing to the METER(hum) and ANGULAR_METER(temp)
static void *handleHumidityTemperature(void *data)
{

	//write to METER, ANGULAR_METER and USER_LED
    for(;;){
        
        genieWriteObj(GENIE_OBJ_METER, 0x00, (int)humidity);
        genieWriteObj(GENIE_OBJ_ANGULAR_METER, 0x00, (int)temperature);
        if((int)temperature >= tempThreshold){
             genieWriteObj(GENIE_OBJ_USER_LED, 0x00, 1);
        }else{
        	genieWriteObj(GENIE_OBJ_USER_LED, 0x00, 0);
        }
    }
  return NULL;
}

//This is the event handler. Messages received from the display are processed here.
void handleTempThresholdEvent(struct genieReplyStruct * reply)
{
  if(reply->cmd == GENIE_REPORT_EVENT)    //check if the cmd byte is a report event
  {
    if(reply->object == GENIE_OBJ_KNOB) //check if the object byte is that of a slider
      {
        if(reply->index == 0)		  //check if the index byte is that of KNOB

        	tempThreshold = (int)reply->data;
      }
  }

  //if the received message is not a report event, print a message on the terminal window
  else
    printf("Evento no manejado: command: %2d, object: %2d, index: %d, data: %d \r\n", reply->cmd, reply->object, reply->index, reply->data);
}

//Inicializa la salida de los leds y escribe en el LED_DIGITS y en el KNOB
void inicializarComponentes()
{
	wiringPiSetup();

	pinMode(LRED, OUTPUT);
	pinMode(RGBGREEN, OUTPUT);
	pinMode(RGBBLUE, OUTPUT);
	pinMode(RGBRED, OUTPUT);

	genieWriteObj(GENIE_OBJ_LED_DIGITS, 0x00, tempThreshold); 
	genieWriteObj(GENIE_OBJ_KNOB, 0x00, tempThreshold); 
}

//Muestra el tiempo por consola y por los LED_DIGITS
void procesarTiempo(){
		t = time(NULL);
        tm = localtime(&t);

	    strftime(dateTime,100,"%d/%m/%Y %H:%M:%S", tm);
		printf ("%s\n", dateTime);

		timeConcatenated = concatenate(tm->tm_hour, tm->tm_min);
		genieWriteObj(GENIE_OBJ_LED_DIGITS, 0x01, timeConcatenated);
		genieWriteObj(GENIE_OBJ_LED_DIGITS, 0x02, tm->tm_sec);
}

//Evalua el valor de la Temperatura y actualiza el estado del USER_LED y del ANGULAR_METER
void procesarTemperatura(int temperatureC)
{

	genieWriteObj(GENIE_OBJ_ANGULAR_METER, 0x00, (int)temperature);

	if( temperatureC >= tempThreshold)
	{ //start the thread for writing to the USER_LED
		digitalWrite(LRED,HIGH);
		genieWriteObj(GENIE_OBJ_USER_LED, 0x00, 1);
  	}else{
  		digitalWrite(LRED,LOW);
		genieWriteObj(GENIE_OBJ_USER_LED, 0x00, 0);
  	} 
}

//Evalua el valor de la Humedad y actualiza el estado del actuador RGB y del METER
void procesarHumedad(int humidityC)
{

	genieWriteObj(GENIE_OBJ_METER, 0x00, (int)humidity);

	if( humidityC < HUMMIN ) // RED ON
	{
		digitalWrite(RGBRED,HIGH);
		digitalWrite(RGBGREEN,LOW);
		digitalWrite(RGBBLUE,LOW);
	}else if( humidityC >= HUMMIN && humidityC <= HUMMAX ){ // GREEN ON
		digitalWrite(RGBGREEN,HIGH);
		digitalWrite(RGBRED,LOW);
		digitalWrite(RGBBLUE,LOW);
	}else{ // BLUE ON
		digitalWrite(RGBBLUE,HIGH);
		digitalWrite(RGBRED,LOW);
		digitalWrite(RGBGREEN,LOW);
	}
}



int main(int argc, char **argv)
{
	pthread_t myThread;              //Declare a thread
  	struct genieReplyStruct reply ;  //Declare a genieReplyStruct type structure

  	//open the Raspberry Pi's onboard serial port with the baudrate
  	genieSetup("/dev/serial0", BAUDURATE);

	inicializarComponentes();

	// Open port (r/w)
	if ((fileDescriptor = open(fileNamePort, O_RDWR)) < 0)
	{
		printf("Error al abrir el puerto I2C\n");
		exit(1);
	}

	// Set port options and slave devie address
	if (ioctl(fileDescriptor, I2C_SLAVE, address) < 0)
	{
		printf("El bus no tiene acceso para hablar con el esclavo\n");
		exit(1);
	}

	//start the thread for writing to the METER, USER_LED and ANGULAR_METER
  	(void)pthread_create (&myThread,  NULL, handleHumidityTemperature, NULL);

	for(;;)
	{
		// Initiate measurement by sending a zero bit (see datasheet for communication pattern)
		if ( (write(fileDescriptor,buffer,0)) != 0 )
		{
			printf("Error escribiendo el bit al esclavo i2c\n");
			exit(1);
		}

		//Show the current time
		procesarTiempo()

		//Wait for 100ms for measurement to complete. Typical measurement cycle is 36.65ms for each of humidity and temperature, so you may reduce this to 74ms.
		usleep(100000);

		// read back data
		if (read(fileDescriptor, buffer, 4) < 0)
		{
			printf("No se pueden leer los datos del esclavo\n");
			exit(1);
		}
		else
		{
			if((buffer[0] & 0xC0) == 0) // Verify state sensor is 0 making AND with 11000000
			{
				// Humidity is located in first two bytes
				reading_hum = (buffer[0] << 8) + buffer[1];
				humidity =reading_hum / 16382.0 * 100.0;

				// Temperature is located in next two bytes, padded by two trailing bits
				reading_temp = (buffer[2] << 6) + (buffer[3] >> 2);
				temperature = reading_temp / 16382.0 * 165.0 - 40;

				sprintf(genieData, "Temperatura (ºC): %.1f\nHumedad (%): %.1f", temperature, humidity);

                printf("Temperatura%s: %.1f\n", temperature);
                printf("Humedad%s: %.1f\n\n", humidity);

				genieWriteStr(0x00, genieData);//Write temp and hum to the screen

				procesarTemperatura((int)temperature);
				procesarHumedad((int)humidity);


                while(genieReplyAvail())      //check if a message is available
			    {
			      genieGetReply(&reply);      //take out a message from the events buffer
			      handleTempThresholdEvent(&reply);   //call the event handler to process the message
			    }
			}else
				printf("Error, el estado es != 0\n");
		}
	}

	return 0;
}
