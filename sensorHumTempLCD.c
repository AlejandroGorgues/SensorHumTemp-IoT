/**
 * Links: 
 *      https://github.com/karlrupp/i2cHoneywellHumidity/blob/master/i2cHoneywellHumidity.c
 *      https://poesiabinaria.net/2012/06/obtener-la-fecha-y-hora-formateada-en-c/
 *      http://rants.dyer.com.hk/rpi/humidity_i2c.html
 *		
 * Conexion del Sensor HIH al Raspberry Pi
 *   SENSOR       RASPBERRY
 * VDD(1)    -->    3V3 (NumPin: 17)
 * VSS(2)    -->    GND (NumPin: 9)
 * SCL(3)    -->    GPIO3 (NumPin: 5)
 * SDA(4)    -->	GPIO2 (NumPin: 3)
 *
 * Compilar: gcc -Wall -o <nombre> <nombre>.c -lwiringPi
 * Ejecutar: sudo ./<nombre>
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
time_t t;
struct tm *time;
int hourMinutes;
int seconds;
char dateTime[100];
int tempMedia = 27;


int reading_temp;
int reading_hum;
double temperature;
double humidity;



//This a thread for writing to the METER(hum) and ANGULAR_METER(temp).
static void *handleHumidityTemperature(void *data)
{

    //write to METER
    genieWriteObj(GENIE_OBJ_METER, 0x00, (int)humidity);	
    genieWriteObj(GENIE_OBJ_ANGULAR_METER, 0x00, (int)temperature);	
  return NULL;
}

//This a thread for writing to the USER_LED
static void *handleThreshold(void *data)
{

  //write to USER_LED
  genieWriteObj(GENIE_OBJ_USER_LED, 0x00, (int)tempMedia);	
  return NULL;
}

//This a thread for writing to the LED_DIGITS
static void *handleHour(void *data)
{

  //write to LED_DIGITS

  genieWriteObj(GENIE_OBJ_LED_DIGITS, 0x01, hourMinutes);	
  genieWriteObj(GENIE_OBJ_LED_DIGITS, 0x02, seconds);	
  return NULL;
}




//This is the event handler. Messages received from the display
//are processed here.
void handleTempThresholdEvent(struct genieReplyStruct * reply)
{
  if(reply->cmd == GENIE_REPORT_EVENT)    //check if the cmd byte is a report event
  {
    if(reply->object == GENIE_OBJ_KNOB) //check if the object byte is that of a slider
      {
        if(reply->index == 0)		  //check if the index byte is that of KNOB

        	tempMedia = (int)reply->data		
      }
  }
  
  //if the received message is not a report event, print a message on the terminal window
  else
    printf("Unhandled event: command: %2d, object: %2d, index: %d, data: %d \r\n", reply->cmd, reply->object, reply->index, reply->data);
}

/**
 * Define el estado de los PINs como salida.
 */
void init()
{
	wiringPiSetup();
	
	pinMode(LRED, OUTPUT);
	pinMode(RGBGREEN, OUTPUT);
	pinMode(RGBBLUE, OUTPUT);
	pinMode(RGBRED, OUTPUT);
}

/*
* Concatena dos int
*/
unsigned concatenateInt(unsigned x, unsigned y) {
    unsigned pow = 10;
    while(y >= pow)
        pow *= 10;
    return x * pow + y;        
}

/**
 * Imprime la fecha y hora del sistema
 */
void imprimirDateTime(pthread_t *myThread)
{
	string hourMinutes;
	t = time(NULL);	
	time = localtime(&_t);
 
    hourMinutes = concatenateInt(tm_struct->tm_hour, tm_struct->tm_min)
	seconds = tm_struct->tm_sec;

	strftime(dateTime,100,"%d/%m/%Y %H:%M:%S", time);
	printf ("%s\n", dateTime);
	timeString = dateTime;

	//start the thread to write hour and seconds to led digits
	(void)pthread_create (myThread,  NULL, handleHour, NULL); 

}

/**
 * Evalua el valor de la Temperatura y actualiza el estado del LED 
 */
void procesarTemperatura(int temperatureC, pthread_t *myThread)
{
	if( temperatureC >= tempMedia)
	{ //start the thread for writing to the USER_LED	
		(void)pthread_create (myThread,  NULL, handleThreshold, NULL); 
		digitalWrite(LRED,HIGH);
  	}
	else digitalWrite(LRED,LOW);
}

/**
 * Evalua el valor de la Humedad y actualiza el estado del actuador RGB
 */
void procesarHumedad(int humidityC)
{
	if( humidityC < RMINHUM ) // RED ON
	{
		digitalWrite(RGBRED,HIGH);
		digitalWrite(RGBGREEN,LOW);
		digitalWrite(RGBBLUE,LOW);
	}else if( humidityC >= RMINHUM && humidityC <= RMAXHUM ){ // GREEN ON
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
	pthread_t myThread;              //declare a thread
  	struct genieReplyStruct reply ;  //declare a genieReplyStruct type structure

  	//open the Raspberry Pi's onboard serial port, baud rate is 115200
    //make sure that the display module has the same baud rate
  	genieSetup("/dev/serial0", 115200);  

	init();

	// Open port (r/w)
	if ((fileDescriptor = open(fileNamePort, O_RDWR)) < 0)
	{
		printf("Error al abrir el puerto I2C\n");
		exit(1);
	}

	// Set port options and slave devie address
	if (ioctl(fileDescriptor, I2C_SLAVE, address) < 0)
	{
		printf("Unable to get bus access to talk to slave\n");
		exit(1);
	}

	for(;;)
	{
		// Initiate measurement by sending a zero bit (see datasheet for communication pattern)
		if ( (write(fileDescriptor,buffer,0)) != 0 )
		{
			printf("Error writing bit to i2c slave\n");
			exit(1);
		}

		//Wait for 100ms for measurement to complete. Typical measurement cycle is 36.65ms for each of humidity and temperature, so you may reduce this to 74ms.
		usleep(100000);
		
		imprimirDateTime(&myThread);
		
		
		// read back data
		if (read(fileDescriptor, buffer, 4) < 0)
		{
			printf("No se puede leer los datos del esclavo\n");
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
				


			    string tempAux = "Temperatura%s: %.1f\n", "(C)", temperature;
			    string humAux = "Humedad%s: %.1f\n\n", "(%)", humidity

				printf(tempAux);
				printf(humAux);

				genieWriteStr(0x00, tempAux + "\n" + humAux);
				
				procesarTemperatura((int)temperature, &myThread);
				procesarHumedad((int)humidity);



				//start the thread for writing to the METER and ANGULAR_METER	
  				(void)pthread_create (&myThread,  NULL, handleHumidityTemperature, NULL);
				

				 while(genieReplyAvail())      //check if a message is available
			    {
			      genieGetReply(&reply);      //take out a message from the events buffer
			      handleTempThresholdEvent(&reply);   //call the event handler to process the message
			    }	
			}else 
				printf("Error, el Estado es != 0\n");
		}
		delay(3000);
	}

	return 0;
}