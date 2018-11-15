

 //Sensores
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <time.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <wiringPi.h>
#include <softPwm.h>
#include <linux/i2c-dev.h>

//LCD
#include <ctype.h>
#include <pthread.h>
#include <errno.h>
#include <unistd.h>

#include <geniePi.h>

int fileDescriptor;
const char *fileNamePort = "/dev/i2c-1"; // Nombre del puerto que usaremos. En rapsberri 3 puede ser 0 o 1, aunque es normalmente 1
int  address = 0x27; // Dirección donde se conectará el sensor
unsigned char buffer[4]; // Buffer para escribir y obtener los datos del sensor a partir del bus i2c

//Time
time_t t;
struct tm *tm;
char dateTime[100];
int timeConcatenated;

//Genie
char genieData[100] = {};

//Temp y Hum
double temperature;
double humidity;
int reading_temp;
int reading_hum;

int tempThreshold = 27;


//Concatena dos enteros
int concatenate(int x, int y){
    int pow = 10;

    while (y >= pow) pow *= 10;
    return x * pow + y;

}


//Thread para escribir al METER(hum) y al ANGULAR_METER(temp)
static void *handleHumidityTemperature(void *data)
{

	//Escribe al METER, ANGULAR_METER y al USER_LED
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

//Se maneja el evento. Los mensajes recibidos del display se manejan aqui.
void handleTempThresholdEvent(struct genieReplyStruct * reply)
{
  if(reply->cmd == GENIE_REPORT_EVENT)    //Comprueba si el byte de cmd es de tipo report event
  {
    if(reply->object == GENIE_OBJ_KNOB) //Comprueba si el byte de object es del knob
      {
        if(reply->index == 0)		  //Comprueba si el byte del index es el correspondiente

        	tempThreshold = (int)reply->data;
      }
  }

  //Si se recibe un mensaje que on es un report event, imprime el mensaje por consola
  else
    printf("Evento no manejado: command: %2d, object: %2d, index: %d, data: %d \r\n", reply->cmd, reply->object, reply->index, reply->data);
}

//Inicializa la salida de los leds y escribe en el LED_DIGITS y en el KNOB
void inicializarPines()
{
	wiringPiSetup();

	pinMode(1, OUTPUT);
	pinMode(4, OUTPUT);
	pinMode(5, OUTPUT);
	pinMode(6, OUTPUT);

}

void procesarTiempo(){
		t = time(NULL);
        	tm = localtime(&t);

        strftime(dateTime,100,"%d/%m/%Y %H:%M:%S", tm);
		printf ("%s\n", dateTime);

		timeConcatenated = concatenate(tm->tm_hour, tm->tm_min);
		genieWriteObj(GENIE_OBJ_LED_DIGITS, 0x01, timeConcatenated);
		genieWriteObj(GENIE_OBJ_LED_DIGITS, 0x02, tm->tm_sec);
}

void procesarTemperatura(int temperatureC)
{

	if( temperatureC >= tempThreshold)
	{
		digitalWrite(1,HIGH);
		genieWriteObj(GENIE_OBJ_USER_LED, 0x00, 1);
  	}else{
  		digitalWrite(1,LOW);
		genieWriteObj(GENIE_OBJ_USER_LED, 0x00, 0);
  	}
}


void procesarHumedad(int humidityC)
{

	if( humidityC < 40.0 )
	{
		digitalWrite(6,HIGH);
		digitalWrite(4,LOW);
		digitalWrite(5,LOW);
	}else if( humidityC >= 40.0 && humidityC <= 70.0 ){
		digitalWrite(4,HIGH);
		digitalWrite(6,LOW);
		digitalWrite(5,LOW);
	}else{
		digitalWrite(5,HIGH);
		digitalWrite(6,LOW);
		digitalWrite(4,LOW);
	}
}



int main(int argc, char **argv)
{
	pthread_t myThread;              //Declara el thread
  	struct genieReplyStruct reply ;  //Declara la estructura de tipo genieReplyStruct

  	//Abre el puerto seria de la raspberri con el baudrate correspondiente
  	genieSetup("/dev/serial0", 115200);

	inicializarPines();

	// Abre el puerto de lectura y escritura del i2c
	if ((fileDescriptor = open(fileNamePort, O_RDWR)) < 0)
	{
		printf("Error al abrir el puerto I2C\n");
		exit(1);
	}

	// Ajusta las opciones del puerto y la dirección del dispositivo del esclavo
	if (ioctl(fileDescriptor, I2C_SLAVE, address) < 0)
	{
		printf("El bus no tiene acceso para hablar con el esclavo\n");
		exit(1);
	}

	//Crea el thread para escribir al METER, USER_LED y al ANGULAR_METER
  	(void)pthread_create (&myThread,  NULL, handleHumidityTemperature, NULL);

	for(;;)
	{
		// Comienza a tomar medidas del sensor enviando un bit de cero
		if ( (write(fileDescriptor,buffer,0)) != 0 )
		{
			printf("Error escribiendo el bit al esclavo i2c\n");
			exit(1);
		}

		//Muestra el tiempo actual
		procesarTiempo();

		// Espera durante 100ms para completar la medida.El tiempo de espera medio para el de temperatura y humedad es de 36.65ms, es decir, que en total seería alrededor de 74ms.
		usleep(100000);

		// Lee los datos obtenidos en el buffer
		if (read(fileDescriptor, buffer, 4) < 0)
		{
			printf("No se pueden leer los datos del esclavo\n");
			exit(1);
		}
		else
		{
			if((buffer[0] & 0xC0) == 0) // Comprueba el estado del sensor es 0 haciendo una AND con 11000000
			{
				// La humedad está en los dos primeros bytes
				reading_hum = (buffer[0] << 8) + buffer[1];
				humidity =reading_hum / 16382.0 * 100.0;

				// La temperatura está en los dos siguientes bytes
				reading_temp = (buffer[2] << 6) + (buffer[3] >> 2);
				temperature = reading_temp / 16382.0 * 165.0 - 40;

				sprintf(genieData, "Temperatura (ºC): %.1f\nHumedad (%%): %.1f", temperature, humidity);

                printf("Temperatura: %.1f\n", temperature);
                printf("Humedad: %.1f\n\n", humidity);

				genieWriteStr(0x00, genieData);//Escribe la humedad y la temperatura al display

				procesarTemperatura((int)temperature);
				procesarHumedad((int)humidity);


                while(genieReplyAvail())      //Comprueba si hay algún mensaje del display
			    {
			      genieGetReply(&reply);      //Saca el mensaje del display
			      handleTempThresholdEvent(&reply);   //Llama al manejador de eventos para procesar el mensaje
			    }
			}else
				printf("Error, el estado es != 0\n");
		}
	}

	return 0;
}
