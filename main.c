#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "main.h"
#include "com_def.h"
// #include "uart.h"
#include "ultrasonic_trigger_echo.h"
#include "LCD.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <stdlib.h>


#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1  // Calculate UBRR value for 16 MHz

#define BUFFER_SIZE 128


// GPS Global variables
char rx_buffer[BUFFER_SIZE]; // Buffer to store incoming GPS data
uint8_t rx_index = 0;
uint8_t sentence_ready = 0;

// Ultrasonic global variables
char data_buffer[4],buffer[50];

int distance = 0,c_m=0,m_m=0;


// Function Prototypes
void USART_Init(unsigned int ubrr);
void USART_Transmit(char data);
char USART_Receive(void);
void USART_SendString(const char *str);
void parse_gprmc(char *nmea_sentence, char *latitude, char *longitude);



ISR(USART_RXC_vect) {
	char received_char = UDR; // Read received character
	if (received_char == '\n' || rx_index >= BUFFER_SIZE - 1) {
		rx_buffer[rx_index] = '\0'; // Null-terminate the string
		sentence_ready = 1;
		rx_index = 0; // Reset index for next sentence
		} else {
		rx_buffer[rx_index++] = received_char; // Store received character
	}
}




void USART_Init(unsigned int ubrr) {
	UBRRH = (unsigned char)(ubrr >> 8);
	UBRRL = (unsigned char)ubrr;
	UCSRB = (1 << RXEN) | (1 << TXEN) | (1 << RXCIE); // Enable RX, TX, and RX interrupt
	UCSRC = (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1); // 8 data bits, 1 stop bit
	sei(); // Enable global interrupts
}

void USART_Transmit(char data) {
	while (!(UCSRA & (1 << UDRE))); // Wait for empty transmit buffer
	UDR = data; // Send data
}

void USART_SendString(const char *str) {
	while (*str) {
		USART_Transmit(*str++);
	}
}

void parse_gprmc(char *nmea_sentence, char *latitude, char *longitude) {
	char *token;
	uint8_t field_count = 0;

	token = strtok(nmea_sentence, ",");
	while (token != NULL) {
		field_count++;
		if (field_count == 3) {
			strcpy(latitude, token); // Latitude
			} else if (field_count == 5) {
			strcpy(longitude, token); // Longitude
		}
		token = strtok(NULL, ",");
	}
}


int convert_cm(int value)
{
 c_m = value/10;
 //m_m = value%10;


  integer_to_char(c_m,data_buffer,3);
  //send2uart(data_buffer);
  /*send2uart(".");
  integer_to_char(m_m,data_buffer,2);
  usart_putchar(data_buffer[1]);*/
  //send2uart("cm");
  //usart_putchar(0x0D);
  //usart_putchar(0x0A);
  
  
  if(c_m<015)
  {
	// Print on LCD
	char lcd_buff[16];
	size_t len = sprintf(lcd_buff, "Dis = %d cm", c_m);
	LCD4_gotoxy(1,1);
	LCD4_write_string(lcd_buff);
	
    sprintf(buffer,"OBSTACLE DETECTED\n");
	
	PORTA |= (1 << 1);
	//send2uart(buffer);
  }
  else
  {
	  LCD4_gotoxy(1,1);
	  LCD4_write_string("Safe Zone");
	  PORTA &= ~(1 << 1);
  }
}


int main(void)
{
	char latitude[16], longitude[16];
	USART_Init(MYUBRR);
	
	_delay_ms(1000);
	print_uart("OK");
	
	// send char
	DDRB = 0xFF;
	DDRC = 0xFF;
	LCD4_init();
	LCD4_gotoxy(1,1);
	LCD4_write_string("Mustafa");
	LCD4_gotoxy(2, 9);
	LCD4_write_string("Refaat");
	
	DDRB&=~(BIT3);
	PORTB |= BIT3;

	DDRA |= (1 << 1);
	PORTA &= ~(1 << 1);

	init_Ultrasonic_sensor();
	
	sei();
	
	 char formatted_output[17]; // 16 characters + 1 null terminator


	while(1)
	{
		
		if (sentence_ready) {
			sentence_ready = 0; // Reset flag

				if (strncmp(rx_buffer, "$GPRMC", 6) == 0) { // Check for $GPRMC sentence
					parse_gprmc(rx_buffer, latitude, longitude);

					// Format the output string
					sprintf(formatted_output, "%.7sL%.7sN", latitude, longitude);

					// Send the formatted string
					LCD4_gotoxy(2,1);
					LCD4_write_string(formatted_output);
			}
		}
		
		distance = get_distance_Ultrasonic_sensor();
		convert_cm(distance);
		_delay_ms(500);
	}

}