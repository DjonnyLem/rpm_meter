/* 
 avr-gcc -Wall -g -Os -mmcu=atmega328p -o main.o main.c lcd1602.c i2c.c &&  avr-objcopy -j .text -j .data -O ihex main.o main.hex &&  avrdude -p atmega328p -c usbasp -U flash:w:main.hex:i -F -P usb

 avr-gcc -Wall -g -Os -mmcu=atmega328 -o main.o main.c
 avr-objcopy -j .text -j .data -O ihex main.o main.hex
 avrdude -p atmega328 -c usbasp -U flash:w:main.hex:i -F -P usb

 avr-gcc -mmcu=atmega328 -Wall -Os -o  main.elf  main.c 
 avr-objcopy -O ihex  main.elf  main.hex
 avrdude -p atmega328 -c usbasp -v -P /dev/ttyS0 -b9600 -U flash:w:main.hex


 avr-objdump -m avr -D ./main.hex //диассамблирование
 avr-objdump -m avr -S ./main.elf > main.asm //диассамблирование
 avr-objdump -m avr -S ./main.elf  //диассамблирование
*/

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include "lcd1602.h"
#include <util/delay.h>

#define COUNT_TIME 156 //значение в регистр совпадения таймера (10мс)

#define PIN_LED_BLUE PC3 //A3
#define PIN_LED_RED  PC2 //A2
#define PIN_LED_GREEN PC1 //A1
#define PIN_LED_BUILTIN PB5 //13 LED_BUILTIN

#define valueCnt 50
#define valueCnt1 25
#define valueCnt2 15
/* Битовые операции */
#define   SetBit(reg, bit)           reg = reg | (1<<bit)
#define   ClearBit(reg, bit)         reg = reg & (~(1<<bit))
#define   InvBit(reg, bit)           reg ^= (1<<bit)
#define   BitIsSet(reg, bit)       ((reg &  (1<<bit)) != 0)
#define   BitIsClear(reg, bit)     ((reg &  (1<<bit)) == 0)


volatile unsigned char	static flag; 
#define flagCycle 0 // флаг устанавливается в прерывании = 1 каждые 10мс
#define flag1 1 // 
#define flagMagic 2
uint8_t cnt = valueCnt;
uint8_t cnt1 = valueCnt1;
uint8_t	cnt2 = valueCnt2;
uint8_t led = 0;

const uint8_t magic[]={
	0b00110011,
	0b00111111,
	0b00000011
};

	uint8_t simA[48]={
		
		0b00000011,
		0b00000111,
		0b00001111,
		0b00011111,
		0b00011111,
		0b00011111,
		0b00011111,
		0b00011111,

		0b00011111,
		0b00011111,
		0b00011111,
		0b00011111,
		0b00000000,
		0b00000000,
		0b00000000,
		0b00000000,

		0b00011111,
		0b00011111,
		0b00011111,
		0b00011111,
		0b00011111,
		0b00011111,
		0b00011111,
		0b00011111,

		0b00011111,
		0b00011111,
		0b00011111,
		0b00011111,
		0b00011111,
		0b00011111,
		0b00011111,
		0b00011111,

		0b00011111,
		0b00011111,
		0b00011111,
		0b00000000,
		0b00000000,
		0b00000000,
		0b00000000,
		0b00000000,

		0b00011111,
		0b00011111,
		0b00011111,
		0b00011111,
		0b00011111,
		0b00011111,
		0b00011111,
		0b00011111
	};
	

uint8_t simR[48]={
	0b00011111,
	0b00011111,
	0b00011111,
	0b00011111,
	0b00011111,
	0b00011111,
	0b00011111,
	0b00011111,
	
	0b00011111,
	0b00011111,
	0b00011111,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	
	0b00011000,
	0b00011100,
	0b00011110,
	0b00011111,
	0b00011111,
	0b00011111,
	0b00011111,
	0b00011111,
	
	
	
	0b00011111,
	0b00011111,
	0b00011111,
	0b00011111,
	0b00011111,
	0b00011111,
	0b00011111,
	0b00011111,
	
	0b00011111,
	0b00011111,
	0b00011111,
	0b00001111,
	0b00000111,
	0b00000011,
	0b00000001,
	0b00000000,
	
	0b00011110,
	0b00011100,
	0b00011000,
	0b00010000,
	0b00011000,
	0b00011100,
	0b00011110,
	0b00011111	
	
	};

//-----Прототипы функций----------------------------
void timer_init();
void port_init();
void blink();


//-----END Прототипы функций----------------------------


//-----Прерывания----------------------------
ISR(TIMER1_COMPA_vect)  {
	
	SetBit(flag, flagCycle);					// установить флаг цикла 10ms
	


}

//-----END Прерывания----------------------------


int main() {
	lcd1602_init();
	timer_init();
	port_init();
	lcd1602_clear();
	sei(); // включить глобальные прерывания
	lcd1602_goto_xy(0, 0);
	lcd1602_send_string("Hello world");
	lcd1602_goto_xy(0,  1);
	lcd1602_send_string("I2C");
	_delay_ms(3000);
	lcd1602_clear();
	
	void A (void){
	lcd1602_send_byte(0b01000000, LCD_COMMAND);
	//_delay_us(40);
	
	for (uint8_t i=0; i<48; i++)
	{
		lcd1602_send_byte(simA[i], LCD_DATA);
	}
	lcd1602_send_byte(0b10000000, LCD_DATA);	
	lcd1602_goto_xy(1, 0);
	//lcd1602_send_char(0b00000000);
	//lcd1602_send_char('D');
	lcd1602_send_byte(0b00000000, LCD_DATA);
	lcd1602_send_byte(0b00000001, LCD_DATA);
	lcd1602_send_byte(0b00000010, LCD_DATA);
	lcd1602_goto_xy(1, 1);
	lcd1602_send_byte(0b00000011, LCD_DATA);
	lcd1602_send_byte(0b00000100, LCD_DATA);
	lcd1602_send_byte(0b00000101, LCD_DATA);
};
	
	void R (void){
	lcd1602_send_byte(0b01000000, LCD_COMMAND);
	//_delay_us(40);
	
	for (uint8_t i=0; i<48; i++)
	{
		lcd1602_send_byte(simR[i], LCD_DATA);
	}
	lcd1602_send_byte(0b10000000, LCD_DATA);	
	lcd1602_goto_xy(5, 0);
	//lcd1602_send_char(0b00000000);
	//lcd1602_send_char('D');
	lcd1602_send_byte(0b00000000, LCD_DATA);
	lcd1602_send_byte(0b00000001, LCD_DATA);
	lcd1602_send_byte(0b00000010, LCD_DATA);
	lcd1602_goto_xy(5, 1);
	lcd1602_send_byte(0b00000011, LCD_DATA);
	lcd1602_send_byte(0b00000100, LCD_DATA);
	lcd1602_send_byte(0b00000101, LCD_DATA);
	};
	sei();	// включить глобальные прерывания
	while(1) {
		A();
		lcd1602_clear();
		//_delay_ms(30);
		R();

	}
}


//-----Функции----------------------------


void timer_init() {// инициализация Timer1
/*
  частота 16МГц  1сек/16000000Гц*1024 = 0,000064 сек/такт
  0,001сек = 1мс= 0,001/0,000064 = 16 значение в регистр совпадения
  0,010сек = 10мс=0,010/0,000064 = 156 значение в регистр совпадения
*/

  cli(); // отключить глобальные прерывания
  TCCR1A = 0; // установить регистры в 0
  TCCR1B = 0;

  OCR1A = COUNT_TIME; // установка регистра совпадения
  TCCR1B |= (1 << WGM12); // включение в CTC режим

  // Установка битов CS10 и CS12 на коэффициент деления 1024
  TCCR1B |= (1 << CS10);
  TCCR1B |= (1 << CS12);

  TIMSK1 |= (1 << OCIE1A);  // включение прерываний по совпадению
  
}

void port_init() //инициализация портов
{

  DDRC |= (1 << PIN_LED_BLUE | 1 << PIN_LED_RED | 1 << PIN_LED_GREEN | 1 << PIN_LED_BUILTIN);
  PORTC &= ~(1 << PIN_LED_BLUE | 1 << PIN_LED_RED | 1 << PIN_LED_GREEN | 1 << PIN_LED_BUILTIN); //В ПОРТАХ 0
}

void blink()
{

}	


