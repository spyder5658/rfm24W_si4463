#ifndef _MORSE_H_
#define _MORSE_H_

#include "main.h"
#include "RH_RF24.h"


#define MINI_MORSE_DIT_TIME    1200/20


typedef enum 
{
  DIT,
  DAH
}MorseLetter;

void mini_morse_tx(const char* msg, const uint16_t len);
char character_to_code(const char x);
void mini_morse_encode_char(const char x);
void radio_init_morse(void);
void mini_morse_start_tx(const  MorseLetter m);
void mini_morse_stop_tx(const MorseLetter m);
void mini_morse_delay(const uint16_t delay);








#endif