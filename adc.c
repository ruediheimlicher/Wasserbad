/*
 *  adc.c
 *  TWI_Master
 *
 *  Created by Sysadmin on 12.11.07.
 *  Copyright 2007 Ruedi Heimlicher. All rights reserved.
 *
 */

#include "adc.h"
#include <avr/io.h>


struct adcwert16 ADCWert16;

void initADC(uint8_t derKanal)
{
   ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS0);    // Frequenzvorteiler auf 32 setzen und ADC aktivieren 
 
  ADMUX = derKanal;                      // �bergebenen Kanal waehlen
//	REFS0 und REFS1 LO: Externe Referenzspannung
//	ADMUX |= (1<<REFS1) | (1<<REFS0); // interne Referenzspannung nutzen 
//	ADMUX |= (1<<REFS0); // VCC als Referenzspannung nutzen
 
  /* nach Aktivieren des ADC wird ein "Dummy-Readout" empfohlen, man liest
     also einen Wert und verwirft diesen, um den ADC "warmlaufen zu lassen" */
  ADCSRA |= (1<<ADSC);              // eine ADC-Wandlung (Der ADC setzt dieses Bit ja wieder auf 0 nach dem Wandeln)
  while ( ADCSRA & (1<<ADSC) )
  {
     ;     // auf Abschluss der Wandlung warten 
  }
}

uint16_t readKanal(uint8_t derKanal) //Unsere Funktion zum ADC-Channel aus lesen
{
  uint8_t i;
  uint16_t result = 0;         //Initialisieren wichtig, da lokale Variablen
                               //nicht automatisch initialisiert werden und
                               //zuf�llige Werte haben. Sonst kann Quatsch rauskommen
 ADMUX = (derKanal&0x07);
   // VCC
   
   
   // interne Referenz 2.56V
   ADMUX |= (1<<REFS2);
   ADMUX |= (1<<REFS1);

//ADMUX |= (1<<REFS0);
  // Eigentliche Messung - Mittelwert aus 4 aufeinanderfolgenden Wandlungen
  for(i=0;i<4;i++)
  {
    ADCSRA |= (1<<ADSC);            // eine Wandlung
    while ( ADCSRA & (1<<ADSC) ) {
      ;     // auf Abschluss der Wandlung warten 
    }
    result += ADCW;            // Wandlungsergebnisse aufaddieren
  }
//  ADCSRA &= ~(1<<ADEN);             // ADC deaktivieren ("Enable-Bit" auf LOW setzen)
 
  result /= 4;                     // Summe durch vier teilen = arithm. Mittelwert
 
  return result;
}

void closeADC()
{
ADCSRA &= ~(1<<ADEN);             // ADC deaktivieren ("Enable-Bit" auf LOW setzen)
}

/*
uint16_t readKanalOrig(uint8_t derKanal, uint8_t num) //Unsere Funktion zum ADC-Channel aus lesen
{
  uint8_t i;
  uint16_t result = 0;         //Initialisieren wichtig, da lokale Variablen
                               //nicht automatisch initialisiert werden und
                               //zuf�llige Werte haben. Sonst kann Quatsch rauskommen
 
   ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS0);    // Frequenzvorteiler auf 32 setzen und ADC aktivieren 
 
  ADMUX = derKanal;                      // �bergebenen Kanal waehlen
//  ADMUX |= (1<<REFS1) | (1<<REFS0); // interne Referenzspannung nutzen 
  ADMUX |= (1<<REFS0); // VCC als Referenzspannung nutzen 
 
  // nach Aktivieren des ADC wird ein "Dummy-Readout" empfohlen, man liest
     also einen Wert und verwirft diesen, um den ADC "warmlaufen zu lassen" 
  ADCSRA |= (1<<ADSC);              // eine ADC-Wandlung (Der ADC setzt dieses Bit ja wieder auf 0 nach dem Wandeln)
  while ( ADCSRA & (1<<ADSC) ) {
     ;     // auf Abschluss der Wandlung warten 
  }
 
  // Eigentliche Messung - Mittelwert aus 4 aufeinanderfolgenden Wandlungen
  for(i=0;i<4;i++)
  {
    ADCSRA |= (1<<ADSC);            // eine Wandlung
    while ( ADCSRA & (1<<ADSC) ) {
      ;     // auf Abschluss der Wandlung warten 
    }
    result += ADCW;            // Wandlungsergebnisse aufaddieren
  }
  ADCSRA &= ~(1<<ADEN);             // ADC deaktivieren ("Enable-Bit" auf LOW setzen)
 
  result /= 4;                     // Summe durch vier teilen = arithm. Mittelwert
 
  return result;
}
*/