//defines.h#define PWM_FAKTOR            1.1#define K_DELTA               .10#define K_PROP                3.0#define K_PROP_RED            1.2#define K_PROP_LO             1.8#define K_PROP_HI             4.0#define K_INT                 0.5#define K_DIFF                0.1#define K_WINDUP_HI           40#define K_WINDUP_LO           -10#define TIMER2_ENDWERT        0x9E // OV 20ms#define TIMER2_COMPA          0x9E // OV 20ms#define TIMER2_PWM_INTERVALL  0xFF // Paketlaenge#define OSZIPORT				PORTB#define OSZIDDR            DDRB#define OSZIA					1#define OSZILO             OSZIPORT &= ~(1<<OSZIA)#define OSZIHI             OSZIPORT |= (1<<OSZIA)#define OSZITOGG           OSZIPORT ^= (1<<OSZIA)#define LOOPLEDPORT        PORTB#define LOOPLEDDDR         DDRB#define LOOPSTEP           0x0004// Define fuer Slave:#define TOPLED_PIN			1// Blinkt waehrend heizen, voll wenn Temp erreicht#define LOOPLED_PIN			0//#define TOPLED           1 #define ADCPORT PORTB#define ADCDDR    DDRB#define ADC_IST_KANAL      1#define ADC_SOLL_KANAL     2#define ADC_SOLL_PIN       4 // von Einstellung Temperatur#define ADC_IST_PIN        2 // von PTC#define OUTPORT            PORTB#define OUTDDR             DDRB#define PWM_OUT_PIN        3 // Pin fuer Mosfet#define PWM_ON             0 // Bit fuer PWM-ON#define PWM_ADC            1#define PID_FIRST_RUN      2 // Bit fuer first run//avr-size  --mcu=attiny85 -C Laminator.elf