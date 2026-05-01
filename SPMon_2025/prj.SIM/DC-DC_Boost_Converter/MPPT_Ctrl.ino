/* --- Pin Definition --- */
#define PIN_CONV_IN_U A1 
#define PIN_CONV_IN_I A0   
#define PIN_CONV_OUT_U A3
#define PIN_CONV_OUT_I A2
#define MPPT_LED_PIN 7
#define PIN_PWM_BOOST 9  // PWM controlled using Timer1

/* --- Calib Values --- */
#define DIV_RATIO    8.0 
#define V_REF        5.0          
#define ADC_RES      1024  
#define ACS717_SENS  0.185
#define V_ZERO_RAW   512         
#define MAX_PWM_T1   319.0

/* ---  MPPT Constants (0-319 -> 10% = 31.9) --- */
int D_val               = 127;  //  40% Duty Cycle                
const uint8_t D_MIN     = 32;   //  10% Duty Cycle         
const uint8_t D_MAX     = 159;  //  50& Duty Cycle      
const uint8_t MPPT_PACE =   5;  //  (5/3)% Duty Cylce  for perturabtion and adjustments
#define ERROR_MARGIN      0.20  //  Hysteresis set to 0.2W

#define MPPT_FACTORS_NUM      3
#define MPPT_P_PV_INPUT_INDEX 0
#define MPPT_U_PV_INPUT_INDEX 1
#define MPPT_I_PV_INPUT_INDEX 2
float MPPT_Measurements[MPPT_FACTORS_NUM];

volatile float      Last_P_PV   = 0.0;  // Last power value 
volatile float      Last_U_PV   = 0.0;  // Last voltage value  
volatile float      Last_I_PV   = 0.0;  // Last current value    
         
volatile float      delta_P_PV  = 0.0;  // Power change
volatile float      delta_U_PV	= 0.0;	// Volatge change
volatile float		delta_I_PV	= 0.0;	// Current change

volatile int8_t    PO_Dir      = 0;    // Perturbation direction +-1 

typedef enum {
	ERROR   = -1,
	STABLE = 0,
	HI_POW,
	LO_POW,
	DRIFT_FREE
}MPPT_STATES;

volatile uint8_t MPPT_ActState = STABLE;

/* --- Converter Output Measurements --- */
#define MPPT_CONV_OUTPUT_MEAS_NUM 3
#define MPPT_CONV_P_OUTPUT_INDEX 0
#define MPPT_CONV_U_OUTPUT_INDEX 1
#define MPPT_CONV_I_OUTPUT_INDEX 2
float MPPT_ConvOutMeasurements[MPPT_CONV_OUTPUT_MEAS_NUM];

/* --- Timers --- */
unsigned long previousMillis_Measure = 0;
unsigned long previousMillis_MPPT = 0;

const long IntervalMillis_Measure = 1;
const long IntervalMillis_MPPT = 5;

/* --- Function prototypes --- */
void setup();
void loop();

void Timer1_Config();         
void MPPT_GetConverterInputPower();
void MPPT_GetConverterOutputPower();
void MMPT_PerturbAndObserve();

/* --- Setup --- */
void setup() 
{
	//Serial.begin(9600);
	pinMode(PIN_PWM_BOOST, OUTPUT);
	pinMode(MPPT_LED_PIN, OUTPUT);
	digitalWrite(MPPT_LED_PIN, LOW);

	/* ################################################# */
	/* ### CONFIGURE TIMER1 (PIN 9) FOR 50 kHz       ### */
	/* ################################################# */
	Timer1_Config();

}

// --- Loop ---
void loop() 
{
		
	unsigned long now = millis();

	if (now - previousMillis_Measure > IntervalMillis_Measure)
	{
		/* Measure POWER, VOLATGE & CURRENT values at the output of the PV_Panel */
		MPPT_GetConverterInputPower();
		/* Measure POWER, VOLATGE & CURRENT values at the output of the Converter */
		MPPT_GetConverterOutputPower();
		previousMillis_Measure = now;
	}
	if (now - previousMillis_MPPT > IntervalMillis_MPPT)
	{
		/* Perturb & Observe Algortihm (P&O) to keep MPPT */
		MMPT_PerturbAndObserve();
		previousMillis_MPPT= now;
	}

}

void Timer1_Config()
{
	/* 1. Modul de Operare (Fast PWM, TOP = ICR1) */
	TCCR1A = (1 << WGM11);
	TCCR1B = (1 << WGM12) | (1 << WGM13);
	// Aceste linii definesc modul de operare al Timer1 ca fiind "Fast PWM Mode 14".
	// Acest mod foloseste registrul ICR1 pentru a seta valoarea de VARF (TOP) a numaratorului.

	/* 2. Setarea Frecventei la 50 kHz (Definirea Varfului) */
	ICR1 = 319;
	// ICR1 (Input Capture Register) seteaza valoarea MAXIMA (TOP) a numaratorului Timer1.
	// Frecventa PWM (f_PWM) = f_CPU / (N * (TOP + 1))
	// Cu f_CPU=16MHz si Prescaler N=1: 50,000 Hz = 16,000,000 / (1 * (319 + 1))
	// Deci, 319 asigura frecventa exacta de 50 kHz.

	// 3. Setarea Prescaler-ului (Viteza de Numarare)
	TCCR1B |= (1 << CS10);
	// CS10 (Clock Select Bit 0) este setat, iar celelalte (CS11, CS12) sunt zero. 
	// Acest lucru seteaza prescaler-ul Timer1 la N=1, adica Timer1 numara cu frecventa completa a CPU (16 MHz).
	// Folosim "|=" pentru a seta bitul CS10 fara a afecta ceilalti biti din TCCR1B.

	// 4. Configurare Pin Iesire (Pinul 9 - OC1A)
	TCCR1A |= (1 << COM1A1);
	// COM1A1 = 1 ?i COM1A0 = 0 (setate implicit la 0) definesc modul de iesire.
	// Aceasta combinatie seteaza Pinul 9 (OC1A) în modul "Non-Inverting PWM":
	// - Iesirea este HIGH când Timer1 numara de la 0 la OCR1A.
	// - Iesirea este LOW când Timer1 numara de la OCR1A la TOP (ICR1).

	// 5. Aplicarea Duty Cycle-ului
	OCR1A = D_val;
	// OCR1A (Output Compare Register A) este registrul care defineste Duty Cycle-ul.
	// Daca D_val este 127 (din 319), Duty Cycle-ul este 96/319 ˜ 40%.
	// Timer1 compara OCR1A cu valoarea numarata pentru a comuta Pinul 9.
	// Aceasta linie înlocuieste functia standard analogWrite().
}

void MPPT_GetConverterInputPower() 
{
	/* Get Input voltage U_PV*/
	int adcRawValueU = analogRead(PIN_CONV_IN_U);
	float adcRawU = (float)adcRawValueU * V_REF / ADC_RES;
	float V_PV = adcRawU * DIV_RATIO; 

	/* Get Input Current I_PV*/
	int adcRawValueI = analogRead(PIN_CONV_IN_I);
	int offset_current_raw = adcRawValueI - V_ZERO_RAW;
	float I_PV = ((float)offset_current_raw * V_REF) / (ADC_RES * ACS717_SENS); 

	/* Calculate P_PV */
	float P_PV = V_PV * I_PV;

	/* Save values in the arra*/
	MPPT_Measurements[MPPT_P_PV_INPUT_INDEX] = P_PV;
	MPPT_Measurements[MPPT_U_PV_INPUT_INDEX] = V_PV;
	MPPT_Measurements[MPPT_I_PV_INPUT_INDEX] = I_PV;

	/* Calculate current duty cycle */
	float D_percent = ((float)D_val / MAX_PWM_T1) * 100.0;

}

void MPPT_GetConverterOutputPower()
{
	/* Get Output voltage U_PV*/
	int adcRawValueU = analogRead(PIN_CONV_OUT_U);
	float adcRawU = (float)adcRawValueU * V_REF / ADC_RES;
	float Vout = adcRawU * DIV_RATIO; 

	/* Get Input Current I_PV*/
	int adcRawValueI = analogRead(PIN_CONV_OUT_I);
	int offset_current_raw = adcRawValueI - V_ZERO_RAW;
	float Iout = ((float)offset_current_raw * V_REF) / (ADC_RES * ACS717_SENS); 

	/* Calculate P_PV */
	float Pout = Vout * Iout;

	/* Save values in the arra*/
	MPPT_ConvOutMeasurements[MPPT_CONV_P_OUTPUT_INDEX] = Pout;
	MPPT_ConvOutMeasurements[MPPT_CONV_U_OUTPUT_INDEX] = Vout;
	MPPT_ConvOutMeasurements[MPPT_CONV_I_OUTPUT_INDEX] = Iout;

}

void MMPT_PerturbAndObserve()
{
	digitalWrite(MPPT_LED_PIN, HIGH);

	/* Get current POWER, VOLATGE & CURRENT values from PV output */
	float Current_P_PV = MPPT_Measurements[MPPT_P_PV_INPUT_INDEX];
	float Current_U_PV = MPPT_Measurements[MPPT_U_PV_INPUT_INDEX];
	float Current_I_PV = MPPT_Measurements[MPPT_I_PV_INPUT_INDEX];

	/* Calculate power change (Delta P_PV) */
	delta_P_PV = Current_P_PV - Last_P_PV;
	/* Calculate voltage change (Delta V_PV) */
	delta_U_PV = Current_U_PV - Last_U_PV;
	/* Calculate current change (Delta I_PV) */
	delta_I_PV = Current_I_PV - Last_I_PV;

	/* Set PO_Dir before switch */
	int8_t next_PO_Dir = PO_Dir; 

	/* 2. Compare last and current power values and change main state*/
	if (delta_P_PV > ERROR_MARGIN) 
	{ 
		MPPT_ActState = HI_POW;
	} 
	else if (delta_P_PV < (-ERROR_MARGIN)) 
	{ 
		MPPT_ActState = LO_POW;
	} 
	else 
	{
		MPPT_ActState = STABLE;
	}

	/* Check each state and make decison for increasing or decreasing PWM dutycycle*/
	switch(MPPT_ActState)
	{
		/* P_PV~, U_PV~, I_PV~ => PWM~ */
		case STABLE:
			/* Do nothing */
			break;
		
		/* P_PV+*/
		case  HI_POW:
			/* P_PV+, U_PV+  */
			if (delta_U_PV > ERROR_MARGIN)
			{
				/* Algo eneters DRIFT FREE LOOP */
				MPPT_ActState = DRIFT_FREE;
				
				/* P_PV+, U_PV+ , I_PV+ => PWM+ */
				if (delta_I_PV > ERROR_MARGIN && MPPT_ActState == DRIFT_FREE)
				{
					next_PO_Dir = 1;
				}
				/* P_PV+, U_PV+ , I_PV- => PWM- */
				else if(delta_I_PV < ERROR_MARGIN && MPPT_ActState == DRIFT_FREE)
				{
					next_PO_Dir = -1;
				}
				else
				{
					MPPT_ActState == ERROR;
					next_PO_Dir = 0;
				}
			}
			/* P_PV+, U_PV+ , I_PV~ => PWM+ */
			else if (delta_U_PV < (-ERROR_MARGIN))
			{
				next_PO_Dir = 1;
			}
			break;

		case LO_POW:
			/* P_PV-, U_PV+ => PWM+ */
			if (delta_U_PV > ERROR_MARGIN)
			{
				next_PO_Dir = 1;
			}
			/* P_PV-, U_PV- => PWM- */
			else if (delta_U_PV < (-ERROR_MARGIN))
			{
				next_PO_Dir = -1;
			}
			break;

		default:
			break;
	}

	/* Update global PO_Dir with the decision in switch case */
	PO_Dir = next_PO_Dir;
	
	/* Set duty cycle according to state machine decision */
	D_val = D_val + (PO_Dir * MPPT_PACE);

	/* Keep Duty Cycle within the limits */
    if (D_val > D_MAX) 
	{
        D_val = D_MAX;
        PO_Dir = -1; 
    } 
	else if (D_val < D_MIN) 
	{
        D_val = D_MIN;
        PO_Dir = 1; 
    }

	/* Apply duty cycle on Timer1 Register */
	OCR1A = D_val;

	/* Set last power value */
	Last_P_PV = Current_P_PV;
	/* Set last voltage value */
	Last_U_PV = Current_U_PV;
	/* Set last current value */
	Last_I_PV = Current_I_PV;

	digitalWrite(MPPT_LED_PIN, LOW);
}

