#ifndef PIPGLOBALDEFINES_H
#define PIPGLOBALDEFINES_H

//sleep period at module startup
#define STARTUP_SLEEP_TIME 20000

//NOP time per loop
#define RELAX_TIME 100

//master clock rate at module startup
#define INITIAL_CYCLE_PERIOD 1000

//time division between channels A and B at module startup
#define INITIAL_DIV_VAL 1000

//length of the output buffer in samples per channel
#define OUTPUT_BUF_LEN 4

//number of output channels
#define NUM_OUTPUTS 2

//maximum output value
#define OUTVAL_MAX 255

//midpoint (bias) for output values
#define OUTVAL_MID 127

//maximum overall input value
#define INPUT_MAX 255

//maximum input value for pots
//needed since the inputs have pots as well as buttons connected
//the DIV pin is also used for output of the SYNC LED flash (time-multiplexed)
#define INPUT_POT_MAX ((uint16_t) 200)

//threshold for input value above which a button press is registered (for PG buttons)
#define INPUT_BTN_THRESHOLD_NORMAL 230

//threshold for input value above which a button press is registered (for REC button)
#define INPUT_BTN_THRESHOLD_REC 210

//threshold for input value below which a button press is registered (for SYNC button)
#define INPUT_BTN_THRESHOLD_SYNC 237

//time threshold above which a long button press is registered
#define BTN_LONG_PRESS_TIME 200

//counterclockwise position deadzone for the DIV pot
#define INPUT_POT_DIV_DEADZONE_CCW 35

//clockwise position deadzone for the DIV pot
#define INPUT_POT_DIV_DEADZONE_CW 220

//moving deadzone around current DIV pot value in normal mode
#define DIV_POT_DEADZONE 19

//count of DIV value change confirmations needed in order to switch to a new DIV ratio
#define DIV_DELTA_SMOOTHING 60

//internal resolution of the waveform offset
#define OFFSET_RESOLUTION ((uint32_t) 0x10000)

//predefined value to indicate that the SYNC tap button is currently not monitored
#define SYNC_COUNT_INACTIVE ((uint32_t) 0xffffffff)

//maximum SYNC tap time
#define SYNC_COUNT_MAX ((uint32_t) 60000)

//minimum SYNC tap time
#define SYNC_COUNT_MIN ((uint32_t) 25)

//maximum time (in main loops) that an external SYNC pulse may deviate from the internal clock
//used to trigger creation of a new S&H sample at cycle boundaries
#define SYNC_PRECISION_SH 10

//relative time expressed in (1/SYNC_PRECISION_PULSE) cycle periods that an external SYNC pulse may deviate from the internal clock
//used to align the internal clock phase to the pulse
#define SYNC_PRECISION_PULSE 10

//the SYNC LED will flash for (SYNC_LED_TIMER - SYNC_LED_MARGIN) time units
#define SYNC_LED_TIMER 25

//time that the pin is not to be used as input following the end of a SYNC LED flash
#define SYNC_LED_MARGIN 20

//length of the recording buffer per channel, in samples
#define RECORDING_BUF_LEN 128

//number of different DIV values in normal mode, as indicated on the front panel
#define DIV_TABLE_COUNT 5

//resolution of the DIV values table
#define DIV_TABLE_RESOLUTION 1000

//maximum product of DIV ratio dividend and divisor (3/2 -> 3*2)
#define DIV_TABLE_MAX_PRODUCT 6

//table containing the frequency division values selectable with the DIV knob
//values are stored multiplied by DIV_TABLE_RESOLUTION
const uint16_t divtable[DIV_TABLE_COUNT] = { 250, 500, 1000, 1500, 2000 };

//pseudo-random number generator seed values
#define PRNG_SEED1 314159265
#define PRNG_SEED2 271828182

//special value that refers to the S&H/S&G page
#define WT_PAGE_SPECIAL_RANDOM WT_PAGES

//predefined values to distinguish the various DIV modes
#define DIV_MODE_NORMAL 0
#define DIV_MODE_FREE 1
#define DIV_MODE_PHASE 2

//predefined values that identify input pins
#define PIN_IN_DIV_REC 0
#define PIN_IN_CTRL_PAGE1 1
#define PIN_IN_CTRL_PAGE2 2
#define PIN_IN_SYNC 3
#define NUM_PINS 4

//mapping of input ports to pin IDs
const int8_t ADCmap[] = { -1, -1, 1, 3, 2, 0 };

#endif //PIPGLOBALDEFINES_H
