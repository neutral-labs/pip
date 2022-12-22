#include "wavetable.h"
#include "pipGlobalDefines.h"

#define _NOP() do { __asm__ __volatile__ ("nop"); } while (0)

//dual CV output buffer
struct rwBuffer
{
  //output values
  uint8_t val[NUM_OUTPUTS][OUTPUT_BUF_LEN];
  
  //current level
  uint8_t lvl = 0;

  //read index (bitfield so it will wrap automatically)
  uint8_t rd:2;

  //write index (bitfield so it will wrap automatically)
  uint8_t wr:2;
};

volatile struct rwBuffer outBuf;

//CV recording buffer
struct recordingBuffer
{
  //values
  uint8_t val[NUM_OUTPUTS][RECORDING_BUF_LEN + 1];

  //currently recording channel
  uint8_t channel;

  //true if channel has a recording stored
  bool recorded[NUM_OUTPUTS];

  //true if channel is currently recording
  bool recording[NUM_OUTPUTS];
};

struct recordingBuffer recBuf;

//input buffer, contains raw values from input pins
volatile uint8_t inVal[4] = { 0, 0, 0, 0 };

//current offset within the waveform cycle
uint32_t offset[NUM_OUTPUTS] = { 0, 0 };

//cycle period that the waveforms are synced to
uint32_t period = INITIAL_CYCLE_PERIOD;

//counter for SYNC cycle period, inactive at startup (until SYNC button press or incoming SYNC signal)
uint32_t syncCount = SYNC_COUNT_INACTIVE;

//start at 1/1 div ratio
uint32_t div_val = INITIAL_DIV_VAL;

//timer to distinguish between short and long button presses
uint32_t buttonTimer[3] = { 0, 0, 0 };

//random values for the S&H/S&G page
uint32_t randomValCurrent[NUM_OUTPUTS], randomValNext[NUM_OUTPUTS];

//variables for the DIV pot state
uint16_t savedDivFreeModeVal = 1;
uint8_t savedDivPotVal = 1, savedDivPhaseModeVal = 1, savedDivIndex = 0, divIndexDeltaCount = 0;

//counter to determine the length of the SYNC LED flash and subsequent disabling of the input pin
uint8_t syncLEDcount = 0;

//counter to identify external SYNC pulses that are late/early compared to the internal clock
uint8_t syncPrecisionCountdown = SYNC_PRECISION_SH;

//current wavetable page per output, initialised to the random pages
uint8_t wt_page[NUM_OUTPUTS] = { WT_PAGE_SPECIAL_RANDOM, WT_PAGE_SPECIAL_RANDOM };

//stores the last read value per pot, which is used in case the pot cannot currently be read
//input pins are used for buttons, pots and the SYNC LED simultaneously
//while a button is pressed or the LED is flashing, the pot input cannot be read
uint8_t lastPotValue[NUM_PINS] = { 1, 1, 1, 1 };

//current DIV mode
uint8_t divMode = DIV_MODE_NORMAL;

//current state (pressed/not pressed) per button
bool buttonState[NUM_PINS] = { false, false, false, false };

//true if the SYNC LED has already been checked this loop
bool syncLEDchecked = false;

//selects pin for ADC
static inline void _setADCpin(uint8_t pin)
{
  ADMUX &= 0xe0;
  ADMUX |= ADCmap[pin] & 0x1f;
}

//gets value from ADC
static inline uint8_t _getPinVoltage()
{
  ADCSRA |= (1 << ADSC);  //start conversion
  while (ADCSRA & (1 << ADSC));
  return ADCH;
}

//low level input update function
//fills pin ADC values into buffer
static inline void _inputUpdateRaw()
{
  int i;

  cli();

  for (i = 0; i <= 3; i++)
  {
    _setADCpin(i + 2);
    inVal[i] = _getPinVoltage();
  }

  sei();
}

//converts (scales) pot value from input struct
//identifies and ignores button press action on the same pin if needed
static inline uint8_t _getPotValue(uint8_t pin)
{
  uint8_t val;
  
  if (inVal[pin] > INPUT_POT_MAX)
  {
    if (inVal[pin] > INPUT_BTN_THRESHOLD_NORMAL)
    {
      val = lastPotValue[pin];  
    }
    else
    {
      val = INPUT_MAX;
      lastPotValue[pin] = INPUT_MAX;
    }
  }
  else if (inVal[pin] == 0)
  {
    val = 1;
    lastPotValue[pin] = 1;
  }
  else
  {
    val = (uint8_t)(((uint16_t) inVal[pin] * (uint16_t) INPUT_MAX) / INPUT_POT_MAX);
    lastPotValue[pin] = val;
  }

  return val;
}

//basic pseudo-random number generator
uint32_t _xorshift32(uint32_t val)
{
  val ^= val << 13;
  val ^= val >> 17;
  val ^= val << 5;
  return val;
}

//updates and swaps random values for the S&H/S&G page
static inline void _newRandomValue(uint8_t channel)
{
  randomValCurrent[channel] = randomValNext[channel];
  randomValNext[channel] = _xorshift32(randomValCurrent[channel]);
}

//writes PWM output values
static inline void _setPWMoutput(uint8_t pin, uint8_t value)
{
  if (pin == 0)
  {
    OCR0A = 0xff - value;
  }
  else
  {
    OCR0B = 0xff - value;
  }
}

//handle recording start and stop at sync intervals
static inline void _checkRecording(uint8_t channel)
{ 
  //return if the current cycle wrap isn't occurring on the currently selected recording channel
  if (channel != recBuf.channel)
  {
    return;
  }
  
  //do not record if on channel B while it's on the S&H page
  if (recBuf.channel == 1 && wt_page[1] == WT_PAGE_SPECIAL_RANDOM)
  {
    return;
  }

  //initiate new recording
  if (_isButtonPressed(PIN_IN_DIV_REC))
  {
    recBuf.recording[recBuf.channel] = true;
  }
  else
  { 
    //stop recording that is currently in progress
    if (recBuf.recording[recBuf.channel])
    {
      recBuf.recording[recBuf.channel] = false;
      recBuf.recorded[recBuf.channel] = true;
    }
  }
}

//handle flashing of SYNC LED at SYNC intervals
static inline void _checkSyncLED(bool start)
{
  //we need to start the flash, disable input on the pin and turn on LED
  if (start)
  {
    syncLEDchecked = true;
    syncLEDcount = SYNC_LED_TIMER;
    DDRB |= (1 << PB2);  //set PB2 as output
    PORTB |= (1 << PB2); //set PB2 to HIGH state
  }
  //just decrease countdown
  else if (syncLEDcount > 0)
  {
    syncLEDcount--;    
  }

  //time for the flash is up, turn off the LED
  if (syncLEDcount == SYNC_LED_MARGIN)
  {
    PORTB &= ~(1 << PB2); //set PB2 to LOW state
    DDRB &= ~(1 << PB2);  //set PB2 as input
  }
}

//checks if button is currently pressed
static inline bool _isButtonPressed(uint8_t btn)
{ 
  //SYNC pin handling
  if (btn == PIN_IN_SYNC)
  {
    if (inVal[btn] < INPUT_BTN_THRESHOLD_SYNC)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  //DIV/REC pin handling
  else if (btn == PIN_IN_DIV_REC)
  {
    //LED flash is in progress, cannot use the pin
    //return last stored value instead
    if (syncLEDcount)
    {
      return buttonState[PIN_IN_DIV_REC];
    }

    //button is pressed
    if (inVal[PIN_IN_DIV_REC] >= INPUT_BTN_THRESHOLD_REC)
    {
      buttonState[PIN_IN_DIV_REC] = true;
      return true;
    }
    //button is not pressed
    else
    {
      buttonState[PIN_IN_DIV_REC] = false;
      return false;
    }
  }
  //PG buttons
  else
  {
    //button is pressed
    if (inVal[btn] >= INPUT_BTN_THRESHOLD_NORMAL)
    {
      //increment timer to identify long press
      buttonTimer[btn]++;
      return true;
    }
    //button is not pressed
    else
    {
      return false;
    }
  }
}

//checks if button has been pressed
//will only return true once per button press
static inline bool _wasButtonPressed(uint8_t btn)
{
  if (!buttonState[btn])
  {
    if (_isButtonPressed(btn))
    {
      buttonState[btn] = true;
      return true;
    }
  }
  else
  {
    if (!_isButtonPressed(btn))
    {
      buttonState[btn] = false;
    }
  }

  return false;
}

//checks if button has been pressed and accounts for long presses (special modes)
//will only return true once per button press
static inline bool _wasButtonPressedSpecial(uint8_t btn, bool *longPress)
{
  *longPress = false;

  if (!buttonState[btn])
  {
    if (_isButtonPressed(btn))
    {
      buttonState[btn] = true;
      return false;
    }
  }
  else
  {
    if (!_isButtonPressed(btn))
    {
      buttonState[btn] = false;

      if (buttonTimer[btn] >= BTN_LONG_PRESS_TIME)
      {
        *longPress = true;
      }

      buttonTimer[btn] = 0;
      
      return true;
    }
  }

  return false;
}

//special handling for the DIV pot which is used like a selector switch, if in synced (normal) mode
static inline uint16_t _getDivVal()
{
  int16_t potVal = _getPotValue(PIN_IN_DIV_REC);
  int16_t delta;
  uint8_t idx;

  //we're not in normal mode
  if (divMode != DIV_MODE_NORMAL)
  { 
    //enforce min and max scaled input value thresholds for free and phase mode
    if (potVal > INPUT_POT_DIV_DEADZONE_CCW)
    {
      potVal -= INPUT_POT_DIV_DEADZONE_CCW;
      potVal *= (uint16_t) INPUT_MAX;
      potVal /= (uint16_t) INPUT_POT_DIV_DEADZONE_CW - (uint16_t) INPUT_POT_DIV_DEADZONE_CCW;
  
      if (potVal > INPUT_MAX)
      {
        potVal = INPUT_MAX;
      }
    }
    else
    {
      potVal = 0;
    }

    //free mode
    if (divMode == DIV_MODE_FREE)
    {
      //SYNC LED flash is in progress, cannot use the pin as input during this short time
      if (syncLEDcount)
      {
        return savedDivFreeModeVal;
      }
      //no SYNC flash in progress
      //return value scaled between the highest and lowest DIV table ratio
      else
      {
        return savedDivFreeModeVal = (uint32_t) divtable[0] + ((uint32_t) potVal * ((uint32_t) divtable[DIV_TABLE_COUNT - 1] - (uint32_t) divtable[0])) / INPUT_MAX;
      }
    }
    //phase mode
    else if (divMode == DIV_MODE_PHASE)
    {
      //sync LED is active, cannot use the pin as input during this short time
      if (syncLEDcount)
      {
        return savedDivPhaseModeVal;
      }
      //no SYNC flash in progress
      //return pot value to be used as phase offset
      else
      {
        return savedDivPhaseModeVal = potVal;
      }
    }
  }

  //everything below will run for normal mode only
  delta = potVal - (int16_t) savedDivPotVal;

  //sync LED is active, cannot use the pin as input during this short time
  if (syncLEDcount)
  {
    return divtable[savedDivIndex];
  }

  //prevent unwanted jumps between div ratios by establishing a deadzone around current value
  if (delta < -DIV_POT_DEADZONE || delta > DIV_POT_DEADZONE)
  {
    savedDivPotVal = potVal;
  }
  else
  {
    potVal = savedDivPotVal;
  }

  idx = (DIV_TABLE_COUNT * potVal) / (INPUT_MAX + 1);

  //switch to new div ratio only after measurement from pot has been confirmed DIV_DELTA_SMOOTHING times
  if (savedDivIndex != idx)
  {
    if (++divIndexDeltaCount > DIV_DELTA_SMOOTHING)
    {
      divIndexDeltaCount = 0;
      savedDivIndex = idx;
    }
  }

  return divtable[savedDivIndex];
}

//handle all user input
static inline void _ioUpdate()
{
  bool longPress = false;

  _inputUpdateRaw();

  //do nothing if a CV recording is currently running
  if (recBuf.recording[0] || recBuf.recording[1] || _isButtonPressed(PIN_IN_DIV_REC))
  {
    return;
  }

  //handle SYNC button presses
  if (_wasButtonPressed(PIN_IN_SYNC) && syncCount)
  {
    //got new pulse, align internal clock phase and frequency to pulse
    if (syncCount != SYNC_COUNT_INACTIVE && syncCount > SYNC_COUNT_MIN)
    {
      period = syncCount;
      offset[1] += OFFSET_RESOLUTION / SYNC_PRECISION_PULSE;
      offset[1] -= offset[1] % OFFSET_RESOLUTION;
    }

    syncCount = 0;
  }

  //handle PG A button presses
  if (_wasButtonPressedSpecial(PIN_IN_CTRL_PAGE1, &longPress))
  {
    //long press detected, enter free mode
    if (longPress)
    {
      if (divMode != DIV_MODE_FREE)
      {
        divMode = DIV_MODE_FREE;
      }
      else
      {
        divMode = DIV_MODE_NORMAL;
      }
    }
    //short press detected, end recording and switch to next page
    else
    {
      recBuf.recorded[0] = false;
      recBuf.channel = 0;
    
      if (++wt_page[0] > WT_PAGES)
      {
        wt_page[0] = 0;
      }
    }
  }

  //handle PG B button presses
  if (_wasButtonPressedSpecial(PIN_IN_CTRL_PAGE2, &longPress))
  {
    //long press detected, enter phase mode
    if (longPress)
    {
      if (divMode != DIV_MODE_PHASE)
      {
        divMode = DIV_MODE_PHASE;
      }
      else
      {
        divMode = DIV_MODE_NORMAL;
      }
    }
    //short press detected, end recording and switch to next page
    else
    {
      recBuf.recorded[1] = false;
      recBuf.channel = 1;
      
      if (++wt_page[1] > WT_PAGES)
      {
        wt_page[1] = 0;
      }
    }
  }

  //handle DIV pot input
  div_val = _getDivVal();
}

//fill output sample buffer
static inline void _fillBuffer()
{
  int32_t val1, val2, wt_val1, wt_val2;
  uint32_t offsetDelta, realOffset, oldChannelAOffset, val;
  uint32_t index[NUM_OUTPUTS], fract[NUM_OUTPUTS];
  uint32_t rec_index[NUM_OUTPUTS], rec_fract[NUM_OUTPUTS];
  uint8_t wt_mix[NUM_OUTPUTS];
  uint8_t wt_row[NUM_OUTPUTS];
  uint8_t outPort;

  syncLEDchecked = false;

  //calculate current wavetable rows and morph ratio
  wt_row[0] = ((WT_ROWS - 1) * _getPotValue(PIN_IN_CTRL_PAGE1)) / INPUT_MAX;
  wt_mix[0] = ((WT_ROWS - 1) * _getPotValue(PIN_IN_CTRL_PAGE1)) - (wt_row[0] * INPUT_MAX);
  wt_row[1] = ((WT_ROWS - 1) * _getPotValue(PIN_IN_CTRL_PAGE2)) / INPUT_MAX;
  wt_mix[1] = ((WT_ROWS - 1) * _getPotValue(PIN_IN_CTRL_PAGE2)) - (wt_row[1] * INPUT_MAX);

  //add samples until output buffer is full
  while (outBuf.lvl < OUTPUT_BUF_LEN)
  {
    //increment SYNC period counter if needed
    if (syncCount != SYNC_COUNT_INACTIVE)
    {
      syncCount++;

      //timeout, reset SYNC period counter
      if (syncCount > SYNC_COUNT_MAX)
      {
        syncCount = SYNC_COUNT_INACTIVE;
      }
    }

    //runs once for each output channel
    for (outPort = 0; outPort < NUM_OUTPUTS; outPort++)
    {
      realOffset = offset[outPort] % OFFSET_RESOLUTION;
      index[outPort] = (realOffset * WT_COLS) / OFFSET_RESOLUTION;
      fract[outPort] = (realOffset * WT_COLS) - (index[outPort] * OFFSET_RESOLUTION);
      rec_index[outPort] = (realOffset * RECORDING_BUF_LEN) / OFFSET_RESOLUTION;
      rec_fract[outPort] = (realOffset * RECORDING_BUF_LEN) - (rec_index[outPort] * OFFSET_RESOLUTION);

      //recording is in progress, pass current input CV through to output and fill recording buffer
      if (recBuf.recording[outPort])
      {
        val = recBuf.channel == 0 ? _getPotValue(PIN_IN_CTRL_PAGE1) : _getPotValue(PIN_IN_CTRL_PAGE2);
        recBuf.val[outPort][rec_index[outPort]] = val;
        
        //set next sample ahead of time
        //this prevents skipped samples due to offset phasing effects
        recBuf.val[outPort][rec_index[outPort] + 1] = val; 
                              
      }
      else
      {
        //CV recording exists, use it as middle wave of the current wavetable output
        if (recBuf.recorded[outPort])
        {
          if (wt_row[outPort] == 1)
          {
            wt_val1 = (int16_t) recBuf.val[outPort][rec_index[outPort]];
            wt_val2 = (int16_t) recBuf.val[outPort][rec_index[outPort] + 1];
            val1 = wt_val1 + (int16_t)(((wt_val2 - wt_val1) * rec_fract[outPort]) / OFFSET_RESOLUTION);
          }
          else
          {
            wt_val1 = (int16_t) pgm_read_byte(&wavetable[wt_page[outPort]][wt_row[outPort]][index[outPort]]);
            wt_val2 = (int16_t) pgm_read_byte(&wavetable[wt_page[outPort]][wt_row[outPort]][index[outPort] + 1]);
            val1 = wt_val1 + (int16_t)(((wt_val2 - wt_val1) * fract[outPort]) / OFFSET_RESOLUTION);
          }

          if (wt_row[outPort] == 0)
          {
            wt_val1 = (int16_t) recBuf.val[outPort][rec_index[outPort]];
            wt_val2 = (int16_t) recBuf.val[outPort][rec_index[outPort] + 1];
            val2 = wt_val1 + (int16_t)(((wt_val2 - wt_val1) * rec_fract[outPort]) / OFFSET_RESOLUTION);
          }
          else
          {
            wt_val1 = (int16_t) pgm_read_byte(&wavetable[wt_page[outPort]][wt_row[outPort] + 1][index[outPort]]);
            wt_val2 = (int16_t) pgm_read_byte(&wavetable[wt_page[outPort]][wt_row[outPort] + 1][index[outPort] + 1]);
            val2 = wt_val1 + (int16_t)(((wt_val2 - wt_val1) * fract[outPort]) / OFFSET_RESOLUTION);
          }
        }

        //special page that outputs S&H/S&G values
        if (wt_page[outPort] == WT_PAGE_SPECIAL_RANDOM)
        {
          //channel A has S&G counterclockwise and S&H clockwise
          if (outPort == 0)
          {
            if (wt_row[0] == 0)
            {
              val1 = (uint8_t) randomValCurrent[outPort] + (int16_t)((((uint8_t) randomValNext[outPort] - (uint8_t) randomValCurrent[outPort]) * (offset[0] % OFFSET_RESOLUTION)) / OFFSET_RESOLUTION);

              if (!recBuf.recorded[0])
              {
                val2 = OUTVAL_MID;
              }
            }
            else if (wt_row[0] == 1)
            {
              if (!recBuf.recorded[0])
              {
                val1 = OUTVAL_MID;
              }
              
              val2 = (uint8_t) randomValCurrent[0];
            }
            else
            {
              val1 = val2 = (uint8_t) randomValCurrent[0];
            }
          }
          //channel B has S&H that is attenuated by the MORPH pot
          else
          {
            val1 = 0;
            val2 = (uint8_t) randomValCurrent[1];
            wt_mix[1] = _getPotValue(PIN_IN_CTRL_PAGE2);
          }
        }
        //normal wavetable pages
        else if (!recBuf.recorded[outPort])
        {
          wt_val1 = (int16_t) pgm_read_byte(&wavetable[wt_page[outPort]][wt_row[outPort]][index[outPort]]);
          wt_val2 = (int16_t) pgm_read_byte(&wavetable[wt_page[outPort]][wt_row[outPort]][index[outPort] + 1]);
          val1 = wt_val1 + (int16_t)(((wt_val2 - wt_val1) * fract[outPort]) / OFFSET_RESOLUTION);
          wt_val1 = (int16_t) pgm_read_byte(&wavetable[wt_page[outPort]][wt_row[outPort] + 1][index[outPort]]);
          wt_val2 = (int16_t) pgm_read_byte(&wavetable[wt_page[outPort]][wt_row[outPort] + 1][index[outPort] + 1]);
          val2 = wt_val1 + (int16_t)(((wt_val2 - wt_val1) * fract[outPort]) / OFFSET_RESOLUTION);
        }

        //interpolate between wavetable row values
        val = ((INPUT_MAX - wt_mix[outPort]) * val1 + (wt_mix[outPort] * val2)) >> 8;
      }

      //limit and set output value
      if (val > OUTVAL_MAX)
      {
        outBuf.val[outPort][outBuf.wr] = OUTVAL_MID;
      }
      else
      {
        outBuf.val[outPort][outBuf.wr] = val;
      }
    }

    oldChannelAOffset = offset[0] % OFFSET_RESOLUTION;
    offsetDelta = OFFSET_RESOLUTION / period;
    realOffset += offsetDelta;
    offset[1] += offsetDelta;

    //wrap around (scaled) channel B offset
    if (offset[1] >= DIV_TABLE_MAX_PRODUCT * OFFSET_RESOLUTION)
    {
      offset[1] -= DIV_TABLE_MAX_PRODUCT * OFFSET_RESOLUTION;
    }

    //handle channel A offset according to the current DIV mode
    //normal mode: channel A offset is a fraction of channel B offset, determined by currently selected DIV table value
    if (divMode == DIV_MODE_NORMAL)
    {
      offset[0] = ((offset[1] * DIV_TABLE_RESOLUTION) / div_val) % OFFSET_RESOLUTION;
    }
    //free mode: channel A offset is an arbitrary fraction of channel B offset, determined by current DIV pot reading
    else if (divMode == DIV_MODE_FREE)
    {
      offset[0] += (OFFSET_RESOLUTION * DIV_TABLE_RESOLUTION) / div_val / period;
  
      if (offset[0] >=  OFFSET_RESOLUTION)
      {
        offset[0] -= OFFSET_RESOLUTION;
      }
    }
    //phase mode: channel A offset is phase shifted from channel B offset, with the phase angle determined by current DIV pot reading
    else if (divMode == DIV_MODE_PHASE)
    {
      offset[0] = (OFFSET_RESOLUTION + offset[1] - (OFFSET_RESOLUTION * div_val) / INPUT_MAX) % OFFSET_RESOLUTION;
    }

    //check if channel A has wrapped
    if (offset[0] < oldChannelAOffset)
    {
      //check if recording on channel A has been initiated or needs to be stopped
      _checkRecording(0);
      
      //update random values for channel A
      if (wt_page[0] == WT_PAGE_SPECIAL_RANDOM)
      {
        _newRandomValue(0);
      }
    }

    if (syncPrecisionCountdown)
    {
      syncPrecisionCountdown--;
    }
    
    //check if channel B has wrapped
    if (offset[1] % OFFSET_RESOLUTION <= offsetDelta)
    {
      //update random values for channel B
      if (wt_page[1] == WT_PAGE_SPECIAL_RANDOM && !syncPrecisionCountdown)
      {
        _newRandomValue(1);
        syncPrecisionCountdown = SYNC_PRECISION_SH;
      }

      //check if recording on channel B has been initiated or needs to be stopped
      _checkRecording(1);

      //handle SYNC LED flash on channel B cycle boundaries
      if (!syncLEDchecked)
      {
        _checkSyncLED(true);
      }
    }
    else
    {
      //call this function in order to decrease timer
      _checkSyncLED(false);
    }

    //increment output buffer indices
    //they will wrap automatically (since they're bitfields)
    outBuf.wr++;
    outBuf.lvl++;
  }
}

//interrupt routine that writes output buffer values to PWM outputs
ISR (TIMER1_OVF_vect)
{
  if (outBuf.lvl)
  {
    _setPWMoutput(0, outBuf.val[0][outBuf.rd]);
    _setPWMoutput(1, outBuf.val[1][outBuf.rd]);
    outBuf.rd++;
    outBuf.lvl--;
  }
}

//sleep function implemented by NOPs
static inline void _nopSleep(uint16_t sleepTime)
{
  while (sleepTime--)
  {
    _NOP();
  }
}

int main()
{
  init();
  cli();

  //interrupt setup
  TCCR1 = 1 << CS12 | 1 << CS11 | 1 << CS10;
  TCNT1 = 0;
  TIMSK |= 1 << TOIE1;

  //Timer0 PWM setup
  DDRB = (1 << PB1) | (1 << PB0);
  TCCR0A = 0x00;
  TCCR0A |= (1 << WGM00) | (1 << COM0A1) | (1 << COM0B1) | (1 << COM0A0) | (1 << COM0B0);
  TCCR0B = 0x00;
  TCCR0B |= (1 << CS00);
  TCNT0 = 0;
  OCR0A = 255;
  OCR0B = 255;

  sei();

  //ADC setup
  ADMUX |= (1 << MUX0);
  ADMUX |= (1 << MUX1);
  ADMUX |= (1 << ADLAR);
  ADCSRA |= (1 << ADEN);
  ADCSRA |= (1 << ADPS1);
  ADCSRA |= (1 << ADPS0);

  //set output buffer indices
  outBuf.rd = outBuf.wr = 0;

  //initialise recording buffer
  recBuf.recorded[0] = false;
  recBuf.recorded[1] = false;
  recBuf.channel = 0;

  //delay main loop for a short time on startup
  //this mainly prevents unpredictable behaviour if the MCU is plugged into an already powered module
  _nopSleep(STARTUP_SLEEP_TIME);

  //seed random value generator and initialise random buffers
  _inputUpdateRaw();
  randomValNext[0] = _xorshift32(PRNG_SEED1 ^ (inVal[0] << 24) ^ (inVal[1]  << 16) ^ (inVal[2] << 8) ^ inVal[3]);
  randomValNext[1] = _xorshift32(PRNG_SEED2 ^ randomValNext[0]);
  _newRandomValue(0);
  _newRandomValue(1);

  //main loop
  while (true)
  {
    _ioUpdate();
    _fillBuffer();
    _nopSleep(RELAX_TIME);
  }

  return 0;
}
