
struct optimAero_State {
    bool enabled:1;         	  // oa Status
    uint32_t last_healthy_ms;  	// system time of last update
}optimaero_state; 


void init_optimaero(void);
void update_optimaero_100Hz(void);
void update_optimaero_50Hz(void);    
void update_optimaero_10Hz(void);
void update_optimaero_1Hz(void);
void update_optimaero_3Hz(void);
void update_optimaero_400Hz(void);

/** The Chirp class produces a Chirp frequency-sweep profile for which
 * each doubling of frequency takes half as long as the previous. */

/** Sets the amplitude of oscillation for the Chirp.
@param amplitude_in Amplitude of oscillation */
void ChirpSetAmplitude( float amplitude_in);

/** Sets the start and stop frequencies of the Chirp
@param start_frequency_hz Starting frequency in Hz
@param stop_frequency_hz Final frequency in Hz
@param dt time step in seconds (rate at which Update will be called) */
void ChirpSetFrequencies( float start_frequency_hz,
                          float stop_frequency_hz,
                          float dt);


/** Resets the state of the Chirp to the beginning of the profile */
void ChirpReset();


/** Initialize an instance of a Chirp
@param start_frequency_hz Starting frequency in Hz
@param stop_frequency_hz Final frequency in Hz
@param CPO Number of complete oscillations before doubling in frequency
@param dt time step in seconds (rate at which Update will be called)
@param amplitude_in Amplitude of oscillation (default 1) */
void ChirpInit(float start_frequency_hz,
               float stop_frequency_hz,
               float CPO,
               float dt,
               float amplitude_in);


/** Executes one iteration of the chirp waveform.  Must be called at the dt
 update period at which it was configured to get the desired frequencies.

The Chirp waveform has 3 stages:
- the phase-in stage increases the amplitude of the oscillation from
  zero to its full value for one cycle.  The phase-in period is at the
  start_frequency.
- the frequency-sweep phase, during which the frequency increases from
  start_frequency to stop_frequency at a constant amplitude
- the termination phase.  The oscillation continues at the stop_frequency
  until it approaches zero from below, then holds at zero until reset.
@return the value of the chirp waveform for the current time step. */
float ChirpUpdate();


/** Returns the approximate number of loop iterations to execute the chirp
   profile as configured.
@return The duration of the chirp profile in number of loop iterations. */
uint32_t ChirpDuration();

/** Returns the frequency in Hz of the current oscillation */
float ChirpFrequencyInHz();

bool ChirpIsCompleted(void);

//log message
void Log_Write_Optim_ChirpFull(void);
