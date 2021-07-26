
#include "Copter.h"

void Copter::init_optimaero(void){
  g2.oa_module.init();
}

void Copter::update_optimaero_400Hz(void){
  g2.oa_module.update400hz();
}

void Copter::update_optimaero_100Hz(void){
  g2.oa_module.update100hz();
}

void Copter::update_optimaero_50Hz(void){
  g2.oa_module.update50hz();
}
    
void Copter::update_optimaero_10Hz(void){
  g2.oa_module.update10hz();
}

void Copter::update_optimaero_3Hz(void){
  g2.oa_module.update3hz();
}

void Copter::update_optimaero_1Hz(void){
  g2.oa_module.update1hz();
}

/** The Chirp class produces a Chirp frequency-sweep profile for which
 * each doubling of frequency takes half as long as the previous. */
/** Sets the amplitude of oscillation for the Chirp.
 @param amplitude_in Amplitude of oscillation */
void Copter::ChirpSetAmplitude( float amplitude_in) { m_amplitude = amplitude_in; }

/** Sets the start and stop frequencies of the Chirp
 @param start_frequency_hz Starting frequency in Hz
@param stop_frequency_hz Final frequency in Hz
@param dt time step in seconds (rate at which Update will be called) */
void Copter::ChirpSetFrequencies( float start_frequency_hz,
                                  float stop_frequency_hz,
                                  float dt)
{
    
  m_dt = dt;

  // Sanity-check inputs.  Set parameters to zero if inputs are bad.
  // Frequencies should be increasing and less than Nyquist.
  if ((start_frequency_hz >= stop_frequency_hz) ||
      ((stop_frequency_hz*2.f) > (1.f/m_dt)))
  {
      m_phaseInRate     = 0.f;
      m_startFrequency  = 0.f;  
      m_stopFrequency   = 0.f;
  }
  else
  {
      m_phaseInRate     = start_frequency_hz*m_dt;
      m_startFrequency  = m_phaseInRate*TAU;  
      m_stopFrequency   = stop_frequency_hz*TAU*m_dt;
  }
}

/** Resets the state of the Chirp to the beginning of the profile */
void Copter::ChirpReset()
{
  // The phase-in stage should start with a cosine wave, but the
  // termination stage is simpler using a sine wave.  Starting at
  // TAU/4 makes it look like a cosine.
  const float starting_phase = TAU / 4.f;

  m_phase           = starting_phase;
  m_phaseInGain     = 0.f;
  m_frequency       = m_startFrequency;
  m_value           = (float)0.0f;
  m_completedFlag   = false;
} 

/** Initialize an instance of a Chirp
 @param start_frequency_hz Starting frequency in Hz
@param stop_frequency_hz Final frequency in Hz
@param CPO Number of complete oscillations before doubling in frequency
@param dt time step in seconds (rate at which Update will be called)
@param amplitude_in Amplitude of oscillation (default 1) */
void Copter::ChirpInit(float start_frequency_hz,
          float stop_frequency_hz,
          float CPO,
          float dt,
          float amplitude_in)
{
  ChirpSetFrequencies(start_frequency_hz, stop_frequency_hz, dt);

  if (CPO < 1.f) cycles_per_octave = 1.f;

  // The expression for 'ratio' is an approximation of the following:
  //
  //  ratio = (exp(1.f/(TAU*cycles_per_octave))-1)*(float)M_LN2;
  //
  // This approximation is very accurate for cycles_per_octave values
  // greater than 1, without the cost of the 'exp' call.
  m_ratio = LN2 / (TAU*cycles_per_octave - 0.5f);

  ChirpSetAmplitude(amplitude_in);

  ChirpReset();
}



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
float Copter::ChirpUpdate()
{
  // phase-in period.  Linear ramp amplitude from 0 to full for one cycle
  // without increasing frequency.  This phase-in period integrates to zero
  // so the integral of the chirp has no bias.
  if (m_phaseInGain < (1.f - m_phaseInRate/2.f))
  {
      m_phase       += m_startFrequency;
      m_phaseInGain += m_phaseInRate;
      m_value        = m_amplitude*(float)(sinf(m_phase)*m_phaseInGain);
  }
  // Termination.  Wait for current cycle to complete, then set to zero.
  else if(m_frequency >= m_stopFrequency)
  {
      m_phase          += m_stopFrequency;
      if(m_phase > TAU)
      {
        m_value         = (float)0.0f;
        m_completedFlag = true;
      }
      else
      {
        m_value         = m_amplitude*(float)sinf(m_phase);
      }
  }
  else
  {
      m_phase           += m_frequency/2.f;
      m_frequency       /= 1.f-m_ratio*m_frequency;
      m_phase           += m_frequency/2.f;

      if(m_phase > TAU) m_phase -= TAU;
      m_value           = m_amplitude*(float)sinf(m_phase);
  }

  return m_value;
}

/** Returns the approximate number of loop iterations to execute the chirp
   profile as configured.
@return The duration of the chirp profile in number of loop iterations. */
uint32_t Copter::ChirpDuration()
{
  return (uint32_t)((1.f/m_phaseInRate) + 
                    ((m_stopFrequency-m_startFrequency)
                        / (m_ratio*m_startFrequency*m_stopFrequency)) +
                    (TAU/m_stopFrequency/2.f) + 0.5f);
}

/** Returns the frequency in Hz of the current oscillation */
float Copter::ChirpFrequencyInHz()
{
  return (float) m_frequency / m_dt / TAU;
}

/** Accesses the current value of the Chirp profile. Implemented as a
   type-cast operator.
@return The value of the Chirp */
 

bool Copter::ChirpIsCompleted(void)
{
  return m_completedFlag;
}
