#include <Arduino.h>
#include <U8g2lib.h>
#include <STM32FreeRTOS.h>
#include <math.h>


// This datatype holds a waveform
struct Sound {
    bool looped;            // Set to true if the waveform should be played continuously in a loop; False if only played once
    int waveform_length;    // Specifies the length of the following waveform array
    int8_t waveform[];  // Array of variable size
};

struct Octave {
    Sound * sounds[12];
};

// Helper functions that generate different waveforms on the fly, so that they don't everything has to be stored

// Sinusoidal waveforms
inline Sound * generate_sinusoid(float sin_freq) {
    // The amount of data that needs to be stored depends on the sinusoid frequency, this can be optimised depending on frequency
    // constant waveform length for now


    // This block calculates a good length of the waveform array
    // The ideal length is f_s/f_sin, but this is a floating point number and we need an int array length
    // It searches for a waveform array length that minimises the error and allows for smooth waveform-cycling
    int waveform_length = 750;

    for(int l=1; l<10; l++) {
        double THRESH = 0.05;
        double division_result = 22000.0/sin_freq;
        double ideal_length = division_result*(float)l;
        double proposed_length = round(ideal_length);
        int deviation = abs(proposed_length - ideal_length);
        if((deviation<THRESH) && (proposed_length < waveform_length)) {
            waveform_length = proposed_length;
        }
    }

    Sound * sinusoid_ptr = (Sound*)malloc( sizeof( Sound ) + sizeof(uint8_t)*waveform_length );

    sinusoid_ptr->looped = true; // A sinusoid is played continuously
    sinusoid_ptr->waveform_length = waveform_length;

    for(int i=0; i<waveform_length; i++) {
        sinusoid_ptr->waveform[i] = (uint8_t)((sin(2.0 * (i/22000.0) * M_PI * sin_freq)) * 128);
    };

    sinusoid_ptr->waveform[waveform_length-1] = sinusoid_ptr->waveform[0];

    return sinusoid_ptr;
};

