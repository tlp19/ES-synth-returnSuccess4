#include "sample_library.hpp"

using namespace std;

//  ----  Octaves of Sinusoidal sound arrays  ----

const struct Octave sineOctave0 = {sounds:{
    generate_sinusoid(16.35),
    generate_sinusoid(17.32),
    generate_sinusoid(18.35),
    generate_sinusoid(19.45),
    generate_sinusoid(20.60),
    generate_sinusoid(21.83),
    generate_sinusoid(23.12),
    generate_sinusoid(24.50),
    generate_sinusoid(25.96),
    generate_sinusoid(27.50),
    generate_sinusoid(29.14),
    generate_sinusoid(30.87),
  }};

const struct Octave sineOctave1 = {sounds:{
    generate_sinusoid(32.70),
    generate_sinusoid(34.65),
    generate_sinusoid(36.71),
    generate_sinusoid(38.89),
    generate_sinusoid(41.20),
    generate_sinusoid(43.65),
    generate_sinusoid(46.25),
    generate_sinusoid(49.00),
    generate_sinusoid(51.91),
    generate_sinusoid(55.00),
    generate_sinusoid(58.27),
    generate_sinusoid(61.74),
  }};

const struct Octave sineOctave2 = {sounds:{
    generate_sinusoid(65.41),
    generate_sinusoid(69.30),
    generate_sinusoid(73.42),
    generate_sinusoid(77.78),
    generate_sinusoid(82.41),
    generate_sinusoid(87.31),
    generate_sinusoid(92.50),
    generate_sinusoid(98.00),
    generate_sinusoid(103.83),
    generate_sinusoid(110.00),
    generate_sinusoid(116.54),
    generate_sinusoid(123.47),
  }};

const struct Octave sineOctave3 = {sounds:{
    generate_sinusoid(130.81),
    generate_sinusoid(138.59),
    generate_sinusoid(146.83),
    generate_sinusoid(155.56),
    generate_sinusoid(164.81),
    generate_sinusoid(174.61),
    generate_sinusoid(185.00),
    generate_sinusoid(196.00),
    generate_sinusoid(207.65),
    generate_sinusoid(220.00),
    generate_sinusoid(233.08),
    generate_sinusoid(246.94),
  }};

const struct Octave sineOctave4 = {sounds:{
    generate_sinusoid(261.63),
    generate_sinusoid(277.18),
    generate_sinusoid(293.66),
    generate_sinusoid(311.13),
    generate_sinusoid(329.63),
    generate_sinusoid(349.23),
    generate_sinusoid(369.99),
    generate_sinusoid(392.00),
    generate_sinusoid(415.30),
    generate_sinusoid(440.00),
    generate_sinusoid(466.16),
    generate_sinusoid(493.88),
  }};

const struct Octave sineOctave5 = {sounds:{
    generate_sinusoid(523.25),
    generate_sinusoid(554.37),
    generate_sinusoid(587.33),
    generate_sinusoid(622.25),
    generate_sinusoid(659.25),
    generate_sinusoid(698.46),
    generate_sinusoid(739.99),
    generate_sinusoid(783.99),
    generate_sinusoid(830.61),
    generate_sinusoid(880.00),
    generate_sinusoid(932.33),
    generate_sinusoid(987.77),
  }};

const struct Octave sineOctave6 = {sounds:{
    generate_sinusoid(1046.50),
    generate_sinusoid(1108.73),
    generate_sinusoid(1174.66),
    generate_sinusoid(1244.51),
    generate_sinusoid(1318.51),
    generate_sinusoid(1396.91),
    generate_sinusoid(1479.98),
    generate_sinusoid(1567.98),
    generate_sinusoid(1661.22),
    generate_sinusoid(1760.00),
    generate_sinusoid(1864.66),
    generate_sinusoid(1975.53),
  }};

const struct Octave sineOctave7 = {sounds:{
    generate_sinusoid(2093.00),
    generate_sinusoid(2217.46),
    generate_sinusoid(2349.32),
    generate_sinusoid(2489.02),
    generate_sinusoid(2637.02),
    generate_sinusoid(2793.83),
    generate_sinusoid(2959.96),
    generate_sinusoid(3135.96),
    generate_sinusoid(3322.44),
    generate_sinusoid(3520.00),
    generate_sinusoid(3729.31),
    generate_sinusoid(3951.07),
  }};

const struct Octave sineOctave8 = {sounds:{
    generate_sinusoid(4186.01),
    generate_sinusoid(4434.92),
    generate_sinusoid(4698.63),
    generate_sinusoid(4978.03),
    generate_sinusoid(5274.04),
    generate_sinusoid(5587.65),
    generate_sinusoid(5919.91),
    generate_sinusoid(6271.93),
    generate_sinusoid(6644.88),
    generate_sinusoid(7040.00),
    generate_sinusoid(7458.62),
    generate_sinusoid(7902.13),
  }};

/// Octaves of Sinusoidal sound arrays
volatile const Octave sineOctaves[9] = {sineOctave0, sineOctave1, sineOctave2, sineOctave3, sineOctave4, sineOctave5, sineOctave6, sineOctave7, sineOctave8};
