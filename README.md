# FFTslice

Module to evaluate FFT. Communication with SPI. PGA for gain control. 2 selectable channels, additional DAC channel for selftest.

<p align="center">
    <img src="img/fftslice_v0_1_pcb.png" width="500" alt="FFTSlice PCB V0.1">
</p>

## DAC samples generation

See [notebook](Doc/DAC_samples/FFTSlice_DAC.ipynb) with all calculations. 

Frequencies from 20 Hz to 20 kHz are splitted into 100 ranges in logspace to better fit sound perception. 

<p align="center">
    <img src="img/dac_samples_plot_4.png" width="500" alt="plot 1">
</p>

3 variables affects output DAC frequency:
- constant TIM1 clock = 64 MHz,
- variable divider
- samples count

Divider can be freely changed, but samples count have to be generated in advance and stored in FLASH. There are 6 samples counts: 64, 128, 256, 512, 1024 and 2048. Higher samples counts fits better to generate low frequencies, lower samples counts are better fot higher frequencies.

<p align="center">
    <img src="img/dac_samples_plot_2.png" width="500" alt="plot 2">
</p>

100 parameters are calculated so that samples count and divider lowers time in which output voltage remain the same, and deviation from theoretical frequency is acceptable.

Resulting error remains below 1% as seen in the plot:

<p align="center">
    <img src="img/dac_samples_plot_3.png" width="500" alt="resulting relative error">
</p>

Matching of samples counts is more or less proportional:

<p align="center">
    <img src="img/dac_samples_plot_1.png" width="500" alt="matching samples counts">
</p>

Resulting frequencies are very close to theoretically calculated. In the plot sinewave with output buffer enabled without low pass filter:

<p align="center">
    <img src="img/dac_samples_freq_compare_1.PNG" width="500" alt="Output frequency comparison">
</p>

