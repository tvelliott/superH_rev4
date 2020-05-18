<html>
<b>Ultra-performance narrow-band receiver hardware.
<BR>
<BR><B>SuperH_rev4
<BR>
<p>
<img src="https://github.com/tvelliott/superH_rev4/blob/master/superH_rev4_pcb.png">
<BR>
<BR>
The firmware may be added to the repository later.  Currently can decode the follow protocols in stand-alone mode (NO PC):
<PRE>
Available SuperH+ built-in/stand-alone demod/decoders
------------------
FM Analog narrow/med/wide (mode fm)
AM Analog narrow/med/wide (mode am)
IQ 16-bit over UDP (mode iq) + GNURadio Driver
P25P1 Voice + Trunking Control Channel (mode p25)
DMR Voice + ConnectPlus Trunking Control Channel (mode dmr)
ADSB Mode-S - Console output (mode adsb)   This mode uses 2 separate ADCs in single ended mode with 2 Msps / 8-bit.
ACARS - Console output (mode acars)
FLEX-4FSK-1600 - Console output (mode pagers, fm)
POCSAG 1200 - Console output (mode pagers, fm)
</PRE>
<BR>
<BR>
This is not a cheap build by any measure, but the RF performance is stellar from 10 MHz to 1100 MHz.  Usable tuning range is from around 4 MHz to 1200 MHz.
<BR><BR>
1st IF is 1.5 GHz  (high IF like a typical spectrum analyzer)
<BR><BR>
Baseband filters are 10 - 150 kHz in 16 steps.  70dB adjacent rejection,  100+ dB alternate rejection.
<BR><BR>NCO based final tuning.
<BR><BR>20dB pre-LNA attenuator front-end
<BR><BR>AGC + DSP extends dynamic range to over 150dB.
<BR><BR>ACARS receives long messages >350 miles away with an indoor antenna and 0 bit errors.
<BR><BR>DC coupling and extremely stable TCXO makes sub-Hz demodulation with software-based DC offset correction possible
<BR><BR>I/Q ADCs SNR is >= 82 dB in 16-bit differential mode of operation (10-150 kHz baseband filter mode).   

<BR>
<BR>
<a href="https://github.com/tvelliott/superH_rev4/blob/master/audio_samples/superH_fm_radio_dynamics_silence_demo.wav">superH_fm_radio_dynamics_silence_demo.wav (recorded off air)</a>
<BR>
<BR>
superH FM demod demonstration    (notice how you can see the phase display almost slow to stop on the unit circle during FMNB). 
https://youtu.be/HBXcUFW8yaA  
<BR><BR>
P25P1 demo
https://youtu.be/G3MCHvNcKHg
<BR><BR>
superH gqrx / 40-meter / AM demo
https://youtu.be/u9L5sJRPFiQ
<BR><BR>
Local ham radio repeater VHF
https://youtu.be/9KYXXc-55yA
<BR><BR>
SuperH+ receiving ADSB
https://youtu.be/LvIR2hlR7_A
<BR><BR>
<BR>
<img src="https://github.com/tvelliott/superH_rev4/blob/master/superH_rev4_pcb_final_assembly_small.png">
<BR>
</html>
