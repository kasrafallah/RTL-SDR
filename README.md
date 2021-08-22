## RTL-SDR 
## (FM radio player using Matlab )

#### highly recommended to use Matlab 2014B for utilizing  RTL-SDR

In this project, I make a radio FM broadcast player with Matlab real-time processing, and for better performance, I wrote a user-friendly GUI for this application.
for listening to FM radio with this code, you just need an RTL-SDR antenna and knowing regional FM channels frequencies to set the radio on them.
in this project, I use three methods for demodulating FM signals and I want to mention that I didn't use any of the Matlab ready demodulators

1. Quadrature detector

2. Zero crossing demodulator


3. Hilbert demodulator

and for presenting signal power spectral density I use two popular methods

1. pwlech

2. wiener-khinchin theorem

GUI view of this FM player

<p align="center">
<image align="center" src = "images/FM.png" width="600">
</p>
highly recommended to use Matlab 2014B for utilizing  RTL-SDR
