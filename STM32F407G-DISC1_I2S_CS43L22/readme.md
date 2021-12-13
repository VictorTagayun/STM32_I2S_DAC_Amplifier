Audio freq for Left and Right = 1.5kHz  
I2S Clock freq:  
PC10 = CK = 1.56Mhz - Serial Clock for the data in/out  
PC12 = SD = N/A - Audio Databits/stream  
PA4  = WS = 48.83kHz - Left/Right clock, determines which channel is active on the serial data line  
PC7  = MCLK = 12.56MHz - Master Clock for the delta sigma modulator  

data bits = CK (1.56MHz) / WS (48.83kHz) = 32bits stereo   
