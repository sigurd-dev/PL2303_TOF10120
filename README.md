# PL2303_TOF10120

   Example program to read distances from TOF10120 Laser Range Sensor.
   If using a PL2303 Serial Port (USB To RS232 TTL PL2303HX Converter link below)
   connect wires like this:
   Red->3V
   Yellow->TDX
   White->RDX
   Black->GND 
   (Blue and Green are not used) 

   Compile with: 
   gcc PL2303_TOF10120_serial.c strrep.c -o PL2303_TOF10120_serial
   or use included makefile.

   The rather cheap hardware can be bought here:
   <a href="https://www.aliexpress.com/item/4001120526796.html?spm=a2g0s.9042311.0.0.27424c4dDGURpH">TOF10120</a> and
   <a href="https://www.aliexpress.com/item/4001134803817.html?spm=a2g0s.9042311.0.0.27424c4dDGURpH"> USB To RS232 TTL PL2303HX</a>
  
