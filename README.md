# GPS disciplined oscillator controller based on MSP430G2553 built-in timers.

Used software components:
- Nokia5110 display driver heavily inspired by Opposum's [frequency counter project](https://web.archive.org/web/20230330184220/https://forum.43oh.com/topic/1954-using-the-internal-temperature-sensor/)
- HW UART interrupt driven transceiver inspired by Musi's [frequency counter](wlan slovenia http://dev.wlan-si.net)

The "heart" of PLL is phase detector implemented in Timer1_A1 ISR. Error function is measured as a difference in phase between two signals connected to either capture CCI1A inputs of Timer1_A. Becasue Timer1 is clocked by SMCLK which is derived from undivided DCO, the resolution of error function is approx 50ns - half period of 10MHz REF and VCO signal, and can be of value from approx -1000 (-pi) to +1000 (+pi). Driftrate of this error function directly relates to the frequency difference between, and can be used to estimate phase and frequency error.
The function of a controller boils down to:
- prepare GPS module for cold start (reset date: [week roll-over](https://fdi.sk/posts/jupiter/) and preset the NMEA messages: RMC and GGA only)
- monitor oven temperature (start operation only when reaches approx 52degC)
- establish target phase difference - at the moment of acquisition and treat it as phase difference set-point for PID controller
- control OCXO to achieve minimum phase error
- bonus: input of Timer0_A is free to count up to 50000, which gives 0.2Hz resolution frequency meter displayed on the 5^th line of LCD

Explanation of LCD fields:
1. top line from left:
   - OCXO temperature - inversed when below 50deg
   - MSP die temperature
   - TDP - when inversed: GPS provides valid information of:
     - time
     - date
     - position
   - low case main controller mode:
     - s start-up
     - w warm-up
     - l locking
     - t tracking - when inversed the abs(error) is less then 8, and error_delta is less than 2
   - capital letter fix status:
     - V inValid
     - A vAlid
   - number of used sattelites
2. bottom line, shows either:
   - phase difference between REF and VCO derived 10kHz. Rolling right indicates the error function to be increasing, meaning (`vco - ref`) VCO is of higher frequency than REF
   - frequency measurement in Timer0, with the scaling factor as the bottom right digit. It indicates how the graph in the middle of screen should be interpreted.
3. middle 4 lines, shows
   - during warm-up phase a temperature every 5th second one pixel, with 50deg mark indicated by dotted line at the top of the graph.
   - during tracking, relative phase difference (error function). PID controller should eliminate it and track it to zero. Scale function is indicated by digit in bottom-right: 5 down to 0 (ommited), indicates a +max/-min range in +16/-16 graph using 4 lines of LCD. Scale indicates power of 2 (2^y_scale) scale, hence 0 indicates +16/-16, while 1 +32/-32, through 5 which indicates +512/-512


<pre> NOKIA 5110 LCD  84x48 pixels                                   MSP-EXP430G2
 ------------                                           -------------------
|         GND|<-- Ground ------------------------------|J6     GND         |
|          BL|<-- Back-light - tie to ground via res   |                   |
|         VCC|<-- Vcc +3..5V --------------------------|1      VCC         |
|            |                                         |                   |
|         CLC|<-- Clock -------------------------------|7      P1.5        |
|         DIN|<-- Data Input --------------------------|15     P1.7        |
|          DC|<-- Data/Command (high/low) -------------|11     P2.3        |
|          CE|<-- Chip Enable (active low) ------------|18     P2.7  XOUT  |
|         RST|<-- Reset - RC                           |                   |
 ------------                                          |                   |
                                                       |                   |
   GPS                                                 |                   |
 ------------                                          |                   |
|         TX |<-- NMEA output -------------------------|3      P1.1        |
|         RX |<-- NMEA input --------------------------|4      P1.2        |
|      10kHz |<-- GPS reference signal--------+--------|19     P2.6 XIN    |
|            |                                \--------|9      P2.1        |
|        PPS |<-- NMEA input --------------------------|Jx.x   Px.x        |
 ------------                                          |                   |
                                                       |                   |
   OCXO                                                |                   |
 ------------                                          |                   |
|      10MHz |<-- VCO output --------------------------|2      P1.0 (ext)  |
|      10kHz |<-- VCO output divided % 1000 -----------|12     P2.4        |
|       Vref |<-- VCO input ------------\--------o<|---|14     P1.6 (neg)  |
|            |                           \ ------o<|---|14     P2.0 (neg)  |
|       LM35 |<-- temperature -------------------------|5      P1.3        |
 ------------                                          |                   |
                                                       |                   |
   4046                                                |                   |
 ------------                                          |                   |
|         PC |<-- Phase comparator output -------------|6      P1.4        |
 ------------                                           -------------------
 </pre>
