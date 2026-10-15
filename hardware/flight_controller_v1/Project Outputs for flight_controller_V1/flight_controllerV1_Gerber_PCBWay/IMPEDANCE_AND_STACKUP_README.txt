=====================================================================
 FLIGHT CONTROLLER V1 - FABRICATION NOTES: STACKUP & IMPEDANCE CONTROL
=====================================================================

1. GENERAL
---------------------------------------------------------------------
- Layers:              4
- Finished thickness:  1.6 mm
- Material:            FR4
- Copper:              1 oz finished outer / 1 oz inner
- Impedance control:   YES (2 controlled traces, both on L1 - see section 3)
- Via filling:         NO resin filling / plugging. Use standard vias.
                       Some vias are in pads; I accept the soldering risk.


2. STACKUP - PLEASE USE YOUR STANDARD STACKUP
---------------------------------------------------------------------
To keep cost down, please use your STANDARD 4-layer 1.6 mm stackup
(listed as No. 1 on the "Multi-layer laminated structure" page:
1 oz outer / 1 oz inner, inner residual copper ratio 70%).
A custom stackup is NOT required.

Layer   Material                     Thickness   After lamination
                                     (mm)        (mm)
-----   --------------------------   ---------   ----------------
L1-CU   Outer base Cu 0.5 oz         0.0175      0.0175 (plated to 1 oz)
PP      7628 RC46%  Dk 4.74          0.1960      0.1855
L2-CU   Inner Cu 1 oz                0.0350      \
CORE    Core  Dk 4.6                 1.0300       > 1.1 (core with Cu)
L3-CU   Inner Cu 1 oz                0.0350      /
PP      7628 RC46%  Dk 4.74          0.1960      0.1855
L4-CU   Outer base Cu 0.5 oz         0.0175      0.0175 (plated to 1 oz)

Layer functions:
- L1  Signal (RF + USB controlled traces)
- L2  Solid GND plane  <-- REFERENCE PLANE for all controlled traces
- L3  Power / signal
- L4  Signal


3. CONTROLLED IMPEDANCE TRACES
---------------------------------------------------------------------

 #  Signal            Layer  Type                          Target           Calculated Width     Spacing   Ref. layer
 -  ----------------  -----  ----------------------------  ---------------  -----------------  --------    ----------
 1  RF antenna feed   L1     Single-ended coated           50 ohm +/-10%    0.3115 mm          -           L2 (GND)
                             microstrip (non-coplanar)                      (12.26 mil)
 2  USB D+ / D-       L1     Edge-coupled differential     90 ohm +/-10%    0.25 mm            0.16 mm     L2 (GND)
                             coated microstrip                              (9.84 mil)         (6.30 mil)
                             (non-coplanar)                                 each trace

Notes:
- Both controlled traces are on L1 (top) only, covered by solder mask,
  referenced to the solid GND plane on L2.
- Trace 1 runs from the RF output of the ESP32-S3 LNA pin module/matching
  network to the chip antenna feed.
- Trace 2 is the USB D+/D- pair running from the USB-C connector to
  the ESP32-S3.

PERMISSION TO ADJUST:
The widths above were calculated for the stackup in section 2 (IPC
microstrip formulas, Dk 4.74, h = 0.1855 mm). If your field-solver
result differs, please adjust the trace width and spacing of these
traces to meet the impedance targets. If there is another default
PCBWay 4 layer 1.6mm stackup which you recommend more please use 
instead.


Please contact if there are any questions, thanks very much for the help.
