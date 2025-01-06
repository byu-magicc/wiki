Designing a Propulsion System
====================================

## Vehicle Power-System Recommendations ##
#### Best Practices ####

*Please read thoroughly all of the following instructions and tips. DJI has expensive vehicles because a lot of effort is necessary to assemble a successful vehicle. If you skimp on effort at the design and assembly stage, by not being aware of the following tips learned through extensive experience and research, you could end up having to start over to replace a pile of ashes. That becomes expensive very quickly.*

When building a vehicle, reliable power-system architecture is critical for a robust vehicle. It is best to ensure a 15% upgrade (if not 20%) in rated capability (typically amperage, but consider total power [wattage] as well) for each component of the power-system, starting at the output (i.e. load. Typically, the motor).

Many factors can affect the power at the motor, including:

* motor size
 * diameter
 * length
 * kv rating
* propeller
 * diameter
 * pitch
* battery
 * number of cells
* cold solder joints

Changing any one of these factors will change the wattage, and hence amperage, at the motor.

e.g.:

* If you expect (or can measure) the amperage into a motor at *100A continuous*, then, ideally, you should aim to have a motor whose spec sheet claims to be rated to *at least 115A continuous*

* Moving up the power-system, the electronic speed controller (ESC) should add an additional 15% on the motor rating. Therefore, the ESC would ideally be rated for *at least 132A*

* The next, and final, component in the system is the battery. Adding 15% on top of the ESC rating would be 152A. So, in this (very) hypothetical case, if you have a 5,000mAh battery, you should make sure its 'C' rating is *at least 30C*

* Finally, make sure a proper connectors are used throughout the system for this max computed amperage out of the battery. You will need two different types of connectors for the system. Bullet connectors will connect the motors to the ESCs, and a polarized connector will be used for all connections and splitters between the ESC and battery. Research connectors to find the right size that satisfies the amperage/power requirement. Additionally, pair the connectors with appropriately sized #-awg wire throughout the system as well, to avoid both insulation-break-down and melting
 * Bullet connectors: usually differentiated by diameter size in millimeters. The larger diameter, the more current and power can be safely pushed through them
 * Polarized connectors: Many different brands exist, such as XT##, Dean's, EC#, Anderson Powerpole (APP##), etc. Find a properly sized/specced one for your amperage and power requirements. **Do not attempt to force nor accidentally connect polarized connectors into each other 'the wrong way.' Reverse polarization can destroy the ESC and/or the battery**
   * You'll see a lot of XT60 and Dean's in the MAGICC lab, with some XT90, depending on preference of the person who assembled the vehicle power system. Feel free to go with preference on connector type, as long as the size is large enough for the amperage of your system

Following a scheme like this will avoid putting unnecessary wear and tear on system components, especially with the dangers of [over-heating Li-Po batteries](l01_lipo_batts.md#Moral_of_the_Story:). We would rather not have the lab vehicles burning up, and your mother would rather not have you burning up (and I don't mean sunburn, though I'm sure she'd rather not that, either).



#### Note: ####
The 15% upgrade in capability for each compoenent is not on the base power output, but is calculated as **compounding** on each component up the chain. Hence, the battery should be rated for **152A**, not just 145A. Likewise for the ESC, it is not 130A, but 132A. The difference may not be large, but safer is better when dealing with [Li-Po's](l01_lipo_batts.md#Moral_of_the_Story:). If you can, try to push the factor to 20% or more.


## Example ##

#### Note ####
The purpose of the following example build was to experiment how much thrust we could harvest from as small a package as possible. This build was completed at a different facility, not affiliated with the MAGICC Lab. It was purposefully designed to push 'small' size hardware to its limits, disregarding some of the safety tips above.

DO NOT attempt to disregard the guidelines above. The only thing you will gain is ashes and charcoal. The purpose of **this example is to demonstrate HOW NOT TO DO IT**.



I personally have built an X-8 vehicle, capable of lifting a 65 lb payload. The vehicle itself weighed 25 lb. Total gross thrust was 90 lb. Using a power analyzer, we measured upwards of 115A continuous (depending on the propeller) to each of the eight motors at full throttle. This was our build:

* Connectors:
 * EC5 (battery to splitter to ESC)
 * 5mm bullet connectors (motor to ESC)
* Wire: 10-awg (2 lines in parallel out of battery, before splitter to each of two ESCs)
* Propeller: APC 16x10
* Motor: Great Planes Rimfire 1.20 50-65-450 Outrunner Brushless DC (rated to 85A continuous)
* ESC: KDEXF-UAS95+, rated for 95+A continuous, bursts up to 120+A
* Battery: Pulse Li-Po, 6S, 5000mAh, 65C
 * quantity: 4
 * one battery for each arm, powering two motors
 * total amperage draw from the battery, for two motors at full throttle: ~210A
 * battery amperage rating: 325A
 * soldered splitters from the battery to two ESCs, using 10-awg wire to each of the ESCs
   * This was a pain, because you have two wires, both 10-awg, that have to solder to the positive terminal of the male connector, as well as two *additional* 10-awg wires soldered to the negative terminal of the male connector. This male connector then plugs into the female connector on the battery
   * When splitting power from a battery, be 110% certain ALL joints are well soldered. A cold joint can raise resistance, leading to a power imbalance to the different ESCs, and can lead to inadvertant overloading of one of the ESCs
   * Be sure to adequately cover all joints with heat shrink for safety
