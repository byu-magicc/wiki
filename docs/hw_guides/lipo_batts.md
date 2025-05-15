# Lithium-Polymer (Li-Po) Batteries #

## General Hardware Thoughts and Tips ##
#### From: Tools Tutorials ####

This module is meant to help learn how to use the hardware we use in this lab. There is really no particular order in which you need to complete these tutorials, complete them as you find necessary in your work.

If you intend to become seriously involved in building and/or flying aircraft in the lab, it is worth noting that you should demonstrate some experience before working on lab equipment. This may include purchasing your own components, building your own airframes and getting practice on your own. Members of the lab can be a great resource for getting help in choosing components and building airframes, but lab resources are usually not best served in teaching RC flying or building aircraft. The best teacher in this situation is experience, and if you want to gain mastery over these concepts, you will have to purchase your own parts. Although it can seem overwhelming to make the investment to purchase your own parts, my experience is that it's generally much more fulfilling and educational to have built and flown your own airplane or multirotor with your own parts. Making the financial investment makes that experience all that much more impactful.

Given that, there are still a lot of opportunities for you to participate in building and working with hardware, even if you don't maintain the entire aircraft, so it's really important to understand how all of the parts function, which is why we have these tutorials.

## History ##

#### **Did you know that the MAGICC lab used to be in the Clyde Building?** ####

Well, it was! However, several years ago a fire started at the battery charging station and as a result, the lab was banished to the basement of B-34 (a more dispensable building). After good behavior, the lab was moved to the Fletcher Building.

#### **Moral of the Story:** ####

is that **if we want to ever have good air conditioning and heating again** we need to be careful when charging batteries.

## Guidelines ##

Below are a few guidelines. If you have any questions, **ask** someone in the lab who knows and then update this wiki.

#### Connectors ####
Any 'hot' or 'source' line should always have a female connector. This ~~protects~~ makes it more difficult for the user to inadvertantly touch the 'hot'/'source' leads and get shocked. This is why wall outlets are female. Unless someone has something skinny and metal, deliberately aiming for the small hole, they won't get shocked. The male plug of an extension cord is only exposed when it is disconnected from a power outlet, and hence cannot shock you when you grab the exposed wires. Once you plug it in, there are no external wires to inadvertently grab.

So, make sure the battery gets a female connector. Likewise, the bullet connectors coming out of the ESC should also be female, because as soon as the ESC is plugged into the battery, the ESC bullets can be 'hot'. Additionally, make sure these female bullet connectors are entirely covered by heat shrink, but not so much that it inhibits the full connection with the male bullets attached to the motor.

Attention!
Never cut the postive and negative wires of the battery at the same time (e.g. when replacing a connector, whether it is the main battery lead or the cell voltage-tester-lead).

Most connectors are labeled with a positive and negative sign on the appropriate sides, to avoid reverse polarization at the soldering stage. Be aware of which side of the connector to solder the black wire to (negative), and the red wire (positive). **If you realize you did it wrong by accident, start over. Do not purposefully solder a matching female connector to an ESC the wrong way, thinking you'll always remember that that battery and ESC must always go together. [Fires Will Happen](#moral-of-the-story). De-solder it and start over.**

* Some connector types (Anderson Powerpoles) are 'genderless.' This means that both sides of the connector are identical, or in other words, neither side has an exposed plug wire. Both are inside a housing, difficult to get accidentally electrically shocked if 'hot.' The MAGICC lab does not use these types of connectors (that I know of), so just get used to being careful and deliberate when soldering connectors. Even with 'genderless' connectors, **they are still polarized**, so make sure the red and black wires are soldered to the correct sides of the connector

#### Li-Po (Lithium-Polymer) Batteries: ####
* 3S - 3 cells in series
* 4S - 4 cells in series
* 2P - 2 cells in parallel


#### Voltage (V): ####
* Batteries have nominal charge of 3.7V/cell
* **WITH NO LOAD, BATTERY SHOULD NOT BE DISCHARGED BELOW THIS!**
* **IF IT IS, RECHARGE BATTERIES IMMEDIATELY.**
  * e.g.
    * 3S is 11.1V nominally
    * 4S is 14.8V nominally

* Batteries when fully charged are 4.2V/cell
  * e.g.
    * 3S is 12.6V fully charged
    * 4S is 16.8V fully charged
* Cells should be stored between 3.75 and 3.85 V
   * Storage meaning anything longer than a day

* At any time, if the voltage in one cell is more than 0.5V different than any
  other, the entire battery should be discarded. This condition is called cell
  balance. All cells need to be balanced regularly, and checked that they have
  not entered a state of imbalance. If you try to charge an imbalanced battery,
  you run a very high risk of explosion.


#### Charging: ####

More info in the [Battery Charger User Guide].

**Attention:** IF YOU DO NOT KNOW HOW TO USE THE CHARGER, ASK FOR HELP

**Attention:** __*REQUIRED READING:*__ More charger-specific info in the [Battery Charger User Guide].

Be absolutely certain the charge mode of the charger matches the type of cell you are charging. Telling the charger it is connected to a Ni-MH battery, and actually connecting a Li-Po battery WILL cause irreparable damage to the equipment AND BUILDING.

* Capacity is measured in milli-amp hours (mAh)
* The 'C' rating is just a multiple of battery capacity
  * e.g.
    * A '25C' max discharge rate on a 5,000mAh battery means the battery is rated for a max discharge rate of 125A
    * A '1C' max charge rate on a 5,000mAh battery means the battery is rated to safely charge at 5A
* Some battery specifications may advertise up to 5C or 10C charge rates, but **did you read the [MORAL OF THE STORY](#moral-of-the-story) above?** High charge rates can be dangerous, and can cause expensive-to-fix damage -- more than you can afford to cover -- hope you either have good insurance or alternate career plans...
* In the MAGICC lab we only charge batteries at '1C' max, even though the batteries may have a higher advertised rating on its spec sheet
  * If you have the time, charge at '0.5C' (or lower) as it will be easier on the battery, and help prolong the life of the cell
  * High charge rates (and high discharge rates, for that matter) degrade cell quality more quickly, which causes otherwise unnecessary battery purchases with sacred funds provided to the lab
  * The degradation level of the cells can most easily be seen by the 'puffiness' of the cells
  * e.g.
    * A 5,000mAh battery is charged at 5A **maximum**

<!-- links -->
[Battery Charger User Guide]: charging_lipo_batts.md
