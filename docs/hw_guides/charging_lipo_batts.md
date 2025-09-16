# Li-Po Battery Charger User Guide #

Attention!
Please charge responsibly.

Attention!
Please read through this __*entire*__ document before you begin the process of following the instructions in each line item.

Note:
**MAGICC Safety** is an individual __*and*__ a community responsibility. There is __nothing__ *magic* about it.


1. Read the MAGICC Lab Wiki page on [Safety]: __magicc.byu.edu/wiki/safety__
1. Read the MAGICC Lab Wiki page on [Batteries]: __magicc.byu.edu/wiki/hw_guides/lipo_batts__
1. Internalize the Safety page - etch it onto your brainstem
1. Internalize the Battery page - etch it onto your frontal lobe
1. Place your battery in a metal case atop the charging cases
1. Press "Type" until the display says "Li-Po" (if you get this step wrong, there *will* be a fire)
1. Triple check how many cells your battery has
1. Press "Next" until the display flashes "__X__s" where __X__ is the number of cells in your battery
1. Press the up/down arrows until the number of cells of your battery is displayed
1. Press "Next"
1. Triple check the capacity of your battery (e.g. 2000mAh, which is also 2Ah)
1. Press the up/down arrows until your battery's capacity is shown
1. Press "Next"
1. Charge rate:
  1. If you are on a short timeline, and need the battery to charge quickly, reassess your timeline. The more often that you charge your battery at a high amperage rate, the shorter the longevity of your battery will be. Please do not waste lab resources more quickly than necessary. If you can charge the battery a little slower, that is always better
  1. Besides, slower charge rates lower risk of fire
  1. For slower charging (but still reasonable) multiply your battery's capacity (in Ah) by 0.5/hour (e.g. for a 2000mAh battery, --> 2Ah * (0.5/h) = 1A)
  1. If you have all day, use a lower multiplier. Trickle charging is best, and safest
  1. For faster charging, multiply your battery's capacity (in Ah) by 1/hour. (e.g. 2000mAh battery --> 2Ah * (1/h) = 2A charge rate)
  1. Some batteries claim to safely charge at a "5C" charge rate. I personally strongly discourage charge rates that high. We can't afford to burn the engineering building to the ground. On such batteries, though, if you absolutely must charge faster than "1C", reconsider your timeline. It may be inconvenient to re-schedule a flight test, but that is infinitely more convenient than the building burning down over your impatience
  1. The MAGICC Lab protocol allows for "2C" max in the most dire of situations (e.g. 2000mAh battery --> 2Ah * (2/hour) = 4A charging on the charger display)
1. Plug the cell wires into the balancer board
1. Plug the main power wires into the charge port
1. Long press the "Start" button
1. Wait for the confirmation that charging has started
1. Put an orange rock in your pocket, or, in the middle of your keyboard (i.e. the point is, somewhere where you will _NOT_ forget about it)
1. Do _NOT_ leave the lab while the battery is charging
  1. If you must leave, even for 1 second, ask another person in the lab to take full responsibility for the battery
  1. If the other person accepts, give the orange rock to them
  1. If they do not accept, you may not leave the lab without first stopping the charge process, and completely unplugging the battery from the charger
1. When the charge cycle completes, verify the voltages are similar across all cells in battery; this can be done by cycling through _MODE_ on the charger display
1. Make sure the charge process has _actually_ stopped (i.e. real-time amperage is 0.0A)
1. Press _STOP_ a few times, just to be sure
1. Unplug the main power connector from your battery
1. Unplug the cell voltage wires from the balancer board
1. You may now return your orange rock to the appropriate location on the charging bench
1. You may now use your charged battery. Please do not drop it, as even dented cells are not worth the risk of burning down the building, or your vehicle. It is cheaper to replace a battery than the building, and batteries are cheaper and easier to replace than most vehicles

<!-- links -->
[Safety]: ../safety.md
[Batteries]: lipo_batts.md
