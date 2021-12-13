Heron's Lights
===============

Heron's port and starboard hulls are equipped with red and green navigation lights.  Under normal, autonomous operation
these lights will be solid.  These lights will flash to indicate certain high-priority states, such as a low-battery
warning or a failure to communicate with the Heron's base-station wireless network.

The table below shows animations of the LED patterns and their meanings.

.. |lights_low_battery| image:: images/lights/low_battery.gif
    :width: 50px
    :height: 100px

.. |lights_manual| image:: images/lights/manual.gif
    :width: 50px
    :height: 100px

.. |lights_no_cmd| image:: images/lights/no_cmd.gif
    :width: 50px
    :height: 100px

.. |lights_no_pc| image:: images/lights/no_computer.gif
    :width: 50px
    :height: 100px

.. |lights_off| image:: images/lights/no_lights.gif
    :width: 50px
    :height: 100px

.. |lights_no_wifi| image:: images/lights/no_wifi.gif
    :width: 50px
    :height: 100px

.. |lights_ok| image:: images/lights/ok.gif
    :width: 50px
    :height: 100px

+-----------------------+-------------------+------------------------------------------------------------------------+
| Light Pattern         | Description       | Meaning                                                                |
+=======================+===================+========================================================================+
| |lights_ok|           | Solid on          | Robot is operating normally                                            |
+-----------------------+-------------------+------------------------------------------------------------------------+
| |lights_off|          | Solid off         | Robot is powered-off, or operating normally with the lights turned off |
+-----------------------+-------------------+------------------------------------------------------------------------+
| |lights_low_battery|  | Fast double blink | Battery is low. Return to shore immediately to recharge                |
+-----------------------+-------------------+------------------------------------------------------------------------+
| |lights_manual|       | Fast single blink | Robot is being driven manually via the remote control                  |
+-----------------------+-------------------+------------------------------------------------------------------------+
| |lights_no_cmd|       | Slow single blink | Robot is idle, awaiting commands                                       |
+-----------------------+-------------------+------------------------------------------------------------------------+
| |lights_no_pc|        | Slow triple blink | Failure to establish communication between the PC and MCU              |
+-----------------------+-------------------+------------------------------------------------------------------------+
| |lights_no_wifi|      | Slow double blink | Failure to communicate with the base-station, or no wireless network   |
+-----------------------+-------------------+------------------------------------------------------------------------+

During normal operation the lights can be turned-off by publishing to the ``disable_lights`` topic.  This will
suppress the lights during normal operation.
