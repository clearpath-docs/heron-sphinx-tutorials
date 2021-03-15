Keeping Heron Updated
======================

. note:: If you are upgrading your Heron from an older version of ROS, please refer to `our upgrade instructions here <https://clearpathrobotics.com/assets/guides/kinetic/kinetic-to-melodic/index.html>`_.

Heron is always being improved, both its own software and the many community ROS packages upon which it
depends! You can use the apt package management system to receive new versions all software running on the
platform.


Getting New Packages
--------------------

Each Heron leaves the factory already configured to pull packages from http://packages.ros.org as well as
http://packages.clearpathrobotics.com. To update your package and download new package versions, simply run:

.. code-block:: bash

    sudo apt-get update
    sudo apt-get dist-upgrade

If you see any errors, please `get in touch`_ and we'll see if we can get you sorted out.

.. _get in touch: https://support.clearpathrobotics.com/hc/en-us/requests/new


MCU Firmware Update
-------------------

.. note::

    TODO

.. _scratch:

Starting From Scratch
---------------------

If Heron's computer has become inoperable, or for any reason you want to restore it to the factory state, begin
by opening Heron, lowering the computer tray, and connecting a screen and keyboard, as well as a wired internet
connection. You can then download the most recent version of the Heron boot ISO from the following location:

http://packages.clearpathrobotics.com/stable/images/latest/

Use unetbootin or a similar tool to flash the ISO image to a USB memory stick. Boot Heron's computer with the USB
memory connected, and you should be in the purple Debian/Ubuntu installer. The installer runs by itself and shuts
down the computer when finished.

Once done, turn Heron on once more, and run the following:

.. code-block:: bash

    rosrun heron_bringup install

This installs Heron's `robot_upstart`_ job, so that ROS starts each time the robot starts.

.. _robot_upstart: http://wiki.ros.org/robot_upstart

Note that if you may need to re-pair your gamepad to the robot, and you'll have some extra work to do if you have
integrated accessories which require additional launchers or URDF.
