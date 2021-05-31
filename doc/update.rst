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


.. _scratch:

Starting From Scratch
---------------------

.. warning::

    Reinstalling Heron's OS will reformat the harddrive inside the robot.  Be sure to
    `back up <http://www.clearpathrobotics.com/assets/guides/kinetic/kinetic-to-melodic/create-backup.html>`_ your
    robot's configuration before reformatting the robot.

If Heron's computer has become inoperable, or for any reason you want to restore it to the factory state, begin
by opening Heron, lowering the computer tray, and connecting a screen and keyboard, as well as a wired internet
connection. You can then download the most recent version of the Heron boot ISO from the following location:

http://packages.clearpathrobotics.com/stable/images/latest/

Use unetbootin or a similar tool to flash the ISO image to a USB memory stick.

To access Heron's PC you must open the electronics enclosure.  This enclosure is tightly sealed with screws and contains
a rubber gasket to repel water.  Be very careful when opening and closing the enclosure that the gasket remains
undisturbed; cables pinched in the gasket, tears, or debris can affect its ability to repel water, which can cause
damage to Heron's electronics.

Once the PC is exposed, insert the USB drive you flashed earlier into a USB port.  You may disconnect sensors during
this process if there are no free USB ports (just make sure to reconnect them before sealing the robot back up).  You
must also connect a monitor to the VGA/HDMI/DisplayPort on the PC (different PCs will have different video outputs, so
use whatever is appropriate for your Heron) and connect a keyboard to a USB port.

Finally, Heron needs a wired internet connection with DHCP to download software during the installation.  If your Heron
has a MicroHard long-range radio, disconnect it from the Heron's PC.  Plug an ethernet cable into one of the Heron PC's
available ethernet ports.

Once the monitor, keyboard, ethernet cable, and USB drive are all connected to Heron's PC, power-on the computer and
boot from the USB drive.  The installer will proceed with minimal user input and will power off the computer when
finished.

The installer will attempt to connect to the internet using the PC's lowest-indexed ethernet port; if you encounter
DHCP failures when booting from the ISO, try connecting the ethernet cable to a different port.

After the installer is finished, remove the USB drive and power-on the Heron.  The installer will automatically create
the `administrator` user with the password `clearpath`.  Log in and run the following command:

.. code-block:: bash

    rosrun heron_bringup install

This installs Heron's `robot_upstart`_ job, so that ROS starts each time the robot starts.

.. _robot_upstart: http://wiki.ros.org/robot_upstart

At this point you can restore your backed-up data.  If your robot has customized URDFs, additional sensors or other
payloads, or custom packages defined in a catkin workspace you must restore these on your own.
