Setting Up Heron's Network
===========================

Heron is equipped with a Microhard wireless module to allow wireless connectivity.  Depending on the confguration
ordered, this can be configured to operate as a wireless access point on the robot itself, or it can be configured
to connect to a portable base-station that remains on shore while the robot operates.


Setup
-------

If this is your first time unboxing Heron, makse sure that the robot's wireless antennas are firmly connected to the
robot.  The connections for the antennas have rubber seals in them to ensure they remain water resistant.  If the
antennas are loose this may allow water ingress, which can inhibit the proper function of the robot.


Changing the Default Password
-----------------------------

.. Note::

  All Clearpath robots ship from the factory with their login password set to ``clearpath``.  Upon receipt of your
  robot we recommend changing the password.

To change the password to log into your robot, run the

.. code-block:: bash

  passwd

command.  This will prompt you to enter the current password, followed by the new password twice.  While typing the
passwords in the ``passwd`` prompt there will be no visual feedback (e.g. "*" characters).

To further restrict access to your robot you can reconfigure the robot's SSH service to disallow logging in with a
password and require SSH certificates to log in.  This_ tutorial covers how to configure SSH to disable password-based
login.

.. _This: https://linuxize.com/post/how-to-setup-passwordless-ssh-login/


Connecting to a Base Station
------------------------------

If your robot came with a base station you can connect your computer to the base station's wireless network using
your computers normal tools.  The robot will connect to the base station automatically when both are powered-on and
in-range of each other.  The following IP addresses will be used by the base station:

+---------------------+----------------------------------+
|IP Address/Range     | Use                              |
+=====================+==================================+
| 192.168.131.1       | The Heron itself                 |
+---------------------+----------------------------------+
| 192.168.131.50      | The base station                 |
+---------------------+----------------------------------+
| 192.168.131.100-254 | DHCP range for connected devices |
+---------------------+----------------------------------+

The SSID of the base station will look something like ``<robot's hostname>-base-station``, for example
``cpr-m300-1234-base-station``.


Connecting Directly to the Heron
----------------------------------

If your Heron did not ship with a base station then you can connect directly to the robot in much the same way.
Open your computer's wireless network manager.  The network's SSID will be something of the form ``<robot's hostname>``,
for example ``cpr-m300-1234``.

When operating in access-point mode, the following IP addresses are used:

+---------------------+----------------------------------+
|IP Address/Range     | Use                              |
+=====================+==================================+
| 192.168.131.1       | The Heron itself                 |
+---------------------+----------------------------------+
| 192.168.131.100-254 | DHCP range for connected devices |
+---------------------+----------------------------------+


Network Configuration on the Heron
------------------------------------

Depending on when your heron shipped, the networking may be configured in one of two possible ways: using
``/etc/network/interfaces`` or using ``netplan``.  Most older Herons will use the ``interfaces`` file, while newer
ones have switched to ``netplan``, as this has become the standard method of configuring networks in more recent
releases of Ubuntu.


**Network configuration with Netplan**

Netplan replaced the older ``ifupdown`` and ``/etc/network/interfaces`` method of configuring network interfaces in
Ubuntu 18.04.  It is possible to revert to the older method if desired, though this is not recommended.  For
instructions on the older network configration method, see below.

Netplan uses yaml files to configure the network interfaces of the robot.  On Heron, this file is located at
``/etc/netplan/50-clearpath-bridge.yaml``.  The contents of the file should look like this:

.. code-block:: yaml

    # Configure the wired ports to form a single bridge
    # We assume wired ports are enp* or eth*
    # This host will have address 192.168.131.1
    network:
      version: 2
      renderer: networkd
      ethernets:
        bridgeports:
          match:
            name: (enp*)|(eth*)
          dhcp4: no
          dhcp6: no
      bridges:
        br0:
          dhcp4: no
          interfaces: [bridgeports]
          addresses:
            - 192.168.131.1/24
          nameservers:
            addresses:
              - 8.8.8.8
              - 8.8.4.4
          gateway4: 192.168.131.50

The file above is for a Heron with a base-station.  If your robot does not have a base-station the last line
``gateway4: 192.168.131.50`` will be omitted.


**Network configuration with /etc/network/interfaces**

.. note::

    If your Heron is already configured to use ``netplan`` we do not advise rolling back to ``interfaces``; we have had
    reports of connectivity problems with base-stations on Herons running Ubuntu 18.04 when using the ``interfaces``
    file to configure the network.

On Ubuntu 18.04 you can revert to the older ``/etc/network/interfaces`` method of configuring the network interfaces by
running

.. code-block:: bash

    sudo apt-get install ifupdown

The network interfaces configuration file, located at ``/etc/network/interfaces`` should contain the following:

.. code-block:: text

    auto lo br0 br0:0
    iface lo inet loopback

    # Bridge together physical ports on machine, assign standard Clearpath Robot IP.
    iface br0 inet static
      bridge_ports regex (eth.*)|(en.*)
      address 192.168.131.1
      netmask 255.255.255.0
      bridge_maxwait 0
      # if you do not have a base-station, omit the following
      gateway 192.168.131.50
      dns-nameservers 8.8.8.8 8.8.4.4

    # Also seek out DHCP IP on those ports, for the sake of easily getting online,
    # maintenance, ethernet radio support, etc.
    iface br0:0 inet dhcp


Remote ROS Connection
---------------------

To use ROS desktop tools, you'll need your computer to be able to connect to Heron's ROS master. This can be a
tricky process, but we've tried to make it as simple as possible.

In order for the ROS tools on your computer to talk to Heron, they need to know two things:

- How to find the ROS master, which is set in the ``ROS_MASTER_URI`` environment variable, and
- How processes on the other computer can find *your computer*, which is the ``ROS_IP`` environment variable.

The suggested pattern is to create a file in your home directory called ``remote-heron.sh`` with the following
contents:

.. code-block:: bash

    export ROS_MASTER_URI=http://cpr-heron-0001:11311  # Heron's hostname
    export ROS_IP=10.25.0.102                          # Your laptop's wireless IP address

If your network doesn't already resolve Heron's hostname to its wireless IP address, you may need to add
a corresponding line to your computer's ``/etc/hosts`` file:

.. code-block:: bash

    10.25.0.101 cpr-heron-0001

Then, when you're ready to communicate remotely with Heron, you can source that script like so, thus defining
those two key environment variables in the present context.

.. code-block:: bash

    source remote-heron.sh

Now, when you run commands like ``rostopic list``, ``rostopic echo``, ``rosnode list``, and others, the output
you see should reflect the activity on Heron's ROS master, rather than on your own machine. Once you've
verified the basics (list, echo) from the prompt, try launching some of the standard visual ROS tools:

.. code-block:: bash

    roslaunch heron_viz view_robot.launch
    rosrun rqt_robot_monitor rqt_robot_monitor
    rosrun rqt_console rqt_console

If there are particular :roswiki:`rqt` widgets you find yourself using a lot, you may find it an advantage to dock them together
and then export this configuration as the default RQT perspective. Then, to bring up your standard GUI, you can simply
run:

.. code-block:: bash

    rqt


Legacy Connectivity
---------------------

Older Herons may have shipped with a combination Wifi/Bluetooth module, for example an
`Intel Centrino Advanced-N 6235`__, instead of the Microhard.

.. _Centrino: http://www.intel.com/content/www/us/en/wireless-products/centrino-advanced-n-6235.html
__ Centrino_


**Connecting to Wifi Access Point**

Heron's standard wireless network manager is wicd_. To connect to an access point in your lab, run:

.. code-block:: bash

    wicd-curses

You should see a browsable list of networks which the robot has detected. Use arrow keys to select the one you
would like to connect to, and then press the right arrow to configure it. You can enter your network's password
near the bottom of the page, and note that you must select the correct encryption scheme; most modern networks
use ``WPA1/2 Passphrase``, so if that's you, make sure that option is selected. You also likely want to select
the option to automatically reconnect to this network, so that Heron will be there for you on your wireless
automatically in the future.

When you're finished, press F10 to save, and then C to connect.

Wicd will tell you in the footer what IP address it was given by your lab's access point, so you can now log out,
remove the network cable, and reconnect over wireless. When you've confirmed that all this is working as expected,
close up Heron's chassis.

.. _wicd: https://launchpad.net/wicd


.. _remote:
