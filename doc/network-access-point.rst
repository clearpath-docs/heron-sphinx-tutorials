Networking with Heron as a Wireless Access Point
====================================================

Heron is capable of operating as its own access point, allowing computers to connect directly to it without the use
of a base station.  This mode of operation has the advantage of requiring less hardware in the field, but has the
down-side of making it harder to connect Heron to the internet.

If your Heron did not ship with a base station it will come pre-configured to operate as an access point.


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
