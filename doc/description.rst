heron_description Package
===========================

The heron_description package is the URDF robot description for Heron USV.

.. _Source: https://github.com/heron/heron


Overview
---------

This package provides a `URDF <http://wiki.ros.org/urdf>`_ model of Heron.  For an example launchfile to use in visualizing this model, see `heron_viz <http://wiki.ros.org/heron_viz>`_.

.. image:: images/heron_urdf.png


Accessories
------------

Heron has a suite of optional payloads called accessories. These payloads can be enabled and placed on Herib using environment variables specified at the time the `xacro <http://wiki.ros.org/xacro>`_ is rendered to URDF. Available accessory vars are:

.. raw:: html

    <table><tbody><tr>  <td><p><strong>Variable</strong> </p></td>
      <td><p><strong>Default</strong> </p></td>
      <td><p><strong>Description</strong> </p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>HERON_URDF_EXTRAS</tt> </p></td>
      <td><p><tt>empty.urdf</tt> </p></td>
      <td><p>Path to a URDF file to add additional joints/links/accessories to Heron's model</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>HERON_IMU_UM6_PORT</tt> </p></td>
      <td><p><tt>/dev/clearpath/imu</tt> </p></td>
      <td><p>The file that the Heron's UM6 IMU is mapped to</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>HERON_IMU_UM6_XYZ</tt> </p></td>
      <td><p><tt>-0.1397 0 0</tt> </p></td>
      <td><p>XYZ offset of the IMU relative to its mount</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>HERON_IMU_UM6_RPY</tt> </p></td>
      <td><p><tt>0 0 -0</tt> </p></td>
      <td><p>RPY offset of the IMU relative to its mount</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>HERON_NAVSAT_UBLOX_PORT</tt> </p></td>
      <td><p><tt>/dev/ublox</tt> </p></td>
      <td><p>The file that the Heron's GPS is mapped to</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>HERON_NAVSAT_UBLOX_BAUD</tt> </p></td>
      <td><p><tt>115200</tt> </p></td>
      <td><p>The baud rate the Heron's GPS uses</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>HERON_NAVSAT_UBLOX_SOCKET</tt> </p></td>
      <td><p><tt>0</tt> </p></td>
      <td><p>If true (1), enable communicating with the GPS over a socket instead of serial</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>HERON_NAVSAT_UBLOX_SOCKET_PORT</tt> </p></td>
      <td><p><tt>2525/tt> </p></td>
      <td><p>The network port to use for communicating with the GPS if <tt>HERON_NAVSAT_UBLOX_SOCKET</tt> is set to <tt>1</tt></p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>HERON_NAVSAT_UBLOX_RTCM</tt> </p></td>
      <td><p><tt>false</tt> </p></td>
      <td><p>???</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>HERON_NAVSAT_UBLOX_XYZ</tt> </p></td>
      <td><p><tt>0.015 0.050 0.072</tt> </p></td>
      <td><p>XYZ offset of the Ublox GPS to its mount</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>HERON_NAVSAT_UBLOX_RPY</tt> </p></td>
      <td><p><tt>0 0 -0</tt> </p></td>
      <td><p>RPY offset of the Ublox GPS relative to its mount</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>HERON_NAVSAT_SMART6</tt> </p></td>
      <td><p><tt>0</tt> </p></td>
      <td><p>Set to <tt>1</tt> if Heron is equipped with a Smart6 GPS</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>HERON_NAVSAT_SMART6_PORT</tt> </p></td>
      <td><p><tt>/dev/clearpath/gps</tt> </p></td>
      <td><p>The file the Smart6 GPS is mapped to</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>HERON_NAVSAT_SMART6_BAUD</tt> </p></td>
      <td><p><tt>57600</tt> </p></td>
      <td><p>The baud rate the Smart6 GPS uses</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>HERON_NAVSAT_SMART6_MOUNT</tt> </p></td>
      <td><p><tt>rear</tt> </p></td>
      <td><p>Prepended to <tt>_navsat</tt> to indicate what link the Smart6 GPS is connected to on the robot</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>HERON_NAVSAT_SMART6_OFFSET</tt> </p></td>
      <td><p><tt>-0.25 0 0.08</tt> </p></td>
      <td><p>XYZ offsets for the Smart6 GPS relative to its mount</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>HERON_NAVSAT_SMART6_RPY</tt> </p></td>
      <td><p><tt>0 0 0</tt> </p></td>
      <td><p>RPY offsets for the Smart6 GPS relative to its mount</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>HERON_NAVSAT_SMART6_RTK</tt> </p></td>
      <td><p><tt>0</tt> </p></td>
      <td><p>???</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>HERON_NAVSAT_SMART6_RTK_DEVICE</tt> </p></td>
      <td><p><tt>wlan0</tt> </p></td>
      <td><p>???</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>HERON_LASER</tt> </p></td>
      <td><p><tt>0</tt> </p></td>
      <td><p>Set to <tt>1</tt> if Heron is equipped with a lidar unit</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>HERON_LASER_HOST</tt> </p></td>
      <td><p><tt>192.168.131.14</tt> </p></td>
      <td><p>The internal IP address of the Heron's lidar</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>HERON_LASER_MOUNT</tt> </p></td>
      <td><p><tt>front</tt> </p></td>
      <td><p>Prepended to <tt>_laser</tt> to indicate the link the lidar is mounted to</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>HERON_LASER_OFFSET</tt> </p></td>
      <td><p><tt>0.0575 0 0.0115</tt> </p></td>
      <td><p>XYZ offsets for the lidar relative to its mount</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>HERON_LASER_RPY</tt> </p></td>
      <td><p><tt>0 0 0</tt> </p></td>
      <td><p>RPY offsets for the lidar relative to its mount</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>HERON_LASER_TOPIC</tt> </p></td>
      <td><p><tt>front/scan</tt> </p></td>
      <td><p>The ROS topic the lidar data is published to</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>HERON_AXIS_PTZ</tt> </p></td>
      <td><p><tt>0</tt> </p></td>
      <td><p>Set to <tt>1</tt> if Heron is equipped with PTZ camera</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>HERON_AXIS_PTZ_NAME</tt> </p></td>
      <td><p><tt>axis_ptz</tt> </p></td>
      <td><p>ROS namespace for topics published by the PTZ camera</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>HERON_AXIS_PTZ_HOST</tt> </p></td>
      <td><p><tt>192.168.131.13</tt> </p></td>
      <td><p>Internal IP address of the PTZ camera</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>HERON_AXIS_PTZ_WIDTH</tt> </p></td>
      <td><p><tt>1280</tt> </p></td>
      <td><p>Horizontal resolution of the PTZ camera</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>HERON_AXIS_PTZ_HEIGHT</tt> </p></td>
      <td><p><tt>720</tt> </p></td>
      <td><p>Vertical resolution of the PTZ camera</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>HERON_AXIS_PTZ_ENABLE_PTZ</tt> </p></td>
      <td><p><tt>1</tt> </p></td>
      <td><p>Set to <tt>0</tt> to disable the PTZ controls and use a fixed-position camera</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>HERON_AXIS_PTZ_ENABLE_THEORA</tt> </p></td>
      <td><p><tt>0</tt> </p></td>
      <td><p>Set to <tt>1</tt> to enable Ogg/Theora video streaming from the camera</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>HERON_AXIS_PTZ_ENABLE_TELEOP</tt> </p></td>
      <td><p><tt>1</tt> </p></td>
      <td><p>Set to <tt>0</tt> to disable remote teleop control of the PTZ functions</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>HERON_PTZ_MOUNT</tt> </p></td>
      <td><p><tt>axis_ptz</tt> </p></td>
      <td><p>Prepended to <tt>_camera_link</tt> to indicate the link the PTZ camera is mounted to</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>HERON_PTZ_OFFSET</tt> </p></td>
      <td><p><tt>-0.124 0 0.078</tt> </p></td>
      <td><p>XYZ offsets for the PTZ camera relative to its mount</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>HERON_PTZ_RPY</tt> </p></td>
      <td><p><tt>0 0 0</tt> </p></td>
      <td><p>RPY offsets for the PTZ camera relative to its mount</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>HERON_SONAR</tt> </p></td>
      <td><p><tt>0</tt> </p></td>
      <td><p>Set to <tt>1</tt> if Heron is equipped with underwater sonar</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>HERON_SONAR_PORT</tt> </p></td>
      <td><p><tt>/dev/ttyS4</tt> </p></td>
      <td><p>The serial port the sonar is connected to</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>HERON_SONAR_BAUD</tt> </p></td>
      <td><p><tt>19200</tt> </p></td>
      <td><p>The baud rate the sonar uses</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>HERON_SONAR_ROS</tt> </p></td>
      <td><p><tt>1</tt> </p></td>
      <td><p>Set to <tt>0</tt> to disable publishing sonar data as ROS topics</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>HERON_SONAR_MOUNT</tt> </p></td>
      <td><p><tt>rear</tt> </p></td>
      <td><p>Indicates where the sonar is mounted to the robot</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>HERON_SONAR_OFFSET</tt> </p></td>
      <td><p><tt>0 0 -0.1</tt> </p></td>
      <td><p>XYZ offset of the sonar relative to its mount</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>HERON_SONAR_RPY</tt> </p></td>
      <td><p><tt>0 0 -0</tt> </p></td>
      <td><p>RPY offset of the sonar relative to its mount</p></td>
    </tr>
    </tbody></table>
