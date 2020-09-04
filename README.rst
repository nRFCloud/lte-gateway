.. _apricity_gateway:

nRF9160: Apricity Gateway
#########################

The Apricity Gateway demonstrates how to use the :ref:`lib_nrf_cloud` to connect an nRF9160-based board to the `nRF Cloud`_ via LTE, connnect to multiple Bluetooth LE peripherals, and transmit their data to the cloud.
Therefore, Apricity Gateway acts as a gateway between Bluetooth LE and the LTE connection to the nRF Cloud.

Overview
********

The application uses the LTE link control driver to establish a network connection.
It is then able to connect to multiple Bluetooth LE peripherals, and transmits the peripheral data to Nordic Semiconductor's cloud solution, `nRF Cloud`_.
The data is visualized in nRF Cloud's web interface.

The `LTE Link Monitor`_ application, implemented as part of `nRF Connect for Desktop`_  can be used to send AT commands to the device and receive the responses.
You can also send AT commands from the **Terminal** card on nRF Cloud when the device is connected.

By default, the Apricity Gateway supports firmware updates through :ref:`lib_aws_fota`.

.. _apicity_gateway_requirements:

Requirements
************

* The following board:

  * |Apricity Gateway nRF9160|

* :ref:`lte-gateway-ble` must be programmed to the nRF52 board controller on the board.
* .. include:: /includes/spm.txt


.. _apricity_gateway_user_interface:

User interface
**************

The button has the following functions:

Button:
    * Reset device when held for 7 seconds.
    * Power off device when held for more than one second and released before reset.

The application state is indicated by the LEDs.

.. _apricty_gateway_operating_states:


.. list-table::
   :header-rows: 1
   :align: center

   * - LED 1 color
     - State
   * - White Pulse
     - Connecting to network
   * - White Pulse
     - Connecting to the nRF Cloud
   * - Yellow Pulse
     - Waiting for user association
   * - Blue Pulse
     - Connected, ready for BLE connections
   * - Red
     - Error

   * - LED 2 color
     - State
   * - White Pulse
     - Waiting for Bluetooth LE connection
   * - White Solid
     - Bluetooth LE connection established

Building and running
********************

.. |sample path| replace:: :file:`lte-gateway`

.. include:: /includes/build_and_run_nrf9160.txt

Testing
=======

After programming the application and all prerequisites to your board, test the Apricity Gateway application by performing the following steps:

1. Connect the board to the computer using a USB cable.
   The board is assigned a COM port (Windows) or ttyACM device (Linux), which is visible in the Device Manager.
#. Connect to the board with a terminal emulator, for example, LTE Link Monitor.
#. Reset the board.
#. Observe in the terminal window that the board starts up in the Secure Partition Manager and that the application starts.
   This is indicated by output similar to the following lines::

      SPM: prepare to jump to Non-Secure image
      *** Booting Zephyr OS build v2.3.0-rc1-ncs1  ***
      Application started
#. Reconnect terminal. (Bluetooth LE HCI control resets the terminal output and needs to be reconnected)
#. Observe in the terminal window that the connection to the nRF Cloud is established. This may take several minutes.
#. Open a web browser and navigate to https://nrfcloud.com/.
   Follow the instructions to set up your account and add an LTE device.
#. The first time you start the application, add the device to your account:

   a. Observe that the LED(s) indicate that the device is waiting for user association.
   #. Follow the instructions on `nRF Cloud`_ to add your device.
   #. If association is successful, the device reconnects to nRF Cloud.
      If the LED(s) indicate an error, check the details of the error in the terminal window.
      The device must be power-cycled to restart the association procedure.
#. Observe that the LED(s) indicate that the connection is established.
#. Observe that the gateway count on your nRF Cloud dashboard is incremented by one.
#. Select the device from your peripheral device list on nRF Cloud, and observe that Bluetooth LE information is recieved from your device.
#. Read, write, and enable notifications on connected peripheral and observe data being received on the nRF Cloud. 
#. Optionally send AT commands from the terminal, and observe that the response is received.


Dependencies
************

This application uses the following |NCS| libraries and drivers:

* :ref:`lib_nrf_cloud`
* :ref:`modem_info_readme`
* :ref:`at_cmd_parser_readme`
* ``drivers/nrf9160_gps``
* ``lib/bsd_lib``
* :ref:`dk_buttons_and_leds_readme`
* ``drivers/lte_link_control``
* ``drivers/flash``
* ``bluetooth/gatt_dm``
* ``bluetooth/scan``

From Zephyr:
  * :ref:`zephyr:bluetooth_api`

In addition, it uses the Secure Partition Manager sample:

* :ref:`secure_partition_manager`

For nrf52840
* :ref:`lte-gateway-ble`


History
************

The Apricity Gateway application was created using the following |NCS| sample applications:

  * :ref:`lte_ble_gateway`
  * :ref:`asset_tracker`

From Zephyr:
  * :ref:`zephyr:bluetooth-hci-uart-sample`