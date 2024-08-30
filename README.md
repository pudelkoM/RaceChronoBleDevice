# CAN Bus Data Logger for RaceChrono

This project is a high-speed CAN-Bus datalogger for a BMW Z4 E85. It integrates into the onboard network and feeds data to [RaceChrono](https://racechrono.com/) over Bluetooth Low Energy (BLE).

## TODOs

- [ ] Bluetooth security
- [ ] Update rate improvements
  - [ ] Merge CAN messages?
- [x] XY-Axis acceleration data

## BoM

- ASL [ESP32-CAN-X2](https://www.autosportlabs.com/product/esp32-can-x2-dual-can-bus-automotive-grade-development-board/)
- Tulay's Wire Werks [E46 CAN-Bus Plug and Play Adapter](https://tulayswirewerks.com/product/e46-can-bus-plug-and-play-adapter-4-pin-ign/)

## Pictures

![img](images/plug.jpg)

![img](images/can_wires.jpg)

![img](images/racechrono.jpg)

## Android connection priority / BLE connection interval

The BLE stack operates on two connection interval values (min, max), which
effectively determine the maximum possible update rate for CAN-bus messages
transmitted from the ESP32 device to the phone. While we can set the _preferred_
values on the server side with `pAdvertising->setMinPreferred(0x06);`, it's the
client (phone) which ultimately chooses the values. By default, my Android Pixel
4 seems to pick a balanced value resulting in a update rate of ~30-40 Hz. Apps
can explicitly request a priority for a given connection [docs](https://developer.android.com/reference/android/bluetooth/BluetoothGatt#requestConnectionPriority(int)), picking between 4 presets.

In my experiments I used the [nRF Connect](https://play.google.com/store/apps/details?id=no.nordicsemi.android.mcp)
app to both log and set BLE parameters. When following these steps, I could
significantly increase the observed CAN message rate:

0) Enable Bluetooth
1) Open nRF Connect app and connect to data logger
2) From drop-down, open "Request connection priority" and pick `HIGH`
3) Confirm change in logs
4) Open Racechrono
5) Start new recoding session
6) Open sensor menu `^` and confirm "Bluetooth LE CAN-bus" update rate

Here are the different rates observed with different settings:

| Connection Prio | Data Rate [Hz] |
|-----------------|----|
| default         | xx |
| HIGH            | xx |
| BALANCED        | xx |
| LOW_POWER       | xx |

## Credits / References
- https://github.com/autosportlabs/ESP32-CAN-X2
- https://github.com/aollin/racechrono-ble-diy-device
