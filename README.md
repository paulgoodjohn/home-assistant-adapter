# home-assistant-adapter for older GEA2 appliances

Based upon the example for GEA3 that subscribes to all ERD changes, this is a modified version using the older, slower GEA2 communications. It runs on the ESP32C3-based Home Assistant adapter [available from FirstBuild](https://firstbuild.com/inventions/home-assistant-adapter/).

## Operation

Since GEA2 does not provide a simple way to subscribe to all ERD changes like GEA3 does, the way this code works is as follows:

- Initially, if the non-volatile store is empty it asks whatever it is connected to for the mandatory ERD 0x0008 (Appliance type). This gives 2 things - first, the reply message contains the GEA address of the machine control, since only it can reply to that read. Second, it indicates the family that the appliance belongs to which allows the number of ERDs to be checked to be vastly reduced. This step is repeated every 3 seconds until a valid response is received.
- Now talking only to the address of the machine control board, all the common ERDs are read, and if the machine control replies, the ERD number is added to a poll list.
- The energy ERDs are next - this is a list of common energy reporting ERDs.
- Finally the appliance specific group is read, and all the ERDs that respond are added to the poll list.
- The complete list of ERDs that can be read (the poll list) is then saved off to non-volatile memory, along with the number of ERDs to poll and the GEA address to read for the machine control.
- Finally, the code then loops round polling every ERD on the list. Write operations are slotted into the stream of read operations, and rely on the buffering in the GEA2 stack.
- If no ERD can be read for 60 seconds, the non-volatile memory is cleared, and the code returns to looking for ERD 0x0008.
- If the non-volatile memory contains valid poll list data at power up, the code goes straight to polling those stored ERDs.

## Hardware

The Home Assistant adapter consists of a [Xiao ESP32C3](https://wiki.seeedstudio.com/XIAO_ESP32C3_Getting_Started/) and [carrier board](doc/schematic-v1.0.pdf) that breaks out the serial interface of the Xiao to an RJ45 jack.

## Setup

- Install [PlatformIO](https://platformio.org/)
- Copy `config/Certificate.h.sample` to `config/Certificate.h` and add your certificate (if any)
- Copy `config/Config.h.sample` to `config/Config.h` and add your WiFi credentials, MQTT configuration, and your device ID

In-depth instructions can be found in the [Getting Started](doc/getting-started.md) guide.

## Usage

### Build

Builds the firmware into `.pio/build/xiao_c3/firmware.bin`.

```shell
make
```

### Clean

Deletes all build artifacts.

```shell
make clean
```

### Upload

Uploads/flashes the firmware to the ESP32-C3. Note that the board may need to be reset into boot loader mode by holding the B (boot) button and pressing the R (reset) button.

```shell
make upload
```

### (Serial) Monitor

Opens the PlatformIO serial monitor to view a connected ESP32-C3's serial output.

```shell
make monitor
```

## Example Home Assistant Configuration

Sample yaml can be found in https://github.com/geappliances/home-assistant-examples
