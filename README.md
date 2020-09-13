# Movesense Gatt

Custom GATT service for retrieving sensor data (9-axis IMU) from a wearable Movesense sensor.

## Description

This custom GATT service serves as a way of retrieving data from a wearable Movesense sensor through Bluetooth GATT. Developed within a degree project at [KTH Royal Institute of Technology](https://www.kth.se/en).

## Getting Started

### Dependencies

* [Movesense Getting Started with Sensor SW](http://www.movesense.com/docs/esw/getting_started/)

### Installing

#### Build the application
```
cmake -G Ninja -DMOVESENSE_CORE_LIBRARY=../MovesenseCoreLib/ -DCMAKE_TOOLCHAIN_FILE=../MovesenseCoreLib/toolchain/gcc-nrf52.cmake -DCMAKE_BUILD_TYPE=Release ../movesense_gatt_service  
```
```
ninja
```

#### Over the air update
```
ninja dfupkg
```

### Executing program

The application will start running once successfully uploaded to a Movesense sensor. 

## Authors

[jonlundv](jonlundv@kth.se)/[DelBocaVista](https://github.com/DelBocaVista/)

## Acknowledgments

* [custom_gattsvc_app](https://bitbucket.org/suunto/movesense-device-lib/src/master/samples/custom_gattsvc_app/)
