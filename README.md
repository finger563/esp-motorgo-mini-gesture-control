# ESP MotorGo Mini Gesture Control

This repository contains sample code for the [MotorGo Mini
board](https://motorgo.net) (ESP32-S3) which receives control commands via
esp-now to configure and command the two motors of the MotorGo Mini.

An example control / transmitter app can be found at
[esp-box-motor-gesture-control](https://github.com/finger563/esp-box-motor-gesture-control).

https://github.com/user-attachments/assets/5c0ec87a-81c8-4247-a4b3-855d4f7b4968

## Cloning

Since this repo contains a submodule, you need to make sure you clone it
recursively, e.g. with:

``` sh
git clone --recurse-submodules https://github.com/finger563/esp-motorgo-mini-gesture-control
```

Alternatively, you can always ensure the submodules are up to date after cloning
(or if you forgot to clone recursively) by running:

``` sh
git submodule update --init --recursive
```

## Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Output

Example screenshot of the console output from this app:

![CleanShot 2025-03-08 at 16 17 09](https://github.com/user-attachments/assets/a9f1e8e5-5282-4725-b344-96d23c7ae5ab)


![CleanShot 2025-03-08 at 16 17 41](https://github.com/user-attachments/assets/0fdc0fb7-8201-4862-8f68-cb1fab0ef426)


## Developing

If you're developing code for this repository, it's recommended to configure
your development environment:

### Code style

1. Ensure `clang-format` is installed
2. Ensure [pre-commit](https://pre-commit.com) is installed
3. Set up `pre-commit` for this repository:

  ``` console
  pre-commit install
  ```

This helps ensure that consistent code formatting is applied, by running
`clang-format` each time you change the code (via a git pre-commit hook) using
the [./.clang-format](./.clang-format) code style configuration file.

If you ever want to re-run the code formatting on all files in the repository,
you can do so:

``` console
pre-commit run --all-files
```
