# LH2 Decoder on RP2040
Valve's Lighthouse v2 positioning system decoder implemented for the RP2040.
completely CPU implementation

## Getting Started

1. Clone the repository and initialize the SDK submodule
```bash
git clone https://github.com/DotBots/lh2_on_rp2040.git
cd lh2_on_rp2040
git submodule update --init
cd pico-sdk
git submodule update --init
cd ..
```

2. Set the enviroment variable
To let the system know where is the SDK
```bash
export PICO_SDK_PATH=$PWD/pico-sdk
```

3. Create a build directory and run CMAKE
```bash
mkdir build && cd build
cmake ..
```

4. Build project
```bash
make
```

5. Flash the `lh2_receiver.uf2` file to the microcontroller/