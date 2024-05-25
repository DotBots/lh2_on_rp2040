# LH2 Decoder on RP2040
Valve's Lighthouse v2 positioning system decoder implemented for the RP2040.
completely CPU implementation

## Getting Started

1. Clone the repository and initialize the SDK submodule
```bash
git clone https://github.com/DotBots/lh2_on_rp2040.git
cd lh2_on_rp2040
git submodule update --init
```

2. Set the enviroment variable
To let the system know where is the SDK
```bash
export PICO_SDK_PATH=./pico-sdk
```