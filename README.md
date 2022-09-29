# SC2430-UHDExtension
USRP Hardware Driver (UHD) Extension Library for use with the SignalCraft [SC2430](https://www.signalcraft.com/products/test-measurement/microwave-systems/sc2430/) NR TDD Signal Conditioning Module (SCM). 

## SCM Hardware
The SCM is a front-end solution that provides signal conditioning and amplification for use with the [NI Ettus-USRP X410](https://www.ettus.com/all-products/usrp-x410/). 

The SCM is compliant with select 3GPP 5G/NR standards for UE and gNB: 
- Frequency Range: 350 MHz to 7.125 GHz
- Instantaneous Bandwidth: 
  - 100 MHz (350 MHz to 600 MHz)
  - 200 MHz (600 MHz to 800 MHz)
  - 400 MHz (>800 MHz)
- 3GPP Standards: NR
  - UE: 3GPP TS 38.101-1
  - gNB: 3GPP TS 38.104
- Bands Supported: n39, n34, n38, n40, n41, n46, n47, n48, n77, n78, n79, n96

## Documentation
See the Wiki for various topics related to the SC2430 Extension:
1. [Building and Installation](https://github.com/SignalCraftTechnologies/SC2430-UHDExtension/wiki/Building-and-Installation)
2. [Programming Details and Examples](https://github.com/SignalCraftTechnologies/SC2430-UHDExtension/wiki/Programming-Details)

For documentation and specifications related to the SCM hardware, see the [SCT Product Page](https://www.signalcraft.com/products/test-measurement/microwave-systems/sc2430/).

For details related to the Ettus USRP Hardware Driver (UHD),  see the [Ettus Research UHD repository](https://github.com/EttusResearch/uhd).

## Applications

Applications using [UHD 4.3.0.0](https://github.com/EttusResearch/uhd/releases/tag/v4.3.0.0) or newer support UHD Extensions such as the SC2430.
