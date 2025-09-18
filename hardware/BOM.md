# Bill of Materials (BOM)

## Main Components

### Microcontroller
| Part | Description | Quantity | Source | Notes |
|------|------------|----------|--------|-------|
| RP2040 Zero | Main MCU | 1 | Waveshare/Amazon | Alternative: ATtiny1614 |
| OR | | | | |
| ATtiny1614 | Alternative MCU | 1 | DigiKey/Mouser | Lower cost option |

### Power Supply
| Part | Description | Quantity | Source | Notes |
|------|------------|----------|--------|-------|
| MP1584EN | Buck converter module | 1 | Amazon/AliExpress | 24V to 3.3V/5V |
| 100µF/35V | Input capacitor | 1 | DigiKey/Mouser | Electrolytic |
| 10µF/16V | Output capacitor | 2 | DigiKey/Mouser | Ceramic |
| 0.1µF | Bypass capacitors | 5 | DigiKey/Mouser | Ceramic |

### Connectors
| Part | Description | Quantity | Source | Notes |
|------|------------|----------|--------|-------|
| Molex 502584-0470 | Female CLIK-Mate (to board) | 1 | DigiKey/Mouser | 4-pos, 2mm pitch |
| Molex 502585-0470 | Male CLIK-Mate (to fan) | 1 | DigiKey/Mouser | 4-pos, 2mm pitch |
| 2x5 pin header | Hackerboard connector | 1 | Amazon | 2.54mm pitch |
| 4-pin header | Extension board connector | 1 | Amazon | 2.54mm pitch |

### Sensors
| Part | Description | Quantity | Source | Notes |
|------|------------|----------|--------|-------|
| BME280 | Temp/Humidity/Pressure | 1 | Adafruit/Amazon | I2C module |
| SGP30 | TVOC/eCO2 sensor | 1 | Adafruit #3709 | Recommended |
| 4.7kΩ | I2C pullup resistors | 2 | DigiKey/Mouser | 1/4W |

### Level Shifting
| Part | Description | Quantity | Source | Notes |
|------|------------|----------|--------|-------|
| BSS138 | N-channel MOSFET | 2 | DigiKey/Mouser | SOT-23 |
| 10kΩ | Pullup resistors | 4 | DigiKey/Mouser | 0805 SMD |

### Heater Control (Optional)
| Part | Description | Quantity | Source | Notes |
|------|------------|----------|--------|-------|
| SSR-25DA | Solid state relay | 1 | Amazon | For AC heater |
| OR | | | | |
| IRLB8721 | Power MOSFET | 1 | DigiKey/Mouser | For 24V PTC |
| PTC Heater | 24V 300W | 1 | Amazon | Self-regulating |

### Miscellaneous
| Part | Description | Quantity | Source | Notes |
|------|------------|----------|--------|-------|
| PCB | Custom or protoboard | 1 | JLCPCB/PCBWay | 70x75mm suggested |
| LED | Status indicators | 3 | DigiKey/Mouser | Red, Green, Blue |
| 330Ω | LED resistors | 3 | DigiKey/Mouser | 1/4W |
| Second Fan | 120mm PWM | 1 | Amazon | Internal circulation |

## Estimated Costs
- Basic version (no heater): ~$40-50
- Full version with heater: ~$70-90
- With VOC sensor: +$20

## Suppliers
- **Molex Connectors**: DigiKey, Mouser, Newark
- **Electronic Components**: DigiKey, Mouser, LCSC
- **Modules & Arduino**: Amazon, Adafruit, SparkFun
- **PCB Fabrication**: JLCPCB, PCBWay, OSH Park
