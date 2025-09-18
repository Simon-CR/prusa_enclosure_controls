# Prusa Enclosure Control System

An intelligent temperature and air quality management system for Prusa MK4 printer enclosures, featuring automatic material-based climate control, VOC filtration, and heating capabilities.

## Features

- **Material-Specific Profiles**: Automatic temperature and ventilation control based on filament type
- **Dual Climate Control**: Integrated heating and cooling with warp prevention
- **VOC Management**: Smart filtration for toxic materials (ABS/ASA/PC)
- **G-code Integration**: Full control via PrusaSlicer custom G-code
- **Safety Features**: Over-temperature protection, heater timeouts, controlled cooldown
- **No Firmware Mods**: Uses standard Prusa GPIO Hackerboard interface
- **Web Dashboard**: Real-time monitoring at http://prusa-enclosure.local
- **MQTT Support**: Full Home Assistant integration
- **REST API**: JSON API for third-party integration
- **OctoPrint Ready**: Foundation provided for plugin development

## System Overview

The controller interfaces between:
- Prusa GPIO Hackerboard (M262 G-code commands)
- 24V Extension Board power
- PWM-controlled enclosure fan
- Temperature/humidity sensor (BME280)
- VOC sensor (SGP30/SGP40/ENS160/BME688)
- Optional enclosure heater (PTC or ceramic)

## Quick Start

1. Build the controller board (see [hardware/](hardware/) folder)
2. Flash the firmware (see [firmware/](firmware/) folder)
3. Install PrusaSlicer profiles (see [prusaslicer/](prusaslicer/) folder)
4. Connect to your Prusa MK4 and enclosure

## Material Profiles

| Material | Target Temp | Min Temp | Ventilation | Preheat | VOC Control |
|----------|------------|----------|-------------|---------|-------------|
| PLA      | 25°C       | 20°C     | Normal      | No      | No          |
| PETG     | 35°C       | 25°C     | Moderate    | No      | No          |
| ABS      | 45°C       | 40°C     | Minimal     | Yes     | Yes         |
| ASA      | 50°C       | 45°C     | Minimal     | Yes     | Yes         |
| PC       | 55°C       | 50°C     | Moderate    | Yes     | Yes         |
| Nylon    | 45°C       | 35°C     | Moderate    | Yes     | No          |

## Documentation

- [Hardware Assembly](docs/assembly.md)
- [Breadboard Testing](docs/breadboard_testing.md)
- [PrusaSlicer Setup](docs/prusaslicer_setup.md)
- [Web Dashboard Guide](docs/web_dashboard.md)
- [Troubleshooting](docs/troubleshooting.md)

## Web Dashboard & Integration

### Features
- **Web Dashboard**: Real-time monitoring at http://prusa-enclosure.local
- **MQTT Support**: Full Home Assistant integration
- **REST API**: JSON API for third-party integration
- **OctoPrint Ready**: Foundation provided for plugin development

### Home Assistant Integration
The system automatically publishes to Home Assistant via MQTT discovery. Entities include:
- Temperature, humidity, VOC sensors
- Fan speed controls
- System enable/disable switches
- Automated alerts for high VOC or temperature

### API Endpoints
- `GET /api/status` - Current system status
- `POST /api/command` - Send commands
- `GET /api/history` - Historical data
- `GET /api/printer/chamber` - OctoPrint-compatible endpoint

### MQTT Topics
- `prusa/enclosure/state` - JSON status updates
- `prusa/enclosure/command` - Command input
- `prusa/enclosure/availability` - Online/offline status

## Safety Warning

⚠️ **This project involves:**
- Mains voltage (if using AC heater)
- High-power heating elements
- Modification of 3D printer peripherals

Always follow proper electrical safety procedures and never leave the system unattended during initial testing.

## License

MIT License - See [LICENSE](LICENSE) file

## Author

Simon-CR - 2025
