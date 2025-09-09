# ROS2 BME280 Sensor Reader

A ROS2 package for reading temperature, humidity, and pressure data from a BME280 sensor via I2C communication.

## Features

- **I2C Communication**: Uses I2C interface for reliable sensor communication
- **Separate Topics**: Publishes data to individual topics for better organization
- **Organized Namespace**: All sensor data published under `/sensors/` namespace
- **Real-time Monitoring**: Continuous sensor data publishing at 2-second intervals

## Topics

The node publishes to the following topics:

- `/sensors/temperature` (std_msgs/Float32) - Temperature in Celsius
- `/sensors/humidity` (std_msgs/Float32) - Humidity percentage
- `/sensors/pressure` (std_msgs/Float32) - Pressure in hPa

## Hardware Requirements

- Raspberry Pi with I2C enabled
- BME280 sensor connected via I2C
- I2C address: 0x76 (default) or 0x77

## Dependencies

- ROS2 (Humble/Iron)
- Python 3.8+
- pimoroni-bme280
- smbus2

## Installation

### 1. Install Dependencies

```bash
# Install Python dependencies
pip install pimoroni-bme280 smbus2

# Or using uv (recommended)
uv add pimoroni-bme280 smbus2
```

### 2. Enable I2C on Raspberry Pi

```bash
sudo raspi-config
# Navigate to Interface Options > I2C > Enable
```

### 3. Verify I2C Connection

```bash
# Check if BME280 is detected
sudo i2cdetect -y 1
# Should show 76 or 77 in the output
```

### 4. Build the Package

```bash
# From your ROS2 workspace
colcon build --packages-select ros2_bme280
source install/setup.bash
```

## Usage

### Running the Node

```bash
ros2 run bme280_reader bme280_serial_node
```

### Monitoring Topics

```bash
# List all topics
ros2 topic list

# Monitor specific sensor data
ros2 topic echo /sensors/temperature
ros2 topic echo /sensors/humidity
ros2 topic echo /sensors/pressure

# Check topic info
ros2 topic info /sensors/temperature
```

### Viewing Node Information

```bash
# List running nodes
ros2 node list

# Get node info
ros2 node info /bme280_i2c_reader
```

## Configuration

The node can be configured by modifying the following parameters in the source code:

- **I2C Port**: Default is port 1 (Raspberry Pi standard)
- **I2C Address**: Default is 0x76 (change to 0x77 if needed)
- **Publishing Rate**: Default is 2 seconds (modify timer interval)

## Troubleshooting

### Common Issues

1. **ModuleNotFoundError: No module named 'bme280'**
   - Install the pimoroni-bme280 library: `pip install pimoroni-bme280`

2. **I2C error: [Errno 2] No such file or directory**
   - Enable I2C interface: `sudo raspi-config nonint do_i2c 0`
   - Reboot the system

3. **I2C error: BME280 not found**
   - Check wiring connections
   - Verify I2C address with `sudo i2cdetect -y 1`
   - Try changing address from 0x76 to 0x77 in the code

4. **Permission denied errors**
   - Add user to i2c group: `sudo usermod -a -G i2c $USER`
   - Log out and log back in

### Debug Commands

```bash
# Check I2C devices
sudo i2cdetect -y 1

# Check I2C permissions
ls -l /dev/i2c-*

# View node logs
ros2 log level /bme280_i2c_reader debug
```

## Package Structure

```
ros2_bme280/
├── bme280_reader/
│   ├── __init__.py
│   └── bme280_serial_node.py
├── package.xml
├── setup.cfg
├── setup.py
├── resource/
│   └── bme280_reader
├── test/
│   ├── test_copyright.py
│   ├── test_flake8.py
│   ├── test_pep257.py
│   └── test_xmllint.py
├── .gitignore
└── README.md
```

## License

This project is licensed under the MIT License.

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests if applicable
5. Submit a pull request

## Support

For issues and questions, please create an issue in the repository or contact the maintainers.
