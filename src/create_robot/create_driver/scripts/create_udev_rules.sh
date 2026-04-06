#!/bin/bash
# Create udev rules for consistent USB port naming

echo "Creating udev rules for motor controller and LiDAR..."
echo ""

# Get the current user
USER_NAME=$(whoami)

# Create udev rules file
UDEV_RULES_FILE="/tmp/99-motor-controller.rules"

cat > "$UDEV_RULES_FILE" << 'EOF'
# udev rules for Yahboom Motor Controller and LiDAR
# Motor Controller: CH340 (Vendor 1a86)
# LiDAR: CP2102 (Vendor 10c4)

# Motor Controller - CH340 chip (Yahboom)
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7522", SYMLINK+="motor_controller", MODE="0666", GROUP="dialout"

# LiDAR - CP2102 chip (Silicon Labs)
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="lidar", MODE="0666", GROUP="dialout"
EOF

echo "Udev rules file created:"
cat "$UDEV_RULES_FILE"
echo ""

read -p "Install these rules? (requires sudo) (y/n) " -n 1 -r
echo ""
if [[ $REPLY =~ ^[Yy]$ ]]; then
    sudo cp "$UDEV_RULES_FILE" /etc/udev/rules.d/99-motor-controller.rules
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    
    echo ""
    echo "✓ Udev rules installed!"
    echo ""
    echo "After unplugging and replugging devices, you should see:"
    echo "  /dev/motor_controller -> motor controller"
    echo "  /dev/lidar -> LiDAR"
    echo ""
    echo "You can now use in launch files:"
    echo "  dev:=/dev/motor_controller"
else
    echo "Rules not installed. File saved to: $UDEV_RULES_FILE"
    echo "To install manually:"
    echo "  sudo cp $UDEV_RULES_FILE /etc/udev/rules.d/99-motor-controller.rules"
    echo "  sudo udevadm control --reload-rules"
    echo "  sudo udevadm trigger"
fi






