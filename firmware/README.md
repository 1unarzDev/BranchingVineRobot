## PlatformIO Linux Error

If you ever have issues communicating with USBs in PlatformIO on Ubuntu devices, run the following commands to see what may be the issue:

```bash
ls /dev/ttyUSB*
dmesg | tail -n 20
```

Chances are, you are having issues with BRLTTY, a service for working with braille devices removing your device. In order to fix this, run the following:

```bash
# Kill brltty
sudo systemctl stop brltty
sudo killall brltty

# Check if it's still running
ps aux | grep brltty

# If you want to permanantly disable or delete it try either option below
sudo apt remove --purge brltty
sudo systemctl disable brltty
sudo systemctl stop brltty
```

## Useful Debugging Commands

Find the serial numbers and information of your USB devices.
```bash 
rs-enumerate-devices
```

List all available serial devices.
```bash
lsusb
```

Useful packet tracing software.
```bash
sudo apt install wireshark
sudo wireshark
```

Disable firewall
```bash
sudo ufw status # Check firewall status
sudo systemctl stop ufw # Disable firewall
sudo ufw disable # Disable firewall at startup
```