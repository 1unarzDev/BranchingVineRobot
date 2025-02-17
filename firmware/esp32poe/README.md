Just a few quick notes when working with the ESP32 POE board:
- You should disable your firewall with `sudo ufw disable` or permanantly by editing `/etc/default/ufw`, changing `DEFAULT_FORWARD_POLICY="DROP"`, and restarting `sudo service ufw restart`.
- As a debugging tool, I highly recommend you use Wireshark. Get it by running `sudo apt update` and `sudo apt install wireshark`. To use Wireshark, run `sudo wireshark` and click on receive packets to start debugging.
- To avoid complications with DHCP for receiving an ip address, manually create a static one in the code or using your prefered method. I just found my laptop's ipv4 address with `ip a` (alternatively `ifconfig`) and made both the server and client scripts talk to it for allowing the server to receive packets.
- In order to get the wired network configured properly on my laptop, I had to manually set the ip address used for the wired network. YOU SHOULD SEE A PROPERLY CONNECTED WIRED NETWORK ON YOUR COMPUTER.
- Ideally I would use UDP for communication for less overhead, but I managed to get TCP working first.
- Refer to the README on the parent firmware page if you are having issues uploading code to the ESP32 POE on PIO or working over the serial port in general.
- Make sure that you use an open port to communicate with the board (you don't necessarily even need to turn off Wifi as long as you find an open port and a way to share the subnet).
