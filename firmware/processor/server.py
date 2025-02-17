import socket
import struct
import subprocess
import psutil
import re
import time


TCP_IP = "192.168.1.236"  # The static IP to bind to
TCP_PORT = 2718
NETMASK = "255.255.255.0"


def get_dns_and_gateway():
    # Get DNS servers from /etc/resolv.conf
    with open('/etc/resolv.conf', 'r') as f:
        resolv_conf = f.read()
    
    dns_servers = re.findall(r'nameserver\s+(\d+\.\d+\.\d+\.\d+)', resolv_conf)
    
    # Get gateway (default route)
    result = subprocess.run(["ip", "route", "show"], capture_output=True, text=True)
    gateway = re.search(r'default via (\d+\.\d+\.\d+\.\d+)', result.stdout)
    
    if gateway:
        gateway_ip = gateway.group(1)
    else:
        gateway_ip = None
    
    return dns_servers, gateway_ip


def check_valid_ip_config(interface):
    # Check if the interface is using DHCP
    result = subprocess.run(["cat", f"/etc/systemd/network/{interface}.network"], capture_output=True, text=True)
    config = result.stdout.lower()
    
    if "dhcp" in config:
        return False
    elif "address" in config and "gateway" in config:
        print("Address and gateway configured")
        return True
    else:
        print("No DHCP or static IP configuration found for the interface") 
        return False

def release_dhcp(interface):
    try:
        if subprocess.run(["which", "dhclient"], stdout=subprocess.PIPE, stderr=subprocess.PIPE).returncode == 0:
            subprocess.run(["sudo", "dhclient", "-r"], check=True)
        elif subprocess.run(["which", "networkctl"], stdout=subprocess.PIPE, stderr=subprocess.PIPE).returncode == 0:
            subprocess.run(["sudo", "networkctl", "disable", interface], check=True)
        else:
            raise Exception("Unable to find dhclient or networkctl")
    except Exception as e:
        print(f"Error releasing DHCP: {e}")


def set_static_ip(interface, current_ip, dns_servers, gateway, ip):
    try:
        valid_ip_config = check_valid_ip_config(interface)
        if not valid_ip_config:
            print(f"Interface {interface} is using DHCP. Releasing DHCP...")
            release_dhcp(interface)
            time.sleep(2)  # Wait a bit for the DHCP to release
        
        # Check if the current IP matches the static IP to avoid redundant configuration
        interfaces = psutil.net_if_addrs()
        current_ip = next(
            (addr.address for iface_name, iface_addresses in interfaces.items() if iface_name == interface 
             for addr in iface_addresses if addr.family == socket.AF_INET), None
        )
        
        if current_ip == ip and valid_ip_config:
            print(f"The interface {interface} is already configured with IP {ip}")
            return
        
        if subprocess.run(["which", "systemctl"], stdout=subprocess.PIPE, stderr=subprocess.PIPE).returncode == 0:
            # systemd-based systems (Ubuntu, Debian)
            with open(f'/etc/systemd/network/{interface}.network', 'w') as f:
                f.write(f"""
                [Match]
                Name={interface}

                [Network]
                Address={ip}/24
                Gateway={gateway}
                DNS={dns_servers}
                """)
            subprocess.run(["sudo", "systemctl", "restart", "systemd-networkd"], check=True)
        else:
            # sysvinit-based systems (older Ubuntu versions)
            with open('/etc/network/interfaces', 'r') as f:
                content = f.readlines()
            with open('/etc/network/interfaces', 'w') as f:
                f.write("".join(content) + f"""
                auto {interface}
                iface {interface} inet static
                address {ip}
                netmask 255.255.255.0
                gateway {gateway}
                dns-nameservers {dns_servers}
                """)

            subprocess.run(["sudo", "systemctl", "restart", "networking"], check=True)
        
        print(f"Static IP {ip} set successfully on {interface}")
    except Exception as e:
        print(f"Error setting static IP: {e}")


def get_wired_interface():
    interfaces = psutil.net_if_addrs()
    for iface_name, iface_addresses in interfaces.items():
        if iface_name == "lo" or iface_name.startswith("docker"):
            continue
        
        has_mac = any(addr.family == socket.AF_PACKET for addr in iface_addresses)
        ipv4_addr = next(
            (addr for addr in iface_addresses if addr.family == socket.AF_INET), None
        )
        
        if has_mac and ipv4_addr and ipv4_addr.address != "127.0.0.1":
            return iface_name, ipv4_addr.address
    return None, None


def get_connection_profile_name(interface):
    try:
        # Run nmcli command to list active connections
        result = subprocess.run(
            ["nmcli", "-t", "-f", "name,device", "connection", "show", "--active"],
            capture_output=True,
            text=True
        )
        output = result.stdout.strip()
        
        # Parse the output to find the connection profile for the interface
        for line in output.splitlines():
            name, dev = line.split(":")
            if dev == interface:
                return name
        return None
    except Exception as e:
        print(f"Error retrieving connection profile name: {e}")
        return None
    

dns_servers, gateway_ip = get_dns_and_gateway()
interface, current_ip = get_wired_interface()
profile_name = get_connection_profile_name(interface)

print(f"Using profile named {profile_name}")


if not interface:
    print("No active wired interface found!")
else:
    set_static_ip(interface, current_ip, dns_servers, gateway_ip, TCP_IP)

    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((TCP_IP, TCP_PORT))
        sock.listen(1)
        print("Started socket")
    except socket.error as e:
        print(f"Socket error: {e}")
        exit(1)

    while True:
        try:
            conn, addr = sock.accept()
            data = conn.recv(1024)
            header = data[0]
            if data:
                x, y, z = struct.unpack('3d', data[1:25])
                
                if header == 0:
                    print(f"orientation: x={round(x, 4)}, y={round(y, 4)}, z={round(z, 4)}")
                elif header == 1:
                    print(f"linaccel: x={round(x, 4)}, y={round(y, 4)}, z={round(z, 4)}")
        except Exception as e:
            print(f"Error during connection: {e}")
            continue