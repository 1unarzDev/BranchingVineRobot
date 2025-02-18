import psutil
import socket

def list_interfaces_using_ip(ip):
    interfaces_using_ip = []
    
    interfaces = psutil.net_if_addrs()
    for iface_name, iface_addresses in interfaces.items():
        for addr in iface_addresses:
            if addr.family == socket.AF_INET and addr.address == ip:
                interfaces_using_ip.append(iface_name)
    
    return interfaces_using_ip

# Example usage:
target_ip = "192.168.1.236"
interfaces_using_ip = list_interfaces_using_ip(target_ip)

if interfaces_using_ip:
    print(f"The IP address {target_ip} is currently being used by the following interfaces: {interfaces_using_ip}")
else:
    print(f"No interfaces found using the IP address {target_ip}")
