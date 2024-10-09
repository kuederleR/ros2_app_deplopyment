#!/bin/bash
sudo apt-get install ipcalc

# Get the default interface
interface=$(ip route | grep default | awk '{print $5}')

# Get the IP address and subnet mask of the default interface
ip_address=$(ip -o -f inet addr show $interface | awk '{print $4}')
subnet=$(ipcalc -n $ip_address | grep Network | awk '{print $2}')
gateway=$(ip route | grep default | awk '{print $3}')


network_name="macvlan_network"

if docker network ls --filter name=^${network_name}$ --format "{{ .Name }}" | grep -w ${network_name} > /dev/null 2>&1; then
  echo "Replacing existing virtual network"
  docker network rm ${network_name}
else
  echo "Creating new virtual network"
fi

# Create the Macvlan network
docker network create -d macvlan \
  --subnet=$subnet \
  --gateway=$gateway \
  -o parent=$interface macvlan_network

echo "Macvlan network created with subnet: $subnet, gateway: $gateway, interface: $interface"
