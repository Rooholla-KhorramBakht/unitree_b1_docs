# Internet Forwarding
## Setup
To share the internet connection from a computer A with IP address IP1 to the other computer we can set up Network Address Translation (NAT) using iptables. Here's a step-by-step procedure:

1. Enable IP forwarding on the computer A. Open a terminal and run the following command:
```
sudo sysctl -w net.ipv4.ip_forward=1
```

2. Configure iptables to perform NAT. Run the following commands in the terminal:
```
sudo iptables -t nat -A POSTROUTING -o wlan0 -j MASQUERADE
sudo iptables -A FORWARD -i wlan0 -o eth0 -m state --state RELATED,ESTABLISHED -j ACCEPT
sudo iptables -A FORWARD -i eth0 -o wlan0 -j ACCEPT
```
Note: Replace `wlan0` with the actual name of your wireless network interface, and `eth0` with the name of the Ethernet interface on the computer with IP address 192.168.1.10.

3. Make the iptables rules persistent. Install the `iptables-persistent` package by running:
```
sudo apt-get install iptables-persistent
```
During the installation, you'll be asked if you want to save the current IPv4 rules. Choose "Yes" to save the rules.

4. On the other computer, set the default gateway to 192.168.1.10. Open a terminal and run the following command:
```
sudo ip route add default via 192.168.123.1
```

5. Also configure DNS settings. Open the `/etc/resolv.conf` file in a text editor and add the following line:
```
nameserver 8.8.8.8
```
This sets Google DNS as the primary DNS server. Save the file and exit.

With these steps, the computer with IP address 192.168.1.10 should be sharing its internet connection with the other computer on the local network. The IP addresses of your local network will remain fixed, and the computer with IP address 192.168.1.10 will act as the gateway for the other computer to access the internet.
## Modification and Saving
If you needed to change any of the configurations, you can save it again as follows. To save the current `iptables` configurations as rules without manually editing the file, you can use the `iptables-save` command. Here's how you can do it:

1. Open a terminal on your Ubuntu computer.

2. Run the following command to display the current `iptables` configurations:
   ```
   sudo iptables-save
   ```

   This command will output the current `iptables` rules in the required format.

3. Redirect the output to the `iptables` configuration file using the following command:
   ```
   sudo iptables-save > /etc/iptables/rules.v4
   ```

   This command will save the output of `iptables-save` to the `/etc/iptables/rules.v4` file, overwriting the existing rules.

4. Restart the `iptables-persistent` service:
   ```
   sudo systemctl restart iptables-persistent
   ```

   This will ensure that the newly saved `iptables` rules are loaded and applied.

Now, the current `iptables` configurations are saved as rules in the `/etc/iptables/rules.v4` file, and they will be automatically loaded during system startup. The changes will persist across reboots.
