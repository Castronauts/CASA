service ifplugd stop
ifconfig wlan0 down 
iwconfig wlan0 mode managed essid "UCB Wireless"
ifconfig wlan0 up
dhclient wlan0 
