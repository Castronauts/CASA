sudo tc qdisc add dev wlan0 root handle 1: htb default 12 
sudo tc class add dev wlan0 parent 1:1 classid 1:12 htb rate 1000Mbps ceil 10000Mbps
sudo tc qdisc add dev wlan0 parent 1:12 netem delay 400ms loss 0%