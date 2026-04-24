GSM Base station for VOICE.  
  
You will need "tmux" for running these scripts.  
Set correct path to your "osmo-trx-soapy" in start_base.sh  
Run  
./start_master.sh  
./start_edge.sh
./start_base.sh  
  
To route your traffic to the Internet (replace "wlo1" to your network interface, "tun4" is set in osmo-ggsn.cfg):  
sudo sysctl -w net.ipv4.ip_forward=1  
sudo iptables -t nat -F    --- Clear iptables  
sudo iptables -t nat -A POSTROUTING -s 172.16.222.0/24 -o wlo1  -j MASQUERADE  
sudo iptables -A FORWARD -i tun4 -o wlo1  -j ACCEPT  
sudo iptables -A FORWARD -i wlo1  -o tun4 -m state --state ESTABLISHED,RELATED -j ACCEPT  
  
Also maybe this can be helpful:  
sudo iptables -t mangle -I FORWARD -p tcp --tcp-flags SYN,RST SYN -j TCPMSS --set-mss 400  
