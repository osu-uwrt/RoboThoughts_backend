# be sure to run in setup directory

#flask and dependencies
sudo apt install python-pip
pip install flask

#ros webvideo server
sudo apt install ros-noetic-web-video-server

#nginx dependenices
sudo apt-get install build-essential libpcre3 libpcre3-dev libssl-dev

#other
sudo apt install lsof
sudo apt install net-tools
sudo apt install ffmpeg

#make nginx setup directory
mkdir nginxHLS

#get and unpack nginx - version 1.18.0
(cd nginxHLS && wget https://nginx.org/download/nginx-1.18.0.tar.gz && tar -xf nginx-1.18.0.tar.gz)
(cd nginxHLS && git clone https://github.com/sergey-dryabzhinsky/nginx-rtmp-module.git)

