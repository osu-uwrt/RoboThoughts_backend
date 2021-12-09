#Runs without sim as a test!
echo "Launching NGINX Sever!"

#Kill everything on port 1935
sudo lsof -ti tcp:1935 | sudo xargs -n1 kill -9 &> /dev/null

#Kill everything on port 8080
sudo lsof -ti :8080 | sudo xargs -n1 kill -9 &> /dev/null

#Kill everything on port 8081
sudo lsof -ti :8081 | sudo xargs -n1 kill -9 &> /dev/null

#Kill instances of web_video_server - throws error due to grep - is ok
ps -aux | grep web_video_server | awk '{print $2'} | xargs -n1 sudo kill -9 &> /dev/null

#Kill any prviously running of nginx - forces restart every time
ps -aux | pgrep nginx | xargs -n1 sudo kill -9 &> /dev/null

#Kill the backend if it already running
sudo lsof -ti :5000 | sudo xargs -n1 kill 9 

sleep 5

#Kill the backend if it already running
sudo lsof -ti :5000 | sudo xargs -n1 kill 9 

#Launch the website api
python3 ./api/app.py &>/dev/null &

#launch ros web video server
#rosrun web_video_server web_video_server &

#Launch the NGINX proxy server
sudo /usr/local/nginx/sbin/nginx

#Give the web video server time to successfully boot before beginning ffmpeg - because the process runs in background
sleep 5

#Launch ffmpeg - this step could possibly be avoided with the addition of HLS to HTTP module in nginx
ffmpeg -i /dev/video0 -vcodec libx264 -x264-params keyint=30:scenecut=0 -acodec libmp3lame -ar 44100 -ac 1 -f flv rtmp://0.0.0.0/puddles/stream -loglevel error &

#
echo "Launched all stream elements!"