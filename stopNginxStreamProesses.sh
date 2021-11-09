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
sudo lsof -ti :5000 | sudo xargs -n1 kill 9 &> /dev/null
