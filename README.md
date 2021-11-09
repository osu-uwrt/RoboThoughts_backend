# Riptide Software

#### Authors
Nicholas Iten,
Noah Limes,
Clark Godwin,
Jerry Qiu,
Seamus Scanlon

## Riptide Robothoughts

This repository holds the backend for the Robothoughts Project. Consisting of a RESTful API and
uses web-video-server from ROS. This is designed to communicate with [robo\_thoughts](https://github.com/osu-uwrt/robo_thoughts.git)

Supported Operating Systems: Ubuntu 16.04


**The Underwater Robotics Team**  
The Ohio State University

[Website](https://uwrt.engineering.osu.edu) \| [License](https://github.com/osu-uwrt/riptide_software/tree/fac98cfa750df74dbb107f83064c3767e6346cc4/LICENSE/README.md)

## Initial Setup

In order to run the backend and nginx stream follow the steps below.

1. "cd src/riptide_robothoughts/setup"

2. Run "sudo ./installRobothoughtsDependencies" to install all of the dependencies and to get nginx.

3. "cd nginxHLS/nginx-1.18.0"

4. Compile with the nessecary rtmp module nginx by running: 
"./configure --with-http_ssl_module --add-module=../nginx-rtmp-module"
"make"
"sudo make install"

5. Change back into the setup directory "cd ../.."

6. Copy the nginx config file into installation "sudo cp nginx.conf /usr/local/nginx/conf/nginx.conf"

7. Recommended: Test the nginx configuration "/usr/local/nginx/sbin/nginx -t"

## Testing the Backend

To test the backend configuration and setup run the testBackendWithNginxStream.sh file. This test does not require a running ROS instance and therefore can be ran without connection to riptide / simulation. This will attempt to stream video from "/dev/video0". If there is no video at"/dev/video0" the stream will fail. Additional errors will be thrown from the frontend console as it will not be able to fetch the riptide position, battery, etc... data. These can be ignored - probably.

To run the backend without streaming run the app.py python script.

To kill the test stream and backend run the stopNginxStreamProesses file.

## Starting the Backend

To run the backend, run the  runBackendWithNginxStream.sh file. This must be ran only when a master ROS node is reachable - either physically or in simulation. 

To kill the test stream and backend run the stopNginxStreamProesses file.

Note: Running the simulation and backend stream requires much computational power. The stream will likely be lagging if working at all.

## Functionality

Once the backend is running, POST requests can be made to the server. The POST request must submit
data as a JSON object with the following format:

```json
{
  "request": [
    {
      "data": "<IDENTIFIER>"
    }
  ]
}
```

Replace `IDENTIFIER` with any of the approved requests below. Also request is an array, which can have
more than one request given.

```text
Controls_Depth
State_Depth
Bboxes
Dvl
Imu
Object
Switches
```

A valid request will yield the following JSON. Multiple identifiers will be placed in the `"data"` array
if requested.

```json
{
  "data": [
    {
      "<IDENTIFIER>": {
          "key" : "value",
          "key2" : "value"
      }
    }
  ]
}
```

Once `run_backend.dms` has been executed, the backend will be fully running. Identify what the
host machine's IP address is. The backend can be reached by sending the POST request to 
`http://<IP_ADDRESS>:5000/` where `IP_ADDRESS` is the host machine's IP. The video feed can
be accessed at `http://<IP_ADDRESS>:8080/`. We recommend going directly to the image feed
at `http://<IP_ADDRESS>/stream?topic=/darknet_ros/image_hud&default_transport=compressed`