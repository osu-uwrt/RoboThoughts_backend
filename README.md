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

In order to run the backend ensure the system is up to date by running `setup.dms` in the Ubuntu terminal.

## Starting the Backend

The backend environment is controlled entirely by running `run_backend.dms` in the Ubuntu terminal. In
order to communicate with the robot, be sure to run the following command:
`export ROS_MASTER_URI=https://baycat:11311`
 
If running inside of Docker, ensure that ports 5000 and 8080 are exposed. This can be done by starting
a container with the `-p 5000:5000 -p 8080:8080` flags. These ports are the only method of accessing
the backend.

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