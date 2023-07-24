# Quick start
### Connect to your webcam
`pip3 install feagi_agent_video_capture`

`python3 -m feagi_agent_video_capture`

# What is feagi_agent_video_capture?
It allows you to feed videos related to FEAGI using a webcam, gif, mp4, picamera, or screen recording. It can also utilize multiple flags. The purpose of this is to enable FEAGI to interact with and work with its peripherals.

# Requirements.txt
These are the requirements which will be updated over time. The script will use this requirement to scan.

Here: [requirements.txt](https://github.com/feagi/feagi/blob/staging/peripherals/feagi_agent_video_capture/feagi_agent_video_capture/requirements.txt)

### Flags:
```
--loop: True or False (Default is False)
--ip: (xxx.xxx.xx.xxx) Default is 127.0.0.1
--device: Target the port or index to use. Default is 0.
--video: You need to provide the file path.(mp4, gif, or wav) Default: None
--port: Change the API port of FEAGI. Default is 8000.
```
Let's say you have a file called earth.gif. If you want to run gif or mp4 only, but it is on your desktop, you will need to specify the file path. So, if you are in the Desktop folder while the file is also in the desktop folder, you can simply do this:

`python3 -m feagi_agent_video_capture --video earth.gif` (mac and linux)

`python -m feagi_agent_video_capture --video earth.gif` (Windows)

But if you are on the Desktop while your earth.gif is in the Downloads folder, do this:

`python3 -m feagi_agent_video_capture --video path/to/directory/earth.gif`

If you want it to loop, just append `--loop` in the command.

`python3 -m feagi_agent_video_capture --video earth.gif --loop true`

If you have two webcams and you want to use the second webcam, you can do this:

`python3 -m feagi_agent_video_capture --device 1` (Depending on what device index your webcam has, 1 is an assumption for the index of the second webcam. 0 is the default.)

If you want to send your webcam feed to FEAGI on another computer, you will need the computer's local IP. Do it like this:

`python3 -m feagi_agent_video_capture --ip xxx.xxx.xx.xxx` (Replace the x with the other machine's IP address.)