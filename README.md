# Okinawa AI Beach Robot Python Interface

## Initial setup
[It is recommended](https://realpython.com/python-virtual-environments-a-primer/#why-do-you-need-virtual-environments) to use virtual environments. Start by creating a virtual environment then installing the beachbot package without dependencies in edit mode (allows you to make use of any recent changes without reinstalling). This will avoid installing any conflicting packages with jetpack on the jetson or otherwise.
```
python3 -m venv .venv
. .venv/bin/activate
pip install -e .
```

Alternatively, if desired to reuse pip packages installed system-wide, the following can be used:
```
python3 -m venv --system-site-packages .venv
. .venv/bin/activate
pip install -e .
```

So for standard production installations this would just mean `pip install beachbot`


### Configuration of Data Folders
Current configuration can be inspecte by:
```
import beachbot
print(beachbot.config)
```

Example for setup of path variables in `.bashrc`, e.g. ubuntu setup for local development folder in /home/[username]/src/OIST_BC/`:
```
export BEACHBOT_HOME=${HOME}/src/OIST_BC/
export BEACHBOT_CACHE=${BEACHBOT_HOME}/Cache
export BEACHBOT_CONFIG=${BEACHBOT_HOME}/Config
export BEACHBOT_LOGS=${BEACHBOT_HOME}/Logs
export BEACHBOT_MODELS=${BEACHBOT_HOME}/Models
export BEACHBOT_MODELS=${BEACHBOT_HOME}/Datasets

```




### System Dependencies
The following commands need to be exeuted to install required software dependencies.
Video IO and codecs:
```
sudo apt install v4l-utils
sudo apt-get install ffmpeg x264 libx264-dev
```

## Troubleshooting
Assuming you follow the guide above for installing on the Jetson, this should not be necessary, but in the event you see any errors, the information below may be of use:

On jetson, python-opencv is installed by nvidia and could be hidden by the pip version.
The pip-version does not support the jetson camera, thus, we have to uninstall all opencv pip versions from the jetson after reinstall of our package:
```
pip uninstall opencv-python
```

Errors such as:
```
(python:7658): GStreamer-WARNING **: 16:29:41.143: Failed to load plugin '/usr/lib/aarch64-linux-gnu/gstreamer-1.0/libgstnvarguscamerasrc.so': /lib/aarch64-linux-gnu/libGLdispatch.so.0: cannot allocate memory in static TLS block

(python:7658): GStreamer-WARNING **: 16:29:41.146: Failed to load plugin '/usr/lib/aarch64-linux-gnu/gstreamer-1.0/libgstnvvidconv.so': /lib/aarch64-linux-gnu/libGLdispatch.so.0: cannot allocate memory in static TLS block
```
can be avoided by prefixing the command such as, e.g.,
```
LD_PRELOAD=/lib/aarch64-linux-gnu/libGLdispatch.so python nicegui_controller.py
```


Error `ModuleNotFoundError: No module named 'distutils'`
(python3 -m pip install --upgrade pip)
`pip install distutils` (old python versions) or `pip install setuptools`

Error `module 'pkgutil' has no attribute 'ImpImporter'.`
Numpy is not compatible 24.4 with newer python versions, try to use 26.4 numpy.
