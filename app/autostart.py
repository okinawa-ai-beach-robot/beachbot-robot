print("autostart.py... let`s start all the crazy stuff")

import sys

# request reload saved configuration*
sys.argv.append("--cfgload")

# start beachbot
import beachbot_controller as beachbot_controller
