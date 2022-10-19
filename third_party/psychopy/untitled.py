#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
This experiment was created using PsychoPy3 Experiment Builder (v2022.2.4),
    on Wed Oct 19 15:12:13 2022
If you publish work using this script the most relevant publication is:

    Peirce J, Gray JR, Simpson S, MacAskill M, Höchenberger R, Sogo H, Kastman E, Lindeløv JK. (2019) 
        PsychoPy2: Experiments in behavior made easy Behav Res 51: 195. 
        https://doi.org/10.3758/s13428-018-01193-y

"""

# --- Import packages ---
from psychopy import locale_setup
from psychopy import prefs
from psychopy import sound, gui, visual, core, data, event, logging, clock, colors, layout
from psychopy.constants import (NOT_STARTED, STARTED, PLAYING, PAUSED,
                                STOPPED, FINISHED, PRESSED, RELEASED, FOREVER)

import numpy as np  # whole numpy lib is available, prepend 'np.'
from numpy import (sin, cos, tan, log, log10, pi, average,
                   sqrt, std, deg2rad, rad2deg, linspace, asarray)
from numpy.random import random, randint, normal, shuffle, choice as randchoice
import os  # handy system and path functions
import sys  # to get file system encoding

import psychopy.iohub as io
from psychopy.hardware import keyboard

# Run 'Before Experiment' code from code
import numpy as np
from feagi_agent import retina as retina
from feagi_agent import feagi_interface as FEAGI
from configuration import *
from datetime import datetime


# Ensure that relative paths start from the same directory as this script
_thisDir = os.path.dirname(os.path.abspath(__file__))
os.chdir(_thisDir)
# Store info about the experiment session
psychopyVersion = '2022.2.4'
expName = 'untitled'  # from the Builder filename that created this script
expInfo = {
    'participant': '',
    'session': '001',
}
# --- Show participant info dialog --
dlg = gui.DlgFromDict(dictionary=expInfo, sortKeys=False, title=expName)
if dlg.OK == False:
    core.quit()  # user pressed cancel
expInfo['date'] = data.getDateStr()  # add a simple timestamp
expInfo['expName'] = expName
expInfo['psychopyVersion'] = psychopyVersion

# Data file name stem = absolute path + name; later add .psyexp, .csv, .log, etc
filename = _thisDir + os.sep + u'data/%s_%s_%s' % (expInfo['participant'], expName, expInfo['date'])

# An ExperimentHandler isn't essential but helps with data saving
thisExp = data.ExperimentHandler(name=expName, version='',
    extraInfo=expInfo, runtimeInfo=None,
    originPath='/Users/neurastation_0/feagi/third_party/psychopy/untitled.py',
    savePickle=True, saveWideText=True,
    dataFileName=filename)
# save a log file for detail verbose info
logFile = logging.LogFile(filename+'.log', level=logging.EXP)
logging.console.setLevel(logging.WARNING)  # this outputs to the screen, not a file

endExpNow = False  # flag for 'escape' or other condition => quit the exp
frameTolerance = 0.001  # how close to onset before 'same' frame

# Start Code - component code to be run after the window creation

# --- Setup the Window ---
win = visual.Window(
    size=[1280, 720], fullscr=False, screen=0, 
    winType='pyglet', allowStencil=False,
    monitor='testMonitor', color=[0,0,0], colorSpace='rgb',
    blendMode='avg', useFBO=True, 
    units='height')
win.mouseVisible = True
# store frame rate of monitor if we can measure it
expInfo['frameRate'] = win.getActualFrameRate()
if expInfo['frameRate'] != None:
    frameDur = 1.0 / round(expInfo['frameRate'])
else:
    frameDur = 1.0 / 60.0  # could not measure, so guess
# --- Setup input devices ---
ioConfig = {}

# Setup iohub keyboard
ioConfig['Keyboard'] = dict(use_keymap='psychopy')

ioSession = '1'
if 'session' in expInfo:
    ioSession = str(expInfo['session'])
ioServer = io.launchHubServer(window=win, **ioConfig)
eyetracker = None

# create a default keyboard (e.g. to check for escape)
defaultKeyboard = keyboard.Keyboard(backend='iohub')

# --- Initialize components for Routine "trial" ---
dots = visual.DotStim(
    win=win, name='dots',units='deg', 
    nDots=100, dotSize=2.0,
    speed=0.1, dir=0.0, coherence=1.0,
    fieldPos=(0.0, 0.0), fieldSize=1.0, fieldAnchor='center', fieldShape='circle',
    signalDots='same', noiseDots='direction',dotLife=3.0,
    color=[1.0,1.0,1.0], colorSpace='rgb', opacity=None,
    depth=0.0)
polygon = visual.ShapeStim(
    win=win, name='polygon',
    size=(0.5, 0.5), vertices='triangle',
    ori=0.0, pos=(0, 0), anchor='bottom-left',
    lineWidth=1.0,     colorSpace='rgb',  lineColor='white', fillColor='white',
    opacity=None, depth=-1.0, interpolate=True)
polygon_2 = visual.ShapeStim(
    win=win, name='polygon_2',
    size=(0.5, 0.5), vertices='circle',
    ori=0.0, pos=(0, 0), anchor='bottom-right',
    lineWidth=1.0,     colorSpace='rgb',  lineColor='black', fillColor='black',
    opacity=None, depth=-2.0, interpolate=True)
# Run 'Begin Experiment' code from code
# Generate runtime dictionary
previous_data_frame = dict()
runtime_data = {"cortical_data": {}, "current_burst_id": None, "stimulation_period": None, "feagi_state": None,
                "feagi_network": None}

# FEAGI section start
print("Connecting to FEAGI resources...")

# address = 'tcp://' + network_settings['feagi_host'] + ':' + network_settings['feagi_outbound_port']

feagi_host, api_port = FEAGI.feagi_setting_for_registration()
api_address = FEAGI.feagi_gui_address(feagi_host, api_port)

stimulation_period_endpoint = FEAGI.feagi_api_burst_engine()
burst_counter_endpoint = FEAGI.feagi_api_burst_counter()

runtime_data["feagi_state"] = FEAGI.feagi_registration(feagi_host=feagi_host, api_port=api_port)

print("** **", runtime_data["feagi_state"])
network_settings['feagi_burst_speed'] = float(runtime_data["feagi_state"]['burst_duration'])

# todo: to obtain this info directly from FEAGI as part of registration
ipu_channel_address = FEAGI.feagi_inbound(runtime_data["feagi_state"]['feagi_inbound_port_gazebo'])
print("IPU_channel_address=", ipu_channel_address)
opu_channel_address = FEAGI.feagi_outbound(network_settings['feagi_host'],
                                           runtime_data["feagi_state"]['feagi_outbound_port'])

feagi_ipu_channel = FEAGI.pub_initializer(ipu_channel_address)
feagi_opu_channel = FEAGI.sub_initializer(opu_address=opu_channel_address)
# FEAGI section ends
previous_frame_data = dict()
msg_counter = runtime_data["feagi_state"]['burst_counter']

redTheta = 0
blueTheta = 0
X = 0
Y = 0
flag = False
rgb = dict()
rgb['camera'] = dict()
mouse = event.Mouse(win=win)
x, y = [None, None]
mouse.mouseClock = core.Clock()

# Create some handy timers
globalClock = core.Clock()  # to track the time since experiment started
routineTimer = core.Clock()  # to track time remaining of each (possibly non-slip) routine 

# --- Prepare to start Routine "trial" ---
continueRoutine = True
routineForceEnded = False
# update component parameters for each repeat
dots.refreshDots()
# setup some python lists for storing info about the mouse
mouse.x = []
mouse.y = []
mouse.leftButton = []
mouse.midButton = []
mouse.rightButton = []
mouse.time = []
gotValidClick = False  # until a click is received
# keep track of which components have finished
trialComponents = [dots, polygon, polygon_2, mouse]
for thisComponent in trialComponents:
    thisComponent.tStart = None
    thisComponent.tStop = None
    thisComponent.tStartRefresh = None
    thisComponent.tStopRefresh = None
    if hasattr(thisComponent, 'status'):
        thisComponent.status = NOT_STARTED
# reset timers
t = 0
_timeToFirstFrame = win.getFutureFlipTime(clock="now")
frameN = -1

# --- Run Routine "trial" ---
while continueRoutine:
    # get current time
    t = routineTimer.getTime()
    tThisFlip = win.getFutureFlipTime(clock=routineTimer)
    tThisFlipGlobal = win.getFutureFlipTime(clock=None)
    frameN = frameN + 1  # number of completed frames (so 0 is the first frame)
    # update/draw components on each frame
    
    # *dots* updates
    if dots.status == NOT_STARTED and tThisFlip >= 0.0-frameTolerance:
        # keep track of start time/frame for later
        dots.frameNStart = frameN  # exact frame index
        dots.tStart = t  # local t and not account for scr refresh
        dots.tStartRefresh = tThisFlipGlobal  # on global time
        win.timeOnFlip(dots, 'tStartRefresh')  # time at next scr refresh
        # add timestamp to datafile
        thisExp.timestampOnFlip(win, 'dots.started')
        dots.setAutoDraw(True)
    
    # *polygon* updates
    if polygon.status == NOT_STARTED and tThisFlip >= 0.0-frameTolerance:
        # keep track of start time/frame for later
        polygon.frameNStart = frameN  # exact frame index
        polygon.tStart = t  # local t and not account for scr refresh
        polygon.tStartRefresh = tThisFlipGlobal  # on global time
        win.timeOnFlip(polygon, 'tStartRefresh')  # time at next scr refresh
        # add timestamp to datafile
        thisExp.timestampOnFlip(win, 'polygon.started')
        polygon.setAutoDraw(True)
    if polygon.status == STARTED:
        # is it time to stop? (based on global clock, using actual start)
        if tThisFlipGlobal > polygon.tStartRefresh + 10-frameTolerance:
            # keep track of stop time/frame for later
            polygon.tStop = t  # not accounting for scr refresh
            polygon.frameNStop = frameN  # exact frame index
            # add timestamp to datafile
            thisExp.timestampOnFlip(win, 'polygon.stopped')
            polygon.setAutoDraw(False)
    
    # *polygon_2* updates
    if polygon_2.status == NOT_STARTED and tThisFlip >= 10-frameTolerance:
        # keep track of start time/frame for later
        polygon_2.frameNStart = frameN  # exact frame index
        polygon_2.tStart = t  # local t and not account for scr refresh
        polygon_2.tStartRefresh = tThisFlipGlobal  # on global time
        win.timeOnFlip(polygon_2, 'tStartRefresh')  # time at next scr refresh
        # add timestamp to datafile
        thisExp.timestampOnFlip(win, 'polygon_2.started')
        polygon_2.setAutoDraw(True)
    if polygon_2.status == STARTED:
        # is it time to stop? (based on global clock, using actual start)
        if tThisFlipGlobal > polygon_2.tStartRefresh + 20-frameTolerance:
            # keep track of stop time/frame for later
            polygon_2.tStop = t  # not accounting for scr refresh
            polygon_2.frameNStop = frameN  # exact frame index
            # add timestamp to datafile
            thisExp.timestampOnFlip(win, 'polygon_2.stopped')
            polygon_2.setAutoDraw(False)
    # Run 'Each Frame' code from code
    message_from_feagi = feagi_opu_channel.receive()
    # Do the drawing
    pixels = np.array(win._getFrame())
    retina_data = retina.frame_split(pixels, capabilities['camera']['retina_width_percent'],
                                         capabilities['camera']['retina_height_percent'])
    for i in retina_data:
        if 'C' in i:
                retina_data[i] = retina.center_data_compression(retina_data[i],
                                                                capabilities['camera']["central_vision_compression"]
                                                                )
        else:
            retina_data[i] = retina.center_data_compression(retina_data[i],
                                                            capabilities['camera']
                                                            ['peripheral_vision_compression'])
    opu_data = FEAGI.opu_processor(message_from_feagi)
    if previous_data_frame == {}:
        for i in retina_data:
            previous_name = str(i) + "_prev"
            previous_data_frame[previous_name] = {}
    for i in retina_data:
        name = i
        if 'prev' not in i:
            data = retina.ndarray_to_list(retina_data[i])
            if 'C' in i:
                previous_name = str(i) + "_prev"
                rgb_data, previous_data_frame[previous_name] = retina.get_rgb(data,
                                                                              capabilities['camera'][
                                                                                  'central_vision_compression'],
                                                                              previous_data_frame[previous_name],
                                                                              name,
                                                                              capabilities[
                                                                                  'camera']['deviation_threshold'])
            else:
                previous_name = str(i) + "_prev"
                rgb_data, previous_data_frame[previous_name] = retina.get_rgb(data,
                                                                              capabilities['camera'][
                                                                                  'peripheral_vision_compression'],
                                                                              previous_data_frame[previous_name],
                                                                              name,
                                                                              capabilities[
                                                                                  'camera']['deviation_threshold'])
            for a in rgb_data['camera']:
                rgb['camera'][a] = rgb_data['camera'][a]
    try:
        if "data" not in message_to_feagi:
            message_to_feagi["data"] = dict()
        if "sensory_data" not in message_to_feagi["data"]:
            message_to_feagi["data"]["sensory_data"] = dict()
        message_to_feagi["data"]["sensory_data"]['camera'] = rgb['camera']
    except Exception as e:
        print(e)
    if opu_data is not None:
        if 'motor' in opu_data:
            if opu_data['motor']:
                for i in opu_data['motor']:
                    if i // 2 == 0:
                        if i % 2 == 0:
                            X += opu_data['motor'][i] / 100
                        else:
                            X -= opu_data['motor'][i] / 100
                    if i // 2 == 1:
                        if i % 2 == 0:
                            Y += opu_data['motor'][i] / 100
                        else:
                            Y -= opu_data['motor'][i] / 100
        if opu_data['misc']:
            for i in opu_data['misc']:
                if i == 0:
                    mouse1 = 1
                if i == 1:
                    mouse2 = 1
                if i == 2:
                    mouse3 = 1
        if 'oculomotor' in opu_data:
            for i in opu_data['oculomotor']:
                print("I: ", i)
                if i == 0:
                    capabilities['camera']['field_of_vision_origin'][0] = \
                        capabilities['camera']['field_of_vision_origin'][0] 
                        + opu_data['oculomotor'][i]
                if i == 1:
                    capabilities['camera']['field_of_vision_origin'][0] = \
                        capabilities['camera']['field_of_vision_origin'][0] 
                        - opu_data['oculomotor'][i]
                if i == 2:
                    capabilities['camera']['field_of_vision_origin'][1] = \
                        capabilities['camera']['field_of_vision_origin'][1] 
                        - opu_data['oculomotor'][i]
                if i == 3:
                    capabilities['camera']['field_of_vision_origin'][1] = \
                        capabilities['camera']['field_of_vision_origin'][1] 
                        - opu_data['oculomotor'][i]
    message_to_feagi['timestamp'] = datetime.now()
    message_to_feagi['counter'] = msg_counter
    feagi_ipu_channel.send(message_to_feagi)
    message_to_feagi.clear()
    for i in rgb['camera']:
        rgb['camera'][i].clear()
    # *mouse* updates
    if mouse.status == NOT_STARTED and t >= 0.0-frameTolerance:
        # keep track of start time/frame for later
        mouse.frameNStart = frameN  # exact frame index
        mouse.tStart = t  # local t and not account for scr refresh
        mouse.tStartRefresh = tThisFlipGlobal  # on global time
        win.timeOnFlip(mouse, 'tStartRefresh')  # time at next scr refresh
        # add timestamp to datafile
        thisExp.addData('mouse.started', t)
        mouse.status = STARTED
        mouse.mouseClock.reset()
        prevButtonState = mouse.getPressed()  # if button is down already this ISN'T a new click
    if mouse.status == STARTED:  # only update if started and not finished!
        buttons = mouse.getPressed()
        if buttons != prevButtonState:  # button state changed?
            prevButtonState = buttons
            if sum(buttons) > 0:  # state changed to a new click
                x, y = mouse.getPos()
                mouse.x.append(x)
                mouse.y.append(y)
                buttons = mouse.getPressed()
                mouse.leftButton.append(buttons[0])
                mouse.midButton.append(buttons[1])
                mouse.rightButton.append(buttons[2])
                mouse.time.append(mouse.mouseClock.getTime())
                
                continueRoutine = False  # abort routine on response
    
    # check for quit (typically the Esc key)
    if endExpNow or defaultKeyboard.getKeys(keyList=["escape"]):
        core.quit()
    
    # check if all components have finished
    if not continueRoutine:  # a component has requested a forced-end of Routine
        routineForceEnded = True
        break
    continueRoutine = False  # will revert to True if at least one component still running
    for thisComponent in trialComponents:
        if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
            continueRoutine = True
            break  # at least one component has not yet finished
    
    # refresh the screen
    if continueRoutine:  # don't flip if this routine is over or we'll get a blank screen
        win.flip()

# --- Ending Routine "trial" ---
for thisComponent in trialComponents:
    if hasattr(thisComponent, "setAutoDraw"):
        thisComponent.setAutoDraw(False)
# store data for thisExp (ExperimentHandler)
thisExp.addData('mouse.x', mouse.x)
thisExp.addData('mouse.y', mouse.y)
thisExp.addData('mouse.leftButton', mouse.leftButton)
thisExp.addData('mouse.midButton', mouse.midButton)
thisExp.addData('mouse.rightButton', mouse.rightButton)
thisExp.addData('mouse.time', mouse.time)
thisExp.nextEntry()
# the Routine "trial" was not non-slip safe, so reset the non-slip timer
routineTimer.reset()

# --- End experiment ---
# Flip one final time so any remaining win.callOnFlip() 
# and win.timeOnFlip() tasks get executed before quitting
win.flip()

# these shouldn't be strictly necessary (should auto-save)
thisExp.saveAsWideText(filename+'.csv', delim='auto')
thisExp.saveAsPickle(filename)
logging.flush()
# make sure everything is closed down
if eyetracker:
    eyetracker.setConnectionState(False)
thisExp.abort()  # or data files will save again on exit
win.close()
core.quit()
