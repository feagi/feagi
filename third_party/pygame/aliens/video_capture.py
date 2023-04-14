import random

# import gymnasium as gym
import aliens
from fastapi import FastAPI, Request
from fastapi.responses import HTMLResponse, StreamingResponse
from fastapi.templating import Jinja2Templates
import cv2
import threading
import requests
from configuration import *
from time import sleep
from datetime import datetime
from feagi_agent import retina as retina
from feagi_agent import feagi_interface as feagi
import pygame
import os

rgb_array = {}
movement = {}
os.environ["SDL_VIDEODRIVER"] = "dummy"
pygame.init()
screen = pygame.Surface((640, 480))

app = FastAPI()

templates = Jinja2Templates(directory="templates")

# Mount static directory for serving static files
# app.mount("/static", StaticFiles(directory="static"), name="static")

# Load the video stream
cap = cv2.VideoCapture(0)

# Define image width and height
WIDTH = 640
HEIGHT = 480

font = pygame.font.Font(None, 36)
number_surface = font.render("42", True, (255, 255, 255))
number_size = number_surface.get_size()
screen.blit(number_surface, (number_size[0] // 2, number_size[1] // 2))


# Define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*"XVID")
out = cv2.VideoWriter("output.avi", fourcc, 20.0, (WIDTH, HEIGHT))

# env = gym.make("CarRacing-v2", render_mode="rgb_array")
# observation, info = env.reset()


# Define the function to capture frames from the camera
def capture_webcam():
    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()

        # Resize the frame
        frame = cv2.resize(frame, (WIDTH, HEIGHT))

        # Write the frame to the video file
        out.write(frame)

        # Convert the frame to JPEG format
        _, buffer = cv2.imencode(".jpg", frame)

        # Yield the buffer as bytes
        yield b"--frame\r\n" b"Content-Type: image/jpeg\r\n\r\n" + buffer.tobytes() + b"\r\n"


def capture_pygame(rgb_array):
    while True:
        # # Convert the numpy array to an OpenCV image
        image = cv2.cvtColor(rgb_array['current'], cv2.COLOR_RGB2BGR)
        image = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
        #
        _, buffer = cv2.imencode('.jpg', image)
        yield b"--frame\r\n" b"Content-Type: image/jpeg\r\n\r\n" + buffer.tobytes() + b"\r\n"
        # print("working")


# Define the route for the index page
@app.get("/", response_class=HTMLResponse)
async def read_item(request: Request):
    return templates.TemplateResponse("webcam.html", {"request": request})


# Define the route for the video feed
# @app.get("/webcam_feed")
# async def video_feed():
#     return StreamingResponse(capture_webcam(), media_type="multipart/x-mixed-replace;boundary=frame")

# f
# # Define the route for the video feed
@app.get("/pygame_feed")
async def video_feed():
    return StreamingResponse(capture_pygame(rgb_array), media_type="multipart/x-mixed-replace;boundary=frame")


if __name__ == "__main__":
    import uvicorn
    bsgk2 = threading.Thread(target=uvicorn.run, args=(app,), kwargs={"host": "0.0.0.0", "port": 6084}, daemon=True)
    bsgk2.start()
    previous_data_frame = dict()
    runtime_data = {"cortical_data": {}, "current_burst_id": None, "stimulation_period": None, "feagi_state": None,
                    "feagi_network": None}

    # FEAGI section start
    print("Connecting to FEAGI resources...")

    feagi_host, api_port, app_data_port = feagi.feagi_setting_for_registration(feagi_settings, agent_settings)

    print(feagi_host, api_port, app_data_port)

    # address = 'tcp://' + network_settings['feagi_host'] + ':' + network_settings['feagi_opu_port']

    api_address = 'http://' + feagi_host + ':' + api_port

    stimulation_period_endpoint = feagi.feagi_api_burst_engine()
    burst_counter_endpoint = feagi.feagi_api_burst_counter()
    print("^ ^ ^")
    runtime_data["feagi_state"] = feagi.feagi_registration(feagi_host=feagi_host,
                                                           api_port=api_port, agent_settings=agent_settings,
                                                           capabilities=capabilities)

    print("** **", runtime_data["feagi_state"])
    feagi_settings['feagi_burst_speed'] = float(runtime_data["feagi_state"]['burst_duration'])

    # todo: to obtain this info directly from FEAGI as part of registration
    # ipu_channel_address = feagi.feagi_inbound(agent_settings["agent_data_port"])
    ipu_channel_address = feagi.feagi_outbound(feagi_settings['feagi_host'],
                                               agent_settings["agent_data_port"])
    print("IPU_channel_address=", ipu_channel_address)
    opu_channel_address = feagi.feagi_outbound(feagi_settings['feagi_host'],
                                               runtime_data["feagi_state"]['feagi_opu_port'])
    feagi_ipu_channel = feagi.pub_initializer(ipu_channel_address, bind=False)
    feagi_opu_channel = feagi.sub_initializer(opu_address=opu_channel_address)

    previous_frame_data = dict()
    msg_counter = runtime_data["feagi_state"]['burst_counter']
    rgb = dict()
    checkpoint_total = 5
    flag_counter = 0
    rgb['camera'] = dict()
    save_pg_path = aliens.pg.mixer
    bgsk = threading.Thread(target=aliens.main, args=(0, rgb_array, movement, save_pg_path), daemon=True)
    bgsk.start()  # start the aliens
    while True:
        if not bgsk.is_alive():
            aliens.pg.quit()
            bgsk = threading.Thread(target=aliens.main, args=(0, rgb_array, movement, save_pg_path), daemon=True)
            bgsk.start()
            print("RENEW!")
        message_from_feagi = feagi_opu_channel.receive()  # Get data from FEAGI
        # print("START")
        # action = env.action_space.sample()  # agent policy that uses the observation and info
        # print("action: ", action)
        # OPU section STARTS
        if message_from_feagi is not None:  # Added OPU
            opu_data = feagi.opu_processor(message_from_feagi)
            if 'misc' in opu_data:
                for i in opu_data['misc']:
                    if i == 0:
                        movement[i] = 1
                    elif i == 1:
                        movement[i] = -1
                    elif i == 2:
                        movement[i] = True
        # OPU section ENDS
        if rgb_array:
            new_rgb = cv2.rotate(rgb_array['current'], cv2.ROTATE_90_CLOCKWISE) # rotate 90 clockwise
            new_rgb = cv2.flip(new_rgb, 1) # flip horizontally
            retina_data = retina.frame_split(new_rgb, capabilities['camera']['retina_width_percent'],
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
                pass
            # Psychopy game ends
        message_to_feagi, battery = feagi.compose_message_to_feagi({**rgb}, battery=aliens.healthpoint*10)
        message_to_feagi['timestamp'] = datetime.now()
        message_to_feagi['counter'] = msg_counter
        msg_counter += 1
        flag_counter += 1
        if flag_counter == int(checkpoint_total):
            feagi_burst_speed = requests.get(api_address + stimulation_period_endpoint).json()
            feagi_burst_counter = requests.get(api_address + burst_counter_endpoint).json()
            flag_counter = 0
            if feagi_burst_speed > 1:
                checkpoint_total = 5
            if feagi_burst_speed < 1:
                checkpoint_total = 5 / feagi_burst_speed
            if msg_counter < feagi_burst_counter:
                feagi_opu_channel = feagi.sub_initializer(opu_address=opu_channel_address)
                if feagi_burst_speed != feagi_settings['feagi_burst_speed']:
                    feagi_settings['feagi_burst_speed'] = feagi_burst_speed
            if feagi_burst_speed != feagi_settings['feagi_burst_speed']:
                feagi_settings['feagi_burst_speed'] = feagi_burst_speed
                msg_counter = feagi_burst_counter
        sleep(feagi_settings['feagi_burst_speed'])
        try:
            pass
            # print(len(message_to_feagi['data']['sensory_data']['camera']['C']))
        except:
            pass
        feagi_ipu_channel.send(message_to_feagi)
        message_to_feagi.clear()
        for i in rgb['camera']:
            rgb['camera'][i].clear()
