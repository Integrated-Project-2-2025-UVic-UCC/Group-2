
# Import necessary libraries for
import cv2 # Image processing
import numpy as np # Numerical calculations and matrices
from picamera2 import Picamera2 # Control the Raspberry Pi camera
import pigpio # Precise GPIO control
import threading # Parallel execution
import time # For timing control
from queue import Queue # Command management

# Function for measuring distance with ultrasonic sensor
def mesurardistancia(TRIG, ECH):
    pi.write(TRIG, 0) 
    pi.gpio_trigger(TRIG, 10, 1)
    while pi.read(ECH) == 0:
        start_time = time.time()
    while pi.read(ECH) == 1:
        end_time = time.time()
    distancia_cm = (end_time - start_time) * 17150
    return round(distancia_cm, 2)

# Function to stop the motors
def stop_motors():
    pi.write(STEP_esq_dav, 0)
    pi.write(STEP_dr_dav, 0)
    pi.write(STEP_esq_darr, 0)
    pi.write(STEP_dr_darr, 0)
    pi.wave_tx_stop()

# Function to generate and send pulse sequences to motors based on commands
def run_wave(comm):
    pi.wave_clear()
    for step_pin, dir_pin, steps, speed_hz, direction in comm:
        pi.write(dir_pin, direction)
        pulse_us = int(500000 / speed_hz)
        pulses = [
            pigpio.pulse(1 << step_pin, 0, pulse_us),
            pigpio.pulse(0, 1 << step_pin, pulse_us)
        ]
        pi.wave_add_generic(pulses * abs(steps))
    wave_id = pi.wave_create()
    pi.wave_send_once(wave_id)
    while pi.wave_tx_busy():
        pass
    pi.wave_delete(wave_id)

# Function that continuously waits for motor commands from the queue and executes them
def motor_worker():
    while True:
        command = command_queue.get()
        if command == "STOP":
            break
        # Received command determine the type of movement and its characteristics
        action, steps, speed_hz, angle = command
        if action == "forward":
            if angle == "right":
                run_wave([
                    (STEP_esq_dav, DIR_esq_dav, steps, speed_hz, 1),
                    (STEP_dr_dav, DIR_dr_dav, steps, speed_hz - 100, 0),
                    (STEP_esq_darr, DIR_esq_darr, steps, speed_hz, 1),
                    (STEP_dr_darr, DIR_dr_darr, steps, speed_hz - 100, 0)
                    ])
            elif angle == "left":
                run_wave([
                    (STEP_esq_dav, DIR_esq_dav, steps, speed_hz - 100, 1),
                    (STEP_dr_dav, DIR_dr_dav, steps, speed_hz, 0),
                    (STEP_esq_darr, DIR_esq_darr, steps, speed_hz - 100, 1),
                    (STEP_dr_darr, DIR_dr_darr, steps, speed_hz, 0)
                    ])
            else:
                run_wave([
                    (STEP_esq_dav, DIR_esq_dav, steps, speed_hz, 1),
                    (STEP_dr_dav, DIR_dr_dav, steps, speed_hz, 0),
                    (STEP_esq_darr, DIR_esq_darr, steps, speed_hz, 1),
                    (STEP_dr_darr, DIR_dr_darr, steps, speed_hz, 0)
                    ])
        elif action == "backward":
            if angle == "right":
                run_wave([
                    (STEP_esq_dav, DIR_esq_dav, steps, speed_hz - 100, 0),
                    (STEP_dr_dav, DIR_dr_dav, steps, speed_hz, 1),
                    (STEP_esq_darr, DIR_esq_darr, steps, speed_hz - 100, 0),
                    (STEP_dr_darr, DIR_dr_darr, steps, speed_hz, 1)
                    ])
            elif angle == "left":
                run_wave([
                    (STEP_esq_dav, DIR_esq_dav, steps, speed_hz, 0),
                    (STEP_dr_dav, DIR_dr_dav, steps, speed_hz - 100, 1),
                    (STEP_esq_darr, DIR_esq_darr, steps, speed_hz, 0),
                    (STEP_dr_darr, DIR_dr_darr, steps, speed_hz - 100, 1)
                    ])
            else:
                run_wave([
                    (STEP_esq_dav, DIR_esq_dav, steps, speed_hz, 0),
                    (STEP_dr_dav, DIR_dr_dav, steps, speed_hz, 1),
                    (STEP_esq_darr, DIR_esq_darr, steps, speed_hz, 0),
                    (STEP_dr_darr, DIR_dr_darr, steps, speed_hz, 1)
                    ])
        elif action == "repose":
            if angle == "right":
                run_wave([
                    (STEP_esq_dav, DIR_esq_dav, steps, speed_hz, 1),
                    (STEP_dr_dav, DIR_dr_dav, steps, speed_hz, 1),
                    (STEP_esq_darr, DIR_esq_darr, steps, speed_hz, 1),
                    (STEP_dr_darr, DIR_dr_darr, steps, speed_hz, 1)
                    ])
            elif angle == "left":
                run_wave([
                    (STEP_esq_dav, DIR_esq_dav, steps, speed_hz, 0),
                    (STEP_dr_dav, DIR_dr_dav, steps, speed_hz, 0),
                    (STEP_esq_darr, DIR_esq_darr, steps, speed_hz, 0),
                    (STEP_dr_darr, DIR_dr_darr, steps, speed_hz, 0)
                    ])
            else:
                stop_motors()
                
# Variable declarations
font = cv2.FONT_HERSHEY_SIMPLEX
width = 1200
height = 800
x_offset = (4608-width)//2
y_offset = 2592-height
hz = 300

# Define GPIO pins for motor direction and stepping control
DIR_esq_dav = 18
STEP_esq_dav = 4
DIR_dr_dav = 22
STEP_dr_dav = 23
DIR_esq_darr = 27
STEP_esq_darr = 17
DIR_dr_darr = 25
STEP_dr_darr = 24
# Also for ultrasonics sensors pins
#TRIGGER1 = 17
#ECHO1 = 27 
#TRIGGER2 = 23
#ECHO2 =24

# Initializations and set configurations
picam2 = Picamera2()
pi = pigpio.pi()
command_queue = Queue(maxsize = 1)
motor_thread = threading.Thread(target = motor_worker, daemon = True)
motor_thread.start()
config = picam2.create_preview_configuration(
    main={"format": "RGB888", "size": (width, height)},
    controls={
        "AwbEnable":False,
        "AeEnable": False,
        "ExposureTime": 100,
        #"AnalogueGain": 2.0,
        "Saturation": 1,
        "FrameRate": 30,
        "ScalerCrop": (x_offset, y_offset, width, height)
        }
)
picam2.configure(config)
picam2.start()

# Set pin modes
pi.set_mode(DIR_esq_dav, pigpio.OUTPUT)
pi.set_mode(STEP_esq_dav, pigpio.OUTPUT)
pi.set_mode(DIR_dr_dav, pigpio.OUTPUT)
pi.set_mode(STEP_dr_dav, pigpio.OUTPUT)
pi.set_mode(DIR_esq_darr, pigpio.OUTPUT)
pi.set_mode(STEP_esq_darr, pigpio.OUTPUT)
pi.set_mode(DIR_dr_darr, pigpio.OUTPUT)
pi.set_mode(STEP_dr_darr, pigpio.OUTPUT)
#pi.set_mode(TRIGGER1, pigpio.OUTPUT)
#pi.set_mode(ECHO1, pigpio.INPUT)
#pi.set_mode(TRIGGER2, pigpio.OUTPUT)
#pi.set_mode(ECHO2, pigpio.INPUT)
        
try:
    while True:
        # Distance calculation of the ultrasonic sensor
        #distanciainf = mesurardistancia(TRIGGER1, ECHO1)
        #distanciasup = mesurardistancia(TRIGGER2, ECHO2)
        # Capture frame from camera and convert color space
        frame_rgb = picam2.capture_array()
        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        red = frame_rgb[:,:,2]
        green = frame_rgb[:,:,1]
        blue = frame_rgb[:,:,0]
        # Process image to detect red objects using color thresholding
        chan = red
        _, threshold = cv2.threshold(chan, 200, 255, cv2.THRESH_BINARY)
        # Find contours
        cnts = cv2.findContours(threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
        cnts = sorted(cnts, key = cv2.contourArea, reverse = True)
        posxy = []
        for c in cnts:
            # Draw the rectangles
            x, y, w, h = cv2.boundingRect(c)
            posxy.append([x + w // 2, y + h // 2])
            cv2.rectangle(frame_bgr, (x, y), (x+w, y+h), (0, 255, 0), 2)
        if len(posxy) >= 2:
            hz = 300
            # K-means to group the contours into two clusters
            Z = np.vstack(posxy)
            Z = np.float32(Z)
            criteris = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
            ret, etiqueta, centre = cv2.kmeans (Z, 2, None , criteris, 10, cv2.KMEANS_RANDOM_CENTERS)
            A = Z[etiqueta.ravel() == 0]
            B = Z[etiqueta.ravel() == 1]
            centre1 = centre[0,:]
            centre2 = centre[1,:]
            punt_mig_x = (int(centre1[0]) + int(centre2[0])) // 2
            punt_mig_y = (int(centre1[1]) + int(centre2[1])) // 2
            cv2.circle(frame_bgr, (punt_mig_x, punt_mig_y), 5, (0, 255, 0), -1)
            # Calculate the distance
            separacio = abs(int(centre1[0])-int(centre2[0]))
            # Determine the frequency
            if separacio <= 150 or separacio >= 490:
                hz = hz + 100
            # and direction of rotation
            if punt_mig_x > width // 2 + 100:
                cv2.putText(frame_bgr,'Girar a la dreta', (int(punt_mig_x),150),1,1.5,(0,255,0),2)
                gir = "right"
            elif punt_mig_x < width // 2 - 100:
                cv2.putText(frame_bgr,'Girar a l\'Esquerra', (int(punt_mig_x),150),1,1.5,(0,255,0),2)
                gir = "left"
            else:
                cv2.putText(frame_bgr,'Centrat', (int(punt_mig_x),150),1,1.5,(0,255,0),2)
                gir = None
            # Determine motor commands
            if command_queue.full():
                command_queue.get()
            #if distanciasup < 20:
             #   command_queue.put(("repose", 400, hz, None))
            #elif distanciainf < 20:
             #   command_queue.put(("forward", 400, hz, None))
            if separacio is not None and 80 <= separacio < 300:
                command_queue.put(("forward", 400, hz, gir))
            elif separacio is not None and separacio > 340:
                command_queue.put(("backward", 400, hz, gir))
            elif separacio is not None and 300 <= separacio <= 340:
                command_queue.put(("repose", 400, hz, gir))
            else:
                command_queue.put(("repose", 400, hz, None))
        else:
            if command_queue.full():
                command_queue.get() 
            command_queue.put(("repose", 400, hz, None))
        # Display processed frame with visual indicators
        cv2.imshow('Deteccions', frame_bgr)
        # Exit loop and stop motors when 's' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('s'):
            if command_queue.full():
                command_queue.get() 
            command_queue.put("STOP")
            motor_thread.join()
            break
                
finally:
    # Stop hardware and close resources
    stop_motors()
    picam2.stop()
    cv2.destroyAllWindows()
    pi.stop() 
