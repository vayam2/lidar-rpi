import cv2 
import dronekit as dk 
import time
import numpy
import os
import requests
    existing_numbers = [int(f.split("_")[-1].split(".")[0]) for f in existing_files]
    img_counter = max(existing_numbers) + 1  # Start from the next number
#time.sleep(1)
#os.system("gnome-terminal -e 'bash -c \"" + '~/.local/bin/mavproxy.py --out 127.0.0.1:6969' + ";bash\"'")
#print("heyyyyyyy")
time.sleep(15)
vid = cv2.VideoCapture(0) 
vehicle = dk.connect('127.0.0.1:6969', baud=57600) # connect to the vehicle, use mavproxy

img_counter = 0
counter = 0

while True:
    time.sleep(1)
    counter = counter + 1
    print(counter)
    if (counter % 120 == 0): 
        print(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon)
        ret, frame = vid.read() 
        frame = frame[50:640, 0:480]
        alt = vehicle.location.global_relative_frame.alt
        gim_p = vehicle.gimbal.pitch
        gim_y = vehicle.gimbal.yaw
        head = vehicle.heading
        veh_lat = vehicle.location.global_relative_frame.lat
        veh_lon = vehicle.location.global_relative_frame.lon
        
        img_name = "opencv_frame_{}.png".format(img_counter)
        try:
            cv2.imwrite(img_name, frame)
        except:
            continue

        img_counter += 1
        time.sleep(2)
        if (alt == None):
            alt = 0.0
        if (veh_lat == None):
            veh_lat = 0.0
        if (veh_lon == None):
            veh_lon = 0.0
        if (head == None):
            head = 0.0
        if (gim_p == None):
            gim_p = 0.0
        if (gim_y == None):
            gim_y = 0.0	
        try:
            url = "http://drone.bravishmah.com:8088/api/drone/UploadFile"
            headers = {"Accept": "*/*"}  
            data = {
                "location": "India",
                "latitude": veh_lat,
                "longitude": veh_lon, 
                "altitude": alt,
                "heading": head,
                "cameraPitch": gim_p,
                "cameraYaw": gim_y
            }

            files = {
                'file': open(img_name, 'rb')
            }
            response = requests.post(url, headers=headers, data=data, files=files)
            print(response)
        except:
            continue

vid.release() 
cv2.destroyAllWindows()
