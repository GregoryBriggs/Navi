#qrcodeGUI.py

from tkinter import messagebox
import subprocess
from PIL import Image
import time
import datetime
import picamera as picam


class SET():
    PV_SIZE=(320,240)
    NORM_SIZE=(2592,1944)
    NO_RESIZE=(0,0)
    PREVIEW_FILE="PREVIEW.gif"
    TEMP_FILE="PREVIEW.ppm"
    QR_SIZE=(640,480)
    READ_QR="zbarimg "

class cameraGUI():

    def __init__(self):
        
        self.scan=False
        self.resultQR =""

    def run_p(cmd):

        proc=subprocess.Popen(cmd,shell=True,stdout=subprocess.PIPE)
        result=""
        for line in proc.stdout:
            result=str(line,"utf-8")

        return result

    def camCapture(filename,size=SET.NORM_SIZE):
        with picam.PiCamera() as camera:
            camera.resolution = size
            camera.capture(filename)

    # Use the timestamp 
    def timestamp():
        ts=time.time()
        tstring=datetime.datetime.fromtimestamp(ts)
        return tstring.strftime("%Y%m%d_%H%M%S")

    def qrGet(self):

        if (self.scan==True):

            self.scan=False

        else:

            self.scan=True

            self.qrScanner()

    def qrScanner(self):
        # ensures the result is found by setting found to false

        cameraGUI.camCapture(SET.PREVIEW_FILE,SET.QR_SIZE)

        #check for QR code in image
        qrcode=cameraGUI.run_p(SET.READ_QR+SET.PREVIEW_FILE)

        if len(qrcode)>0:
            # delete this line after removing the line that adds the string "QR-Code"
            qrcode=qrcode

            self.resultQR = qrcode
            self.resultQR = self.resultQR.split(":")
            print(self.resultQR)
            self.resultQR = self.resultQR[1]
            
            self.scan=False

            found=True
            
        else:

            self.resultQR = ""

        return self.resultQR

    def normal(self):
        name=cameraGUI.timestamp()+".jpg"
        cameraGUI.camCapture(name,SET.NORM_SIZE)
        
    def exit(self):
        exit()



while True:
    qr_cam = cameraGUI()
    qr_code = qr_cam.qrGet()
    print("QR-Code: ", qr_code)
    print("QR-Code: ", qr_cam.resultQR)
#End
