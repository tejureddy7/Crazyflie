'''--------------------------------------------------------------
 File:    stream.py
 Author:  Griffin Bonner      <griffi1@umbc.edu>                           
 Date:    8.3.2021
 Description: streaming process: receive images from AI-deck, 
 computes NN-outputs, manages queue and rolling sum of (n) 
 recent network outputs, sends values to control process. 
--------------------------------------------------------------'''

# streaming dependencies 
import argparse
import gi 
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, Gdk, GdkPixbuf, GLib
import threading
import time
from datetime import datetime

import socket,os,struct
from six.moves import builtins

# inference dependencies
import numpy
import tensorflow as tf
from tensorflow.keras.models import load_model
from collections import deque

# multiprocessing dependencies 
from multiprocessing.connection import Client

# deck identifiers
deck_ip = None
deck_port = None

# launch time
start_time = time.time()

# Create numpy array from GdkPixbuf object
def array_from_pixbuf(p):
    ''' nd-numpy array from GdkPixbuf '''
    w,h,c,r=(p.get_width(), p.get_height(), p.get_n_channels(), p.get_rowstride())
    assert p.get_colorspace() == GdkPixbuf.Colorspace.RGB
    assert p.get_bits_per_sample() == 8
    if  p.get_has_alpha():
        assert c == 4
    else:
        assert c == 3
    assert r >= w * c
    a=numpy.frombuffer(p.get_pixels(),dtype=numpy.uint8)
    if a.shape[0] == w*c*h:
        return a.reshape( (h, w, c) )
    else:
        b=numpy.zeros((h,w*c),'uint8')
        for j in range(h):
            b[j,:]=a[r*j:r*j+w*c]
        return b.reshape( (h, w, c) )

# Create GdkPixbuf object from numpy array
def pixbuf_from_array(z):
    ''' GdkPixbuf from nd-numpy array '''
    z=z.astype('uint8')
    h,w,c=z.shape
    assert c == 3 or c == 4
    if hasattr(GdkPixbuf.Pixbuf,'new_from_bytes'):
        Z = GLib.Bytes.new(z.tobytes())
        return GdkPixbuf.Pixbuf.new_from_bytes(Z, GdkPixbuf.Colorspace.RGB, c==4, 8, w, h, w*c)
    return GdkPixbuf.Pixbuf.new_from_data(z.tobytes(),  GdkPixbuf.Colorspace.RGB, c==4, 8, w, h, w*c, None, None)

# Double ended queue stores inference data
class InfQueue():
    ''' manages sum of queue for rolling average '''
    def __init__(self):
        self.pred_queue = deque()
        #self.steer_queue =deque()
        self.sum = 0
    
    def enqueue(self, pred):
        if (len(self.pred_queue) <= 4):
            self.pred_queue.append(pred)
            self.sum += pred
        else:
            self.sum -= self.pred_queue.popleft()
            self.pred_queue.append(pred)
            self.sum += pred
            cf_tx.send(self.sum)
        now = datetime.now()
        print("que",str(now.strftime("%M:%S.%f")))


# Connect to AI-Deck and parse frame data
class ImgThread(threading.Thread):
    
    def __init__(self, callback):
        threading.Thread.__init__(self, daemon=True)
        now = datetime.now()
        print("fir5",str(now.strftime("%M:%S.%f")))
        self._callback = callback

    def run(self):
        print("Connecting to socket on {}:{}...".format(deck_ip, deck_port))
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect((deck_ip, deck_port))
        print("Socket connected")
        
        imgdata = None
        data_buffer = bytearray()

        while(1):
            now = datetime.now()
            print("fir",str(now.strftime("%M:%S.%f")))
            # Receive image data from the AI-deck
            data_buffer.extend(client_socket.recv(512))
            #now = datetime.now()
            #print("fir2",str(now.strftime("%M:%S.%f")))

            # Look for start-of-frame and end-of-frame
            start_idx = data_buffer.find(b"\xff\xd8")
            end_idx = data_buffer.find(b"\xff\xd9")

            # At startup we might get an end before we get the first start, if
            # that is the case then throw away the data before start
            if end_idx > -1 and end_idx < start_idx:
                now = datetime.now()
                print("fir2a",str(now.strftime("%M:%S.%f")))
                data_buffer = data_buffer[start_idx:]
                print("exception")
            

            # We have a start and an end of the image in the buffer now
            if start_idx > -1 and end_idx > -1 and end_idx > start_idx:
                # Pick out the image to render ...
                now = datetime.now()
                print("fir2b",str(now.strftime("%M:%S.%f")))
                imgdata = data_buffer[start_idx:end_idx + 2]
                print(len(data_buffer))
                now = datetime.now()
                print("fir2c",str(now.strftime("%M:%S.%f")))
                #print(imgdata)
                # .. and remove it from the buffer
                data_buffer = data_buffer[end_idx + 2 :]
                try:
                    self._callback(imgdata)
                    now = datetime.now()
                    print("fir2d",str(now.strftime("%M:%S.%f")))
                except gi.repository.GLib.Error:
                    print("Error rendering image")
               
                now = datetime.now()
                print("fir2e",str(now.strftime("%M:%S.%f")))
            
        
     


# UI for showing frames streamed from AI-Deck
class FrameViewer(Gtk.Window):

    def __init__(self):
        super(FrameViewer, self).__init__()
        self.frame = None
        self.init_ui()
        self._start = None
        self.set_default_size(374, 294)

    def init_ui(self):            
        self.override_background_color(Gtk.StateType.NORMAL, Gdk.RGBA(0, 0, 0, 1))
        self.set_border_width(20)
        self.set_title("Connecting...")
        self.frame = Gtk.Image()
        f = Gtk.Fixed()
        f.put(self.frame, 10, 10)
        self.add(f)
        self.connect("destroy", Gtk.main_quit)
        #now = datetime.now()
        #print("firi",str(now.strftime("%M:%S.%f")))
        #now = datetime.now()
        self._thread = ImgThread(self._showframe)
        #print("firi2",str(now.strftime("%M:%S.%f")))
        self._thread.start()

    def _update_image(self, pix):
        self.frame.set_from_pixbuf(pix)
        now = datetime.now()
        print("fir3",str(now.strftime("%M:%S.%f")))


    # Add FPS/img size to window title and display frame
    def _showframe(self, imgdata):
        #now = datetime.now()
        #print("fir4",str(now.strftime("%M:%S.%f")))
        if (self._start != None):
            fps = 1 / (time.time() - self._start)
            sync_time = time.time() - start_time
            GLib.idle_add(self.set_title, "{:.1f} fps / {:.1f} kb / {:.1f} prediction".format(fps, len(imgdata)/1000, (inference.sum/5)))
            #GLib.idle_add(self.set_title, "{:.1f} fps / {:.1f} kb / {:.1f} prediction / {:.1f} angle".format(fps, len(imgdata)/1000, (inference.sum/5)), (((inference_s.sum2))) # inference_s.sum2/3
        self._start = time.time()
        #now = datetime.now()
        #print("fir4",str(now.strftime("%M:%S.%f")))
        img_loader = GdkPixbuf.PixbufLoader()
        #now = datetime.now()
        #print("fir5",str(now.strftime("%M:%S.%f")))
        # Attempt to decode JPEG from the data sent from the stream
        try:
            img_loader.write(imgdata)
            #print("check2")
            pix = img_loader.get_pixbuf()
            #print("check3")
            if (pix != None):
                pix_arr = array_from_pixbuf(pix)
                # cast triple channel .jpg data to single channel numpy array (324,244,1)
                image = pix_arr[:,:,0:1].astype(numpy.float32)[tf.newaxis,...] / 255.0 
                time3 = time.time() - start_time
                # generate inference from model
                print("time")
                prediction  = model(image).numpy()[0,0]
                time4 = time.time() - start_time
                now = datetime.now()
                print("model time",str(now.strftime("%M:%S.%f")))
                print("model inference time", time4 -time3)
                # append inference data to queue
                inference.enqueue(prediction)
                #time5 = time.time() - start_time)
                GLib.idle_add(self._update_image, pix)
        except gi.repository.GLib.Error:
            print("Could not set image!")
        img_loader.close() 



if __name__ == '__main__':

    # open inference->control pipeline
    cf_tx = Client(('localhost', 6000), authkey=b'crazyflie')
    print("start",time.time() - start_time)
    # load neural-network model
    model = load_model("model_old.h5")
    

    # arguments for setting IP/port of AI-deck, default=Access Point Mode
    parser = argparse.ArgumentParser(description='Connect to AI-deck JPEG streamer example')
    parser.add_argument("-n",  default="192.168.4.1", metavar="ip", help="AI-deck IP")
    parser.add_argument("-p", type=int, default='5000', metavar="port", help="AI-deck port")
    args = parser.parse_args()
    deck_port = args.p
    deck_ip = args.n

    
    # queue to store inference data 
    inference = InfQueue()
    
    print("start1", time.time() - start_time)

    # initialize streaming objects 
    fw = FrameViewer()
    fw.show_all()
    Gtk.main()
    print("last", time.time())
    # close inference->control pipeline
    cf_tx.close()
    