#!/usr/bin/pythonRoot
# bring in the libraries
import RPi.GPIO as G     
from flup.server.fcgi import WSGIServer 
import sys, urlparse
import i2cmodule as i2c

# set up our GPIO pins
G.setmode(G.BCM)
G.setup(18, G.OUT)

# all of our code now lives within the app() function which is called for each http request we receive
def app(environ, start_response):
  # start our http response 
  start_response("200 OK", [("Content-Type", "text/html")])
  # look for inputs on the URL
  i = urlparse.parse_qs(environ["QUERY_STRING"])
  yield ('test') # flup expects a string to be returned from this function
  # if there's a url variable named 'q'
  if "q" in i:
    mecanumBaseAddr = 0x20
    i2c.initI2C()
    i2c.openPort(mecanumBaseAddr)
    if i["q"][0] == "jointReset":
       i2c.jointReset()
    elif i["q"][0] == "jointSetPWM":
       i2c.jointSetPWM(float(i["arg1"][0]),float(i["arg2"][0]),float(i["arg3"][0]),float(i["arg4"][0]))       
    elif i["q"][0] == "jointSetSpeed":
       i2c.setControlMode(1)
       i2c.jointSetSpeed(float(i["arg1"][0]),float(i["arg2"][0]),float(i["arg3"][0]),float(i["arg4"][0]))
    elif i["q"][0] == "s":
      G.output(18, False)  # Turn it off
    elif i["q"][0] == "s":
      G.output(18, False)  # Turn it off
    elif i["q"][0] == "s":
      G.output(18, False)  # Turn it off
    elif i["q"][0] == "s":
      G.output(18, False)  # Turn it off

#by default, Flup works out how to bind to the web server for us, so just call it with our app() function and let it get on with it
WSGIServer(app).run()

