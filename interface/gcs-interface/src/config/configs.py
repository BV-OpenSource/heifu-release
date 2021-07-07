DRONE_TYPE='heifu'
MODULE_LIST = {'heifu': 'heifu_interface',
               'vtol':  'vtol_interface'}

MODULE_NAME = MODULE_LIST[DRONE_TYPE]

# Select the endpoint target: prod, preprod, qa, local
ENDPOINT = 'preprod'

#Endpoints
API_VERSION = '/api/v1'
BACKEND_LIST = {'preprod': 'https://beyond-skyline-backend.preprod.pdmfc.com',
               'qa': 'https://beyond-skyline-backend.qa.pdmfc.com',
               'prod': 'https://beyond-skyline.com',
               'local': 'http://127.0.0.1:3000', 
               'graca': 'http://85.242.57.8:3000',
               'newpreprod': 'https://bexstream-preprod.beyond-vision.pt'}
JANUS_LIST = {'preprod': 'janus.preprod.pdmfc.com', 'qa': 'janus.qa.pdmfc.com', 'local': '127.0.0.1',
             'prod': '213.63.138.90', 'graca': '85.242.57.8', 'newpreprod':'213.63.130.233'}

URL = BACKEND_LIST[ENDPOINT] + API_VERSION
URL_WS = BACKEND_LIST[ENDPOINT] + API_VERSION
JANUS_IP = JANUS_LIST[ENDPOINT]

INSTA360_IP = '192.168.1.100'

# Cameras available: PC: default, RealSense: realsense, RaspyCam: raspy, Insta360 Pro: insta360, Stabilize: stab
CAM_TYPE = 'gazebo_integrated'
#Set to true to enable stablization
STABILIZE = False
#Set to true to enable record of stabilized video
RECORD = False
# To enable jetson streaming pipelines set to True
JETSON = False
# TODO Use gstreamer monitor to discover video capture devices and their paths
DEV_ID = 2  # Id for realsense
#Enable No Fly Zones
ENABLE_NFZ = True

