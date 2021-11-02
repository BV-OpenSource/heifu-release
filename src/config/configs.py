DRONE_TYPE='heifu'
MODULE_LIST = {'heifu': 'heifu_interface',
             'vtol': 'vtol_interface'}

MODULE_NAME = MODULE_LIST[DRONE_TYPE]

# Select the endpoint target: prod, preprod, qa, local
#ENDPOINT = 'newpreprod-draco'
#ENDPOINT = 'newpreprod'

#Endpoints
JANUS_LIST = {'preprod': 'janus.preprod.pdmfc.com', 
                'qa': 'janus.qa.pdmfc.com', 
                'local': '127.0.0.1',
                'prod': '213.63.138.90', 
                'graca': '85.242.57.8', 
                'newpreprod':'213.63.130.233', 
                #'newpreprod-draco': '85.242.58.105',
                'draco': '85.242.58.105',
                'localdocker': '172.17.0.1'}

#URL = BACKEND_LIST[ENDPOINT] + API_VERSION
#URL_WS = BACKEND_LIST[ENDPOINT] + API_VERSION
#JANUS_IP = JANUS_LIST[ENDPOINT]

# Cameras available: PC: default, RealSense: realsense, RaspyCam: raspy, Insta360 Pro: insta360, EH640_30x_zoom: eh640
CAM_TYPE = 'eh640'

INSTA360_IP = "192.168.1.188"

#Set to true to enable stablization
STABILIZE = False
# To enable jetson streaming pipelines set to True
JETSON = True

# JoaoBalao: Enables triple pipeline and disables original heifu pipe
TRIPLE_PIPE = True
REC_PATH = "/home/heifu/heifu_ws/src/heifu/perception/gstreamer/src/recordings/"
PHOTO_PATH = "/home/heifu/heifu_ws/src/heifu/perception/gstreamer/src/photos/"

SAVE_PIPELINES = True
GRAPHVIZ_PATH = "/home/heifu/heifu_ws/src/heifu/perception/gstreamer/src/pipe_diagrams"
