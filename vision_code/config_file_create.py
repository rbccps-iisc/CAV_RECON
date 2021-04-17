from configparser import ConfigParser

#Get the configparser object
config_object = ConfigParser()

#Assume we need 2 sections in the config file, let's call them USERINFO and SERVERCONFIG
config_object["Image"] = {
    "height": "480",
    "width": "640",
    "Webcam": "0", #number of the webcam 
    "Recorded_video_path": "./logi2.webm" #if recorded video is to be used
}

config_object["Object_detection"] = {
    "config_file_path": "./darknet_folders/yolo-obj.cfg",
    "weights_file_path": "./darknet_folders/yolo-obj_best.weights",
    "meta_file_path": "./darknet_folders/obj.data",
    "threshold":"0.5" 
}

config_object["Segmentation"] = {
    "Homography_matrix": "[[-16.191704376381097,38.925044312248865,685.879428629976928],[-9.574977774177905,-0.168457950406949,-6069.195706433699343],[-0.057077859951752,0.002135952913283,1.000000000000000]]",
    "3d_ROI": "[[400,500,1],[400,-500,1],[1200,-500,1],[1200,500,1]]",
    "path_for_segment_folders": "/home/rbccps/deployment/version1/segment_folders/ShelfNet18_realtime/",
    "weights_path": "./segment_folders/model_final_final_iisc_idd_16kweights.pth"
}

config_object["Results_path"] = {
    "Path": "./res/"
}

config_object["models_needed"] = {
    "one": "",#obj/seg
    "both": "true"
}




#Write the above sections to config.ini file
with open('config.ini', 'w') as conf:
    config_object.write(conf)