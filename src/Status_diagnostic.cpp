#include "status_diagnostic/Status_diagnostic.hpp"

using namespace SD;

Status_diagnostic::Status_diagnostic():n("~") {
    std::string ns = ros::this_node::getNamespace();

    // Parameters
    n.param<double>     ("paramNodeRate",           paramNodeRate,           5.0);

    n.param<std::string>("paramSubTopicDiagnostic", paramSubTopicDiagnostic, "/diagnostics");
    n.param<std::string>("paramPubTopicGPSFixType", paramPubTopicGPSFixType, "/g/f/m");
    n.param<std::string>("paramPubTopicVehicleType", paramPubTopicVehicleType, "/vehicle/type");
    n.param<std::string>("paramPubTopicAutoPilotType", paramPubTopicAutoPilotType, "/vehicle/autopilotType");

    // Subscribers
    subDiagnostic   = n.subscribe(paramSubTopicDiagnostic, 1, &Status_diagnostic::cbDiagnostics, this);
    
    // Publishers
    pubGpsFixState      = n.advertise <std_msgs::Int8> (ns + paramPubTopicGPSFixType, 1);
    pubVehicleType      = n.advertise <std_msgs::Int8> (ns + paramPubTopicVehicleType, 1);
    pubAutoPilotType    = n.advertise <std_msgs::Int8> (ns + paramPubTopicAutoPilotType, 1);

    // Variables
    msgString  = {"3D fix", "Normal"};
    keyString  = {"Fix type", "Vehicle type", "Autopilot type"};
    nameString = {"mavros: GPS", "mavros: Heartbeat"};

    if (ns != "/"){
        for(std::string& name:nameString){
            name = ns.substr(1) + "/" + name; // Ignore first char of ns
        }
    }

    gpsFixTypeName = std::vector<std::string>{ "No GPS",  "No FIX",     "2D FIX",
            "3D FIX",  "DGPS FIX",   "RTK FLOAT",
            "RTK FIX", "Static FIX", "PPP"};

    // Node Ready
    ROS_INFO_STREAM("Status Diagnostic Node Ready");
}


void Status_diagnostic::run(){
    
    ros::Rate go(paramNodeRate);

    while (ros::ok()) {
        ros::spinOnce();
        go.sleep();
    }
}

void Status_diagnostic::cbDiagnostics(const diagnostic_msgs::DiagnosticArrayConstPtr msg){

    bool foundGPSFix = false;
    bool foundDroneInfo = false;

    for(size_t i = 0; i<msg->status.size() && !(foundGPSFix && foundDroneInfo); i++) {

        size_t nameIndex = static_cast<size_t>( std::find(nameString.begin(), nameString.end(), msg->status[i].name) - nameString.begin() );
        size_t messageIndex = static_cast<size_t>( std::find(msgString.begin(), msgString.end(), msg->status[i].message) - msgString.begin() );

        if( nameIndex != nameString.size() && messageIndex != msgString.size()){
            for(diagnostic_msgs::KeyValue keyValue:msg->status[i].values){
                size_t keyIndex = static_cast<size_t>( std::find(keyString.begin(), keyString.end(), keyValue.key) - keyString.begin() );
                if( keyIndex != keyString.size() ){
                    // GPS INFO
                    if( nameIndex == 0){

                        pubIntData.data = static_cast<int8_t>(stoi(keyValue.value));
                        ROS_INFO_THROTTLE(10, "GPS Fix Type: %s", gpsFixTypeName[static_cast<uint8_t>(pubIntData.data)].c_str());

                        if (pubGpsFixState.getNumSubscribers() > 0){
                            pubGpsFixState.publish(pubIntData);
                        }
                        foundGPSFix = true;
                        break;
                    }else if( nameIndex == 1 ){
                        // Vehicle INFO
                        pubIntData.data = -1;
                        if( keyIndex == 1){
                            // Vehicle Type
                            if(keyValue.value == "Hexarotor"){
                                pubIntData.data = 13;
                            }else if(keyValue.value == "VTOL reserved 2"){
                                pubIntData.data = 22;
                            }
                            if (pubVehicleType.getNumSubscribers() > 0){
                                pubVehicleType.publish(pubIntData);
                            }
                        }else if( keyIndex == 2 ){
                            // AutoPilot Type
                            if(keyValue.value == "ArduPilot"){
                                pubIntData.data = 3;
                            }else if(keyValue.value == "PX4 Autopilot"){
                                pubIntData.data = 12;
                            }
                            if (pubAutoPilotType.getNumSubscribers() > 0){
                                pubAutoPilotType.publish(pubIntData);
                            }
                            foundDroneInfo = true;
                            break;
                        }
                    }
                }
            }
        }
    }

    //    if (!foundGPSFix) {
    //        ROS_INFO_THROTTLE(10, "GPS Fix Type: No INFO");
    //    }
}
