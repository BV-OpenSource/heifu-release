#include "loiter_controller/loiter_controller.hpp"
using namespace LC;
using namespace hvtol_msgs;


Loiter_Controller::Loiter_Controller():nh("~"){
    std::string ns = ros::this_node::getNamespace();

    //	Parameters
    nh.param<double>("paramLoiterError",		paramLoiterError,	4.0);
    nh.param<double>("paramNodeRate",           paramNodeRate,      100.0);
    nh.param<int>("paramNumberLaps",            paramNumberLaps,	3);
    // Subscribers
    subPosition = nh.subscribe(ns+"/mavros/local_position/pose", 1, &Loiter_Controller::cbPosition, this);

    // Publishers
    pubLocalPos = nh.advertise<mavros_msgs::PositionTarget>(ns+"/mavros/setpoint_raw/local", 10);
}

void Loiter_Controller::cbPosition(const geometry_msgs::PoseStamped &msg){
    mavros_msgs::PositionTarget goToPoint;
    bool stopPublish=false;
    ros::Rate r(paramNodeRate);

    if(loiterSetpointRec){

        goToPoint.type_mask = 12288;
        goToPoint.position.x=goalPoint.x;
        goToPoint.position.y=goalPoint.y;
        goToPoint.position.z=goalPoint.z;

        if( checkLoiterStart(msg.pose.position.x,msg.pose.position.y,msg.pose.position.z) && (x_check == 0) && (y_check == 0)){
            x_check = msg.pose.position.x;
            y_check = msg.pose.position.y;
            loiter_point_check=0;
            canIncrement=false;
        }

        if((x_check!=0) && (y_check!=0)){
            if(((x_check-paramLoiterError)<msg.pose.position.x) && (msg.pose.position.x<(x_check+paramLoiterError)) &&
                    ((y_check-paramLoiterError)<msg.pose.position.y) && (msg.pose.position.y<(y_check+paramLoiterError)) &&
                    ((goalPoint.z-paramLoiterError)<msg.pose.position.z) && (msg.pose.position.z<(goalPoint.z+paramLoiterError))){

                if(canIncrement){
                    loiter_point_check++;
                    canIncrement=false;
                    printf(YELLOW"Finished turns= %d\n",loiter_point_check);
                }

                if(loiter_point_check==paramNumberLaps){
                    loiter_point_check=0;
                    stopPublish=true;
                    x_check=0;
                    y_check=0;
                }
            }
            else{
                float distance2=getDistance2D(x_check,y_check,msg.pose.position.x,msg.pose.position.y);
                if(distance2>paramLoiterError){
                    canIncrement=true;
                }
            }
        }
        if(stopPublish){
            loiterSetpointRec=false;
        }
        else{
            pubLocalPos.publish(goToPoint);
        }
    }

}


bool Loiter_Controller::checkLoiterStart(float currX, float currY, float currZ){

    double distance=getDistance3D(currX,currY,currZ,goalPoint.x,goalPoint.y,goalPoint.z);

    if((distance>(goalPoint.radius-1)) && (distance<(goalPoint.radius+1))){
        counter++;
        if(counter==50){
            counter=0;
            return true;
        }
    }
    else {
        counter=0;
    }
    return false;
}

float Loiter_Controller::getDistance2D(double x1, double y1, double x2, double y2){
    return sqrt(pow(x1-x2, 2) + pow(y1-y2, 2) );
}

float Loiter_Controller::getDistance3D(double x1, double y1, double z1, double x2, double y2, double z2){
    return sqrt(pow(x1-x2, 2) + pow(y1-y2, 2) + pow(z1-z2, 2));
}

