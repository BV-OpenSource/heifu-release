#include "position_controller/position_controller.hpp"

using namespace PC;
using namespace mavros_msgs;
using namespace uav_msgs;

Position_Controller::Position_Controller():nh("~"){
    std::string ns = ros::this_node::getNamespace();

    //	Parameters
    nh.param <double>("paramPosError", paramPosError, 1.0);
    nh.param <double>("paramYawError", paramYawError, 10.0);

    paramYawError *= M_PI/180.0;

    // Subscribers
    subPosition = nh.subscribe(ns + "/mavros/local_position/pose", 1, &Position_Controller::cbPosition, this);

    // Publishers
    pubLocalPos = nh.advertise <mavros_msgs::PositionTarget>(ns + "/mavros/setpoint_raw/local", 10);
    pubControlState = nh.advertise<diagnostic_msgs::KeyValue>(ns + "/positionController/state", 10);
}

void Position_Controller::cbPosition(const geometry_msgs::PoseStamped &msg){

    mavros_msgs::PositionTarget goToPoint;
    bool stopPublish = false;

    tf::Quaternion quat = tf::Quaternion(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);
    double robotYaw = tf::getYaw(quat);

    if(posSetpointRec){
        geometry_msgs::Point errorPosition;
        double errorNorm;

        if(goalPoint.frame == goToPoint.FRAME_LOCAL_NED){
            errorPosition.x = goalPoint.x - msg.pose.position.x;
            errorPosition.y = goalPoint.y - msg.pose.position.y;
            errorPosition.z = goalPoint.z - msg.pose.position.z;

            goToPoint.position.x = goalPoint.x;
            goToPoint.position.y = goalPoint.y;
            goToPoint.position.z = goalPoint.z;
        }else if(goalPoint.frame == goToPoint.FRAME_BODY_NED){
            ROS_INFO_STREAM("GP: "<<goalPoint);
            errorPosition.x = goalPoint.x * cos(robotYaw) - goalPoint.y * sin(robotYaw);
            errorPosition.y = goalPoint.y * cos(robotYaw) + goalPoint.x * sin(robotYaw);
            errorPosition.z = goalPoint.z;

            ROS_INFO_STREAM("EP: "<<errorPosition);
            goToPoint.position.x = errorPosition.x + msg.pose.position.x;
            goToPoint.position.y = errorPosition.y + msg.pose.position.y;
            goToPoint.position.z = errorPosition.z + msg.pose.position.z;

            goalPoint.yaw = goalPoint.yaw + static_cast<float>(robotYaw);

            ROS_INFO_STREAM("GT: "<<goToPoint);

            goalPoint.x = static_cast<float>(goToPoint.position.x);
            goalPoint.y = static_cast<float>(goToPoint.position.y);
            goalPoint.z = static_cast<float>(goToPoint.position.z);

            goalPoint.frame = goToPoint.FRAME_LOCAL_NED;
        }

        goToPoint.coordinate_frame = goalPoint.frame;

        errorNorm = sqrt(errorPosition.x*errorPosition.x + errorPosition.y*errorPosition.y + errorPosition.z*errorPosition.z);

        goalPoint.radius = (goalPoint.radius > 0.5f) ? goalPoint.radius : static_cast<float>(paramPosError);

        maxYawError = (goalPoint.yawError > 0.1f) ? goalPoint.yawError : paramYawError;

        switch(goalPoint.type){
            case UAVWaypoint::TYPE_POSITION: //Position with yaw
                goToPoint.type_mask = (~(PositionTarget::IGNORE_PX |
                                         PositionTarget::IGNORE_PY |
                                         PositionTarget::IGNORE_PZ |
                                         PositionTarget::IGNORE_YAW)) & 0x0FFF;

                goToPoint.yaw = static_cast<float>(goalPoint.yaw);

                stopPublish = isDistanceWithinRadius(errorNorm, goalPoint.radius) & isAngleWithinRange(goalPoint.yaw, robotYaw);

                break;

            case UAVWaypoint::TYPE_POSITION_ONLY: //Only Position
                goToPoint.type_mask = (~(PositionTarget::IGNORE_PX |
                                         PositionTarget::IGNORE_PY |
                                         PositionTarget::IGNORE_PZ)) & 0x0FFF;

                stopPublish = isDistanceWithinRadius(errorNorm, goalPoint.radius);

                break;

            case UAVWaypoint::TYPE_POSITION_YAW: //Set yaw first, then goto Position
                if(stage == 0){
                    goToPoint.type_mask = (~(PositionTarget::IGNORE_VX |
                                             PositionTarget::IGNORE_VY |
                                             PositionTarget::IGNORE_VZ |
                                             PositionTarget::IGNORE_YAW))& 0x0FFF;

                    goToPoint.velocity.x = 0.0;
                    goToPoint.velocity.y = 0.0;
                    goToPoint.velocity.z = 0.0;
                    goToPoint.yaw = static_cast<float>(goalPoint.yaw);

                    stage = isAngleWithinRange(goalPoint.yaw, robotYaw);
                }

                if(stage == 1){
                    goToPoint.type_mask = (~(PositionTarget::IGNORE_PX |
                                             PositionTarget::IGNORE_PY |
                                             PositionTarget::IGNORE_PZ |
                                             PositionTarget::IGNORE_YAW))& 0x0FFF;

                    goToPoint.yaw = static_cast<float>(goalPoint.yaw);

                    stopPublish = isDistanceWithinRadius(errorNorm, goalPoint.radius) & isAngleWithinRange(goalPoint.yaw, robotYaw);
                }

                break;

            case UAVWaypoint::TYPE_POSITION_YAW_WP: //Set heading to WP, goto Position
                goToPoint.type_mask = (~(PositionTarget::IGNORE_PX |
                                         PositionTarget::IGNORE_PY |
                                         PositionTarget::IGNORE_PZ |
                                         PositionTarget::IGNORE_YAW))& 0x0FFF;

                goToPoint.yaw = calculateWaypointHeading(msg, goalPoint);

                stopPublish = isDistanceWithinRadius(errorNorm, goalPoint.radius);

                break;

            case UAVWaypoint::TYPE_YAW_ONLY:
                goToPoint.type_mask = (~(PositionTarget::IGNORE_VX |
                                         PositionTarget::IGNORE_VY |
                                         PositionTarget::IGNORE_VZ |
                                         PositionTarget::IGNORE_YAW))& 0x0FFF;

                goToPoint.velocity.x = 0.0;
                goToPoint.velocity.y = 0.0;
                goToPoint.velocity.z = 0.0;
                goToPoint.yaw = static_cast<float>(goalPoint.yaw);

                stage = isAngleWithinRange(goalPoint.yaw, robotYaw);

                break;

            case UAVWaypoint::TYPE_POSITION_YAW_FIRST_WP: //Set heading to WP, then goto Position
                if(stage == 0){
                    goToPoint.type_mask = (~(PositionTarget::IGNORE_VX |
                                             PositionTarget::IGNORE_VY |
                                             PositionTarget::IGNORE_VZ |
                                             PositionTarget::IGNORE_YAW)) & 0x0FFF;

                    //Force no movement
                    goToPoint.velocity.x = 0.0;
                    goToPoint.velocity.y = 0.0;
                    goToPoint.velocity.z = 0.0;

                    //Calculate yaw to waypoint
                    WPYaw = static_cast<float>(calculateWaypointHeading(msg, goalPoint));
                    goToPoint.yaw = static_cast<float>(WPYaw);
                    ROS_WARN_STREAM_THROTTLE(1,"Yaw: " << WPYaw);

                    stage = isAngleWithinRange(WPYaw, robotYaw);
                    ROS_WARN_STREAM_THROTTLE(1,"Stage: " << stage);
                }
                if(stage == 1){
                    goToPoint.type_mask = (~(PositionTarget::IGNORE_PX |
                                             PositionTarget::IGNORE_PY |
                                             PositionTarget::IGNORE_PZ |
                                             PositionTarget::IGNORE_YAW)) & 0x0FFF;

                    //Maintain yaw to waypoint
                    goToPoint.yaw = static_cast<float>(WPYaw);

                    stopPublish = isDistanceWithinRadius(errorNorm, goalPoint.radius);

                }
                break;


            default :
                ROS_ERROR("Type not recognized, requesting next waypoint!");
                stopPublish = true;
                return;
        }

        if(stopPublish){
            posSetpointRec = false;
        }
        else{
            if(pubControlState.getNumSubscribers() > 0){
                controlState.key = "errorNorm";
                controlState.value = std::to_string(errorNorm);
                pubControlState.publish(controlState);

                controlState.key = "goalPoint.radius";
                controlState.value = std::to_string(goalPoint.radius);
                pubControlState.publish(controlState);

                controlState.key = "robotYaw";
                controlState.value = std::to_string(robotYaw);
                pubControlState.publish(controlState);

                controlState.key = "goToPoint.yaw";
                controlState.value = std::to_string(goToPoint.yaw);
                pubControlState.publish(controlState);

                controlState.key = "maxYawError";
                controlState.value = std::to_string(maxYawError);
                pubControlState.publish(controlState);

                controlState.key = "---";
                controlState.value = "";
                pubControlState.publish(controlState);
            }
            pubLocalPos.publish(goToPoint);
        }

    }
}

bool Position_Controller::isDistanceWithinRadius(double distance, double radius){
    if(distance < radius){
        distanceInc++;
        if(distanceInc > 10){
            distanceInc--;
            return true;
        }
    }
    else {
        distanceInc = 0;
        return false;
    }
    return false;
}

bool Position_Controller::isAngleWithinRange(double desAngle, double actAngle){
    double diffAngle = desAngle - actAngle;
    diffAngle = fmod(diffAngle, 2*M_PI);

    diffAngle = (diffAngle > M_PI) ? diffAngle - 2 * M_PI : (diffAngle < -M_PI) ? diffAngle + 2 * M_PI : diffAngle;

    if( fabs(diffAngle) < maxYawError){
        angleInc++;
        if(angleInc > 10){
            angleInc--; // Avoid overflow
            return true;
        }
    }
    else {
        angleInc = 0;
        return false;
    }
    return false;
}

float Position_Controller::calculateWaypointHeading(geometry_msgs::PoseStamped currPos, uav_msgs::UAVWaypoint goalPos){
    float heading = atan2(goalPos.y - currPos.pose.position.y, goalPos.x - currPos.pose.position.x);
    return heading;
}

