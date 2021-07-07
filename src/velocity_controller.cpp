#include "velocity_controller/velocity_controller.hpp"

using namespace VC;
using namespace mavros_msgs;
using namespace uav_msgs;

Velocity_Controller::Velocity_Controller():nh("~"){
    std::string ns = ros::this_node::getNamespace();

    //	Parameters
    nh.param<double>("paramPosError",           paramPosError, 2.0);
    nh.param<double>("paramYawError",           paramYawError, 10.0);
    nh.param<double>("paramMaxVelocity",        paramMaxVelocity, 4.0);
    nh.param<double>("paramSmoothFactor",       paramSmoothFactor, 2.0);
    nh.param<double>("paramReachDistance",      paramReachDistance, 4.0);
    nh.param<double>("paramForceLinearSpeed",   paramForceLinearSpeed, 0.04);
    nh.param<double>("paramMaxAngularVelocity", paramMaxAngularVelocity, 45.0);

    paramYawError *= deg2rad;
    maxYawError = paramYawError;
    paramMaxAngularVelocity *= deg2rad;
    maxAngularVelocity = paramMaxAngularVelocity;

    // Subscribers
    subPosition = nh.subscribe(ns + "/mavros/local_position/pose", 1, &Velocity_Controller::cbPosition, this);

    // Publishers
    pubVelocity = nh.advertise<mavros_msgs::PositionTarget>(ns + "/mavros/setpoint_raw/local", 10);
    pubControlState = nh.advertise<diagnostic_msgs::KeyValue>(ns + "/velocityController/state", 10);
}

void Velocity_Controller::cbPosition(const geometry_msgs::PoseStamped &msg){

    mavros_msgs::PositionTarget goToPoint;
    goToPoint.header.stamp = ros::Time::now();

    localPosition.pose = msg.pose;

    bool stopPublish = false;

    if(velSetpointRec){
        goalPoint.radius = (goalPoint.radius > 0.5f) ? goalPoint.radius : paramPosError;

        maxAngularVelocity = (goalPoint.maxAngularVel > 0.1f) ? goalPoint.maxAngularVel : paramMaxAngularVelocity;

        maxYawError = (goalPoint.yawError > 0.1f) ? goalPoint.yawError : paramYawError;

        maxLinearVelocity = (goalPoint.maxLinearVel >= 0.5f) ? goalPoint.maxLinearVel : paramMaxVelocity;

        calculateVelocitySetpoints();
        goToPoint.velocity = velocity.twist.linear;

        goToPoint.coordinate_frame = goalPoint.frame;

        switch(goalPoint.type){
            case UAVWaypoint::TYPE_VELOCITY: //Velocity with yaw
                goToPoint.type_mask = (~(PositionTarget::IGNORE_VX |
                                         PositionTarget::IGNORE_VY |
                                         PositionTarget::IGNORE_VZ |
                                         PositionTarget::IGNORE_YAW_RATE)) & 0x0FFF;

                //Calculate yaw rate to setpoint
                calculateYawRate(goalPoint.yaw);
                goToPoint.yaw_rate = static_cast<float>(velocity.twist.angular.z);

                stopPublish = isDistanceWithinRadius(errorNorm, goalPoint.radius) & isAngleWithinRange(goalPoint.yaw, yaw);

                break;

            case UAVWaypoint::TYPE_VELOCITY_ONLY: //Only Velocity
                goToPoint.type_mask = (~(PositionTarget::IGNORE_VX |
                                         PositionTarget::IGNORE_VY |
                                         PositionTarget::IGNORE_VZ)) & 0x0FFF;

                stopPublish = isDistanceWithinRadius(errorNorm, goalPoint.radius);

                break;

            case UAVWaypoint::TYPE_VELOCITY_YAW: //Set yaw first, then goto Position
                if(stage == 0){
                    goToPoint.type_mask = (~(PositionTarget::IGNORE_VX |
                                             PositionTarget::IGNORE_VY |
                                             PositionTarget::IGNORE_VZ |
                                             PositionTarget::IGNORE_YAW_RATE)) & 0x0FFF;

                    //Force no movement
                    goToPoint.velocity.x = 0.0;
                    goToPoint.velocity.y = 0.0;
                    goToPoint.velocity.z = 0.0;

                    //Calculate yaw rate to setpoint
                    calculateYawRate(goalPoint.yaw);
                    goToPoint.yaw_rate = static_cast<float>(velocity.twist.angular.z);

                    stage = isAngleWithinRange(goalPoint.yaw, yaw);
                    ROS_WARN_STREAM("AngleRange: " << isAngleWithinRange(goalPoint.yaw, yaw) <<"Stage: " << stage);

                }
                if(stage == 1){
                    goToPoint.type_mask = (~(PositionTarget::IGNORE_VX |
                                             PositionTarget::IGNORE_VY |
                                             PositionTarget::IGNORE_VZ |
                                             PositionTarget::IGNORE_YAW_RATE)) & 0x0FFF;

                    //Calculate yaw rate to setpoint
                    calculateYawRate(goalPoint.yaw);
                    goToPoint.yaw_rate = static_cast<float>(velocity.twist.angular.z);

                    stopPublish = isDistanceWithinRadius(errorNorm, goalPoint.radius) & isAngleWithinRange(goalPoint.yaw, yaw);

                }
                break;

            case UAVWaypoint::TYPE_VELOCITY_YAW_WP: //Set heading to WP, goto Position
                goToPoint.type_mask = (~(PositionTarget::IGNORE_VX |
                                         PositionTarget::IGNORE_VY |
                                         PositionTarget::IGNORE_VZ |
                                         PositionTarget::IGNORE_YAW_RATE)) & 0x0FFF;

                //Calculate yaw to waypoint
                WPYaw = calculateWaypointHeading(msg, goalPoint);

                //Calculate yaw rate to setpoint
                calculateYawRate(WPYaw);
                goToPoint.yaw_rate = static_cast<float>(velocity.twist.angular.z);

                stopPublish = isDistanceWithinRadius(errorNorm, goalPoint.radius);

                break;

            case UAVWaypoint::TYPE_VELOCITY_YAW_FIRST_WP: //Set heading to WP, then goto Position
                if(stage == 0){
                    goToPoint.type_mask = (~(PositionTarget::IGNORE_VX |
                                             PositionTarget::IGNORE_VY |
                                             PositionTarget::IGNORE_VZ |
                                             PositionTarget::IGNORE_YAW_RATE)) & 0x0FFF;

                    //Force no movement
                    goToPoint.velocity.x = 0.0;
                    goToPoint.velocity.y = 0.0;
                    goToPoint.velocity.z = 0.0;

                    //Calculate yaw to waypoint
                    WPYaw = calculateWaypointHeading(msg, goalPoint);

                    //Calculate yaw rate to setpoint
                    calculateYawRate(WPYaw);
                    goToPoint.yaw_rate = static_cast<float>(velocity.twist.angular.z);

                    stage = isAngleWithinRange(WPYaw, yaw);

                }
                if(stage == 1){
                    goToPoint.type_mask = (~(PositionTarget::IGNORE_VX |
                                             PositionTarget::IGNORE_VY |
                                             PositionTarget::IGNORE_VZ |
                                             PositionTarget::IGNORE_YAW_RATE)) & 0x0FFF;

                    //Calculate yaw rate to setpoint
                    calculateYawRate(WPYaw);
                    goToPoint.yaw_rate = static_cast<float>(velocity.twist.angular.z);

                    stopPublish = isDistanceWithinRadius(errorNorm, goalPoint.radius);

                }
                break;

            default :
                ROS_ERROR("Type not recognized, requesting next waypoint!");
                stopPublish = true;
                return;
        }

        if(stopPublish){
            velSetpointRec = false;
        }
        else{
            if(pubControlState.getNumSubscribers() > 0){
                controlState.key = "errorNorm";
                controlState.value = std::to_string(errorNorm);
                pubControlState.publish(controlState);

                controlState.key = "goalPoint.radius";
                controlState.value = std::to_string(goalPoint.radius);
                pubControlState.publish(controlState);

                controlState.key = "yaw";
                controlState.value = std::to_string(yaw);
                pubControlState.publish(controlState);

                controlState.key = "goalPoint.yaw";
                controlState.value = std::to_string(goalPoint.yaw);
                pubControlState.publish(controlState);

                controlState.key = "maxAngularVelocity";
                controlState.value = std::to_string(maxAngularVelocity);
                pubControlState.publish(controlState);

                controlState.key = "maxLinearVelocity";
                controlState.value = std::to_string(maxLinearVelocity);
                pubControlState.publish(controlState);

                controlState.key = "---";
                controlState.value = "";
                pubControlState.publish(controlState);
            }
            pubVelocity.publish(goToPoint);
        }
    }
}

void Velocity_Controller::calculateVelocitySetpoints(){

    geometry_msgs::Point errorPosition;

    errorPosition.x = goalPoint.x - localPosition.pose.position.x;
    errorPosition.y = goalPoint.y - localPosition.pose.position.y;
    errorPosition.z = goalPoint.z - localPosition.pose.position.z;

    errorNorm = sqrt(errorPosition.x*errorPosition.x + errorPosition.y*errorPosition.y + errorPosition.z*errorPosition.z);

    double scale = maxLinearVelocity / errorNorm;

    if(errorNorm > 0.0){
        velocity.twist.linear.x = errorPosition.x*scale;
        velocity.twist.linear.y = errorPosition.y*scale;
        velocity.twist.linear.z = errorPosition.z*scale;
    }
    if(errorNorm < paramReachDistance){
        velocity.twist.linear.x *= pow(errorNorm/paramReachDistance, paramSmoothFactor);
        velocity.twist.linear.y *= pow(errorNorm/paramReachDistance, paramSmoothFactor);
        velocity.twist.linear.z *= pow(errorNorm/paramReachDistance, paramSmoothFactor);

        double modvel = sqrt(velocity.twist.linear.x*velocity.twist.linear.x + velocity.twist.linear.y*velocity.twist.linear.y + velocity.twist.linear.z*velocity.twist.linear.z);

        velocity.twist.linear.x += (velocity.twist.linear.x / modvel) * paramForceLinearSpeed;
        velocity.twist.linear.y += (velocity.twist.linear.y / modvel) * paramForceLinearSpeed;
        velocity.twist.linear.z += (velocity.twist.linear.z / modvel) * paramForceLinearSpeed;
        velocity.twist.angular.z += velocity.twist.angular.z * 0.1;
    }
}

void Velocity_Controller::calculateYawRate(double desAngle){

    tf::Quaternion quat = tf::Quaternion(localPosition.pose.orientation.x, localPosition.pose.orientation.y, localPosition.pose.orientation.z, localPosition.pose.orientation.w);
    yaw = tf::getYaw(quat);

    double diffAngle = calculateDiffAngle(desAngle, yaw);

    velocity.twist.angular.z = diffAngle * maxAngularVelocity / M_PI;   //M_PI assuming max speed for max rotation (M_PI)
}

bool Velocity_Controller::isDistanceWithinRadius(double distance, double radius){
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

bool Velocity_Controller::isAngleWithinRange(double desAngle,double actAngle){

    double diffAngle = calculateDiffAngle(desAngle, actAngle);

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

double Velocity_Controller::calculateWaypointHeading(geometry_msgs::PoseStamped currPos, uav_msgs::UAVWaypoint goalPos){
    double heading = atan2(goalPos.y - currPos.pose.position.y, goalPos.x - currPos.pose.position.x);
    return heading;
}

void Velocity_Controller::changeMaxSpeed(float maxSpeed){
    this->maxLinearVelocity = static_cast<double>(maxSpeed);
}

double Velocity_Controller::calculateDiffAngle(double desAngle, double actAngle){
    double diffAngle = desAngle - actAngle;
    diffAngle = fmod(diffAngle, 2*M_PI);

    diffAngle = (diffAngle > M_PI) ? diffAngle - 2 * M_PI : (diffAngle < -M_PI) ? diffAngle + 2 * M_PI : diffAngle;

    return diffAngle;
}
