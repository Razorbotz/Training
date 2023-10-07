#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <messages/msg/button_state.hpp>
#include <messages/msg/hat_state.hpp>
#include <messages/msg/axis_state.hpp>
#include <messages/msg/key_state.hpp>
#include <messages/msg/zed_position.hpp>
#include <messages/msg/autonomy_out.hpp>

#include "logic/Automation1.hpp"
#include "logic/AutomationTypes.hpp"


/** @file
 * @brief Node handling logic for robot
 * 
 * This node receives information published by the communication node,
 * wraps the information into topics, then publishes the topics.  
 * The topics that the node subscribes to are as follows:
 * \li \b joystick_axis
 * \li \b joystick_button
 * \li \b joystick_hat
 * \li \b key
 * \li \b zed_position
 * 
 * To read more about the communication node
 * \see communication_node.cpp
 * 
 * The topics that are being published are as follows:
 * \li \b drive_left_speed
 * \li \b drive_right_speed
 * \li \b dump_bin_speed
 * \li \b shoulder_speed
 * \li \b excavationArm
 * \li \b excavationDrum
 * 
 * To read more about the nodes that subscribe to this one
 * \see talon_node.cpp
 * \see excavation_node.py
 * \see falcon_node.cpp
 * 
 * */

rclcpp::Node::SharedPtr nodeHandle;

float joystick1Roll=0;
float joystick1Pitch=0;
float joystick1Yaw=0;
float joystick1Throttle=0;

float maxSpeed=0.4;
float maxNeoSpeed = 0.1;

std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > talonSpeedPublisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > falconSpeedPublisher;
std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > neoSpeedPublisher;


/** @brief Function to initialize the motors to zero
 * 
 * This function is called on start of the node and
 * sends a message with a zero message to all motors
 * to ensure that the motors are not set to an old 
 * value.
 * @return void
 * */
void initSetSpeed(){
    std_msgs::msg::Float32 speed;
    speed.data = 0.0;
    
    talonSpeedPublisher->publish(speed);
    falconSpeedPublisher->publish(speed);
    neoSpeedPublisher->publish(speed);
}

/** @brief Function to update speed of the wheels
 * 
 * This function is called by the joystickAxisCallback
 * to update the speed to the wheels.  It uses the data 
 * published to compute the speed of the wheels based on
 * the joystick information and limited by the maxSpeed.
 * @return void
 * */
void updateSpeed(){
    
    std_msgs::msg::Float32 speed;
    speed.data  = (joystick1Pitch + joystick1Roll);
    speed.data  = speed.data * maxSpeed;
    falconSpeedPublisher->publish(speed);
}

/** @brief Function to stop drive train motors
 * 
 * This function is called when toggle excavation 
 * button is pressed and switches to the excavation
 * state.  It publishes a speed of 0.0 for both the
 * right and left wheels.
 * @return void
 * */
void stopSpeed(){
    std_msgs::msg::Float32 speed;
    speed.data = 0.0;
    talonSpeedPublisher->publish(speed);
    falconSpeedPublisher->publish(speed);
    neoSpeedPublisher->publish(speed);
}

/**
 * @brief  Function to transform the joystick input
 * 
 * This function is called to transform the joystick input 
 * into a slightly modified version.  Because the joystick
 * doesn't go to exactly zero, there will always be data
 * published to the robot, causing it to drift slightly.  
 * This function zeroes the input in the range
 * [-deadZone, deadZone] to prevent this.
 * 
 * @param info - Float value of the joystick axis
 * @param deadZone - Float value of the deadzone of the joystick
 * @return float 
 */
float transformJoystickInfo(float info, float deadZone){
    float transformed = (fabs(info) < deadZone) ? 0.0 : info;
    transformed = (transformed > 0) ? transformed - deadZone : transformed;
    transformed = (transformed < 0) ? transformed + deadZone : transformed;
    return transformed;
}


/** @brief Callback function for joystick axis topic
 * 
 * This function is called when the node receives the
 * topic joystick_axis, then converts the joystick
 * input using a series of linear transformations 
 * before calling updateSpeed() or updateExcavation()
 * depending on the current state.
 * @param axisState \see AxisState.msg
 * @return void
 * */
void joystickAxisCallback(const messages::msg::AxisState::SharedPtr axisState){
    //RCLCPP_INFO(nodeHandle->get_logger(),"Axis %d %d %f", axisState->joystick, axisState->axis, axisState->state);
    //RCLCPP_INFO(nodeHandle->get_logger(),"Axis %d %f %f %f %f", axisState->joystick, axisState->state0, axisState->state1, axisState->state2, axisState->state3);
    float deadZone = 0.1;
    if(axisState->axis==0){
        joystick1Roll = transformJoystickInfo(-axisState->state, deadZone);
        updateSpeed();
    }
    else if(axisState->axis==1){
        joystick1Pitch = transformJoystickInfo(axisState->state, deadZone);
        updateSpeed();
    }
    else if(axisState->axis==2){
        joystick1Yaw = transformJoystickInfo(axisState->state, deadZone);
        std_msgs::msg::Float32 speed;
        speed.data  = joystick1Yaw;
        speed.data  = speed.data * maxSpeed;
        talonSpeedPublisher->publish(speed);
    }
    else if(axisState->axis==3){
        joystick1Throttle = axisState->state/2 + 0.5;
        joystick1Throttle = transformJoystickInfo(joystick1Throttle, deadZone * maxNeoSpeed);
        std_msgs::msg::Float32 speed;
        speed.data  = joystick1Throttle;
        speed.data  = speed.data * maxSpeed;
        neoSpeedPublisher->publish(speed);
    }
}

/** @brief Callback function for joystick buttons
 * 
 * This function is called when the node receives a
 * topic with the name joystick_button.  Button 2 
 * toggles the drive and excavation states while
 * button 3 inverts the direction of the drum.  
 * Buttons 6 and 7 control the locking servo.
 * @param buttonState \see ButtonState.msg
 * @return void
 * */
void joystickButtonCallback(const messages::msg::ButtonState::SharedPtr buttonState){
    std::cout << "Button " << buttonState->joystick << " " << buttonState->button << " " << buttonState->state << std::endl;
    std_msgs::msg::Float32 speed;
    std_msgs::msg::Bool state;

    switch (buttonState->button) {
        case 0:
            break;
        case 1:
            break;
        case 2:
            break;
        case 3:
            break;
        case 4:
            break;
        case 5:
            break;
        case 6:
            break;
        case 7:
            break;
        case 8:
            break;
        case 9:
            break;
        case 10:
            break;
        case 11:
            break;
    }
}

/** @brief Callback function for joystick hat
 * 
 * This function is called when the node receives a
 * topic with the name joystick_hat.  It publishes
 * the dump bin speed based on the hat.
 * @param hatState \see HatState.msg
 * @return void
 * */
void joystickHatCallback(const messages::msg::HatState::SharedPtr hatState){
    std::cout << "Hat " << (int)hatState->joystick << " " << (int)hatState->hat << " " << (int)hatState->state << std::endl;
}

/** @brief Function to update max speed multiplier
 * 
 * This function is called when the keyCallback
 * function receives either a '+' or '-' keypress
 * event.  A delta speed value is then passed to
 * this function to change the value of the max
 * speed multiplier dynamically, instead of the 
 * user recompiling the code and reexecuting it.
 * @param deltaSpeed - Amount to change max speed
 * @return void
 * */
void updateMaxSpeed(float deltaSpeed){
    maxSpeed += deltaSpeed;
    if(maxSpeed <= 0){
        maxSpeed = 0.1;
    }
    if(maxSpeed > 1){
        maxSpeed = 1;
    }
    RCLCPP_INFO(nodeHandle->get_logger(), "maxSpeed: %f", maxSpeed);
}


void updateMaxNeoSpeed(float deltaSpeed){
    maxNeoSpeed += deltaSpeed;
    if(maxNeoSpeed <= 0){
        maxNeoSpeed = 0.1;
    }
    if(maxNeoSpeed > 1){
        maxNeoSpeed = 1;
    }
    RCLCPP_INFO(nodeHandle->get_logger(), "maxNeoSpeed: %f", maxNeoSpeed);
}

/** @brief Callback function for the keys
 * 
 * This function is called when the node receives a
 * topic with the name key_state.  It currently prints
 * the key pressed to the screen and inverts  
 * @arg automationGo if the key is 's'.  This function
 * also increases the max speed multiplier when the '+'
 * key is pressed and decreases it when the '-' key is 
 * pressed.
 * @param keyState \see KeyState.msg
 * @return void
 * */
void keyCallback(const messages::msg::KeyState::SharedPtr keyState){
    std::cout << "Key " << keyState->key << " " << keyState->state << std::endl;

    if(keyState->key==115 && keyState->state==1){
    }
    if(keyState->key==120 && keyState->state==1){
    }
    if(keyState->key==43 && keyState->state==1){
        updateMaxSpeed(0.1);
    }
    if(keyState->key==45 && keyState->state==1){
        updateMaxSpeed(-0.1);
    }
    if(keyState->key==104 && keyState->state==1){
        updateMaxNeoSpeed(0.1);
    }
    if(keyState->key==108 && keyState->state==1){
        updateMaxNeoSpeed(-0.1);
    }
}

int main(int argc, char **argv){
    rclcpp::init(argc,argv);
    nodeHandle = rclcpp::Node::make_shared("logic");
    automation->setNode(nodeHandle);

    auto joystickAxisSubscriber= nodeHandle->create_subscription<messages::msg::AxisState>("joystick_axis",1,joystickAxisCallback);
    auto joystickButtonSubscriber= nodeHandle->create_subscription<messages::msg::ButtonState>("joystick_button",1,joystickButtonCallback);
    auto joystickHatSubscriber= nodeHandle->create_subscription<messages::msg::HatState>("joystick_hat",1,joystickHatCallback);
    auto keySubscriber= nodeHandle->create_subscription<messages::msg::KeyState>("key",1,keyCallback);

    talonSpeedPublisher= nodeHandle->create_publisher<std_msgs::msg::Float32>("talon_speed",1);
    falconSpeedPublisher= nodeHandle->create_publisher<std_msgs::msg::Float32>("falcon_speed",1);
    neoSpeedSpublisher= nodeHandle->create_publisher<std_msgs::msg::Float32>("neo_speed",1);

    initSetSpeed();

    rclcpp::Rate rate(30);
    while(rclcpp::ok()){
        rclcpp::spin_some(nodeHandle);
        rate.sleep();
    }
}
