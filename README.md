# ROS2 Api
## Overview
This project provides a ROS2 Humble-based API for converting, serializing, and communicating various ROS2 messages using different communication protocols. The project is structured into multiple packages, each responsible for specific functionalities such as message conversion, serialization, communication protocols, and configuration parsing. A matching client SDK in Python can be found [here](https://github.com/Mesnero/ros2_sdk). Right now support is limited to manipulators.
## Packages
1. [config_parser](https://github.com/Mesnero/ros2-api/tree/main/config_parser)
Handles the parsing of the configuration file.
2. [converter](https://github.com/Mesnero/ros2-api/tree/main/converter)
Provides the functionalities for serializing and deserializing ROS2 messages with the use of [nlohmann/json](https://github.com/nlohmann/json) and [msgpack-c](https://github.com/msgpack/msgpack-c/tree/cpp_master).
3. [core](https://github.com/Mesnero/ros2-api/tree/main/core)
Core functionalities for handling state and feedback messages, sending messages, and managing communication protocols.
4. [protocol_base](https://github.com/Mesnero/ros2-api/tree/main/protocol_base)
Defines the base [pluginlib](https://github.com/ros/pluginlib/tree/humble) class for communication protocols.
5. [protocols](https://github.com/Mesnero/ros2-api/tree/main/protocols)
Implements specific communication protocols such as Unix Domain Sockets and Transmission Control Protocol (using ZeroMQ) and exports them as plugins.
6. [publisher](https://github.com/Mesnero/ros2-api/tree/main/publisher)
Provides a wrapper and factories for ROS2 publishers, to make them polymorpic.
7. [ros2_api_msgs](https://github.com/Mesnero/ros2-api/tree/main/ros2_api_msgs)
Adds custom message type for ClientFeedback.
9. [types](https://github.com/Mesnero/ros2-api/tree/main/types)
Defines various types and enums used across the project.

## Building the project
To build the project, follow these steps:
1.  Clone the repository.
2.  Navigate to the root directory of the project.
3. Install [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)
4.  Run the following commands:
`rosdep install --from-paths src --ignore-src -r -y`
`colcon build`
## Running the project
To run the project, you can use the example launch file provided:
`ros2  launch  core  server_launch.py`

## Configuration
The project uses a YAML configuration file to specify various parameters. One example configuration file is located at  [api_config.yaml](https://github.com/Mesnero/ros2-api/blob/main/core/config/api_config.yaml).
### File Structure
The configuration file is structured under a single top-level key: `ros2_api`. Under this key, you define the parameters and settings used by the API. Here’s an annotated breakdown of each section: 
#### `states_topic`:
- Type: String
- Description: Specifies the ROS topic from which the joint states are read.
- Default / Example: `"joint_states"`
- Required: false
- 
#### `publishers`:
- Type: List of Publisher Objects
- Description: At least one publisher is required. Each publisher defines a ROS2 publisher, that publishes messages to the topic.
- Publisher Objects:
	- `publisher`
		- Type: String
		- Descrpiton: The type of publisher. Supported message types:
			- JointTrajectoryController
			- JointGroupEffortController
			- JointGroupPositionController
			- JointGroupVelocityController
			- JoyMessage
		- Required: True
	- `name`
		- Type: String
		- Description: A unique identifier for the publisher
		- Required: True
	- `topic`:
		- Type: String
		- Description: The topic name to which the publisher sends its command. If omitted, it defaults to a value specified by the Message (found in types).
		- Required: False
#### `transport`:
- Type: Object
	- `type`
		- Type: String
		- Description: Name of the protocol as exported in the plugin.xml file.
		- Required: true
	- `params`:
		- Type: Object
		- Description: Custom yaml structure of each protocol. See [protocols](#protocols)
		- Required: Depending on protocol



## Currently supported messages
Since this project is mainly used to directly communicate with [ROS2 Control](https://control.ros.org/humble/index.html) the messages right now span the most frequently used controllers when talking to Manipulators. To add teleoperation support, the [Joy Message](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Joy.html) is also available.

#### [JointTrajectoryController](https://control.ros.org/humble/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html)
Default topic: `<name>/joint_trajectory`

JSON Structure:

    {
		"type": 20;
		"publisher_name": string;
		"payload": {
  			"joint_names": string[];
			"joint_traj_points": 
			[
				{
					"positions": double[];
					"velocities": double[];
					"accelerations": double[];
					"effort": double[];
					"seconds": int;
					"nanoseconds": int;
				}
			];
   		}
	}
#### [JointGroupPositionController](https://control.ros.org/humble/doc/ros2_controllers/position_controllers/doc/userdoc.html)
Default topic: `<name>/commands`

JSON Structure:

    {
		"type": 30;
		"publisher_name": string;
		"payload": {
			"joint_values": double[];
   		}
    }
#### [JointGroupEffortController](https://control.ros.org/humble/doc/ros2_controllers/effort_controllers/doc/userdoc.html)
Default topic: `<name>/commands`

JSON Structure:

    {
		"type": 31;
		"publisher_name": string;
		"payload": {
			"joint_values": double[];
   		}
    }
#### [JointGroupVelocityController](https://control.ros.org/humble/doc/ros2_controllers/velocity_controllers/doc/userdoc.html)
Default topic: `<name>/commands`

JSON Structure:

    {
		"type": 32;
		"publisher_name": string;
		"payload": {
			"joint_values": double[];
   		}
    }
#### [JoyMessage](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Joy.html)
Default topic: `/controller_input`
JSON Structure:

    {
		"type": 50;
		"publisher_name": string;
		"payload": {
			"buttons": int[];
   			"axes": double[];
      		}
    }
#### JointStates
Default topic: `/joints_states`

JSON Structure:

    {
		"type": 2;
		"publisher_name": string;
		"payload": {
				"names": string[];
				"positions": double[];
				"velocity": double[];
				"effort": double[];
    				"nanoseconds": int;
				"seconds": int;
    		}
	}
#### ClientFeedback
Default topic: `/feedback_channel`

JSON Structure:

    {
		"type": 1;
		"publisher_name": string;
		"payload": {
			"feedback_code": int;
			"message": string;
   		}
    }

Most feedback code only make sense when used with the Validator found [here](https://github.com/Mesnero/rosbco).
The Feedback Code currently has the following mapping:

    	PUBLISHER_NOT_FOUND (1): Returned when the publisher_name was not found.
	UNEXPECTED_MSG_STRUCTURE (2):  Returned when the JSON Structure was formatted unexpectedly.
	ROBOT_DISCONNECTED_UNEXPECTEDLY (3): Returned when the robot suddenly disconnected.
	INVALID_CMD (100): Returned if the validator can't use the command.
	VELOCITY_LIMIT (101): Returned if a joint exceeded the velocity limit.
	DECELERATE_FOR_APPROACHING_SINGULARITY (102): Returned if the Validator needs to break, because of an approaching singularity (Only sent when using cartesian mode in Validator).
 	HALT_FOR_SINGULARITY (103): Returned if the Validator needs to halt, because of a singularity (Only sent when using cartesian mode in Validator).
  	DECELERATE_FOR_COLLISION (104): Returned if the Validator needs to break, because of an upcomming collision.
   	HALT_FOR_COLLISION (105): Returned if the Validator needs to halt, because of a collision.
	POSITION_LIMIT (106): Returned if the Validator is close to a Joint Position limit and needs to break/halt.
	DECELERATE_FOR_LEAVING_SINGULARITY: Sent when robot is currently leaving the singularity (Only sent when using cartesian mode in Validator).
 
## Adding a new message type
Right now it is difficult to add new message types. I will work on it, to make it easier. Currently it is only possible to extend the publishers, not the subscribers.
1. In types.hpp: extend the MessageType enum with your new message and assign it your type number.
2. In types.hpp: add your new enum to the mapping array, along with it's String representation and default topic.
3. In publisher_factory.hpp: Add your new enum to the switch case in create_publisher and return a new publisher with the msg it should have
4. In json_serializer_msgs.hpp: If your msg isn't yet present: Add a wrapper struct around your message. (Similar to the others) And define the to_json and from_json methods.
5. In json_serializer_msgs.cpp: Implement the to_json and from_json methods.
6. In json_serializer.cpp: Add your MessageType to the get_message_content method

## Protocols
In general, there are 3 types of Protocols supported: TCP, Unix Domain Sockets and ZeroMQ.
Both TCP and Unix Domain Sockets use ZeroMQ under the hood. One Push and one Pull socket are created (one for sending and one for receiving).
### protocols::TCPSocket
Implementation of the Transmission Control Protocol to communicate with the SDK.
#### Parameters:
- `ip_address`: String
	- Default: "127.0.0.1"
	- Description: The IP-Address the TCP connection should be established on
- `port_send`: int
	- Default: 5556
	- Description: The port the TCP Push Socket should be established on
- `port_recv`: int
  	- Default: 5555
  	- Description: The port the TCP Pull Socket should be established on
### protocols::UnixDomainSocket
Implementation of Unix Domain Sockets to communicate with the SDK.
#### Parameters:
- `socket_path_recv`: String
	- Default: "/tmp/ros2_api_recv.socket"
	- Description: The path to the socket file, that is used for the PULL Socket
- `socket_path_send`: String
	- Default: "/tmp/ros2_api_send.socket"
 	- Description: The path to the socket file, that is used for the PUSH Socket 	 	 
### protocols::ZeroMQ
Implementation of ZeroMQ to communicate with the SDK.
#### Parameters:
- `endpoint_recv`: String
	- Default: "tcp://127.0.0.1:5555"
 	- Description: The ZeroMQ endpoint used to create the PULL Socket.
- `endpoint_send`: String
	- Default: "tcp://127.0.0.1:5556"
 	- Description: The ZeroMQ endpoint used to create the PUSH Socket. 	  	
### Adding a new protocol
1. Extend protocol_base::CommunicationProtocol
2. Implement: 
	- `sendToClient(const  std::uint8_t  *message, int  length)`
		- This method is called when the API wants to send a message
	- `start()`
		- This method is called in the beginning and should setup the server
	- `stop()`
		- This method is called when the API is stopped.
	- `initialize(const  YAML::Node  &config)`
		- In this method you can set your protocols parameters.
		- It uses [yaml-cpp](https://github.com/jbeder/yaml-cpp), so make sure to add it to your CMake and package.xml if you plan to use it.
	- To pass down data to ROS2 when a message is received, call `callback_(const  std::uint8_t  *message, int  length)`
3. At the bottom of your cpp file, add:
	`PLUGINLIB_EXPORT_CLASS(YOUR_CLASS_NAME, protocol_base::CommunicationProtocol)` 
4. Add a plugins.xml file as described [here](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Pluginlib.html#plugin-declaration-xml)
5. Edit your CMakeLists.txt as described [here](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Pluginlib.html#cmake-plugin-declaration)
6. Now you can use your plugin by adding YOUR_CLASS_NAME and the defined parameters under transport.type and transport.params in the config.yaml file.

IMPORTANT: Threading is the plugins job! You have to make sure no deadlocks or race conditions can happen!

## Launch
To launch the API, you have to pass the path to your YAML file as a string.
Additionally, you have to set `use_sim_time` to true or false, based on if you are using Gazebo or not.
Example:

    from  launch  import  LaunchDescription
	from  launch_ros.actions  import  Node
	from  launch_ros.actions  import  Node
	from  launch_ros.substitutions  import  FindPackageShare
	from  launch.substitutions  import  PathJoinSubstitution
	
	def  generate_launch_description():
		# EXCHANGE WITH YOUR PATH THE CONFIG FILE
		config_file  =  PathJoinSubstitution([FindPackageShare('core') , 'config', 'api_config.yaml'])
  		# EXCHANGE WITH YOUR USE CASE
  		use_sim_time = 'false'
		node_server  =  Node(
			package='core',
			executable='ros2_api_node',
			output='screen',
			arguments=['--config_file', config_file],
   			parameters=[{'use_sim_time': use_sim_time}]
		)
		return  LaunchDescription([node_server])
