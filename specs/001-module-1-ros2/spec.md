# Feature Specification: Module 1 - The Robotic Nervous System (ROS 2)

**Feature Branch**: `001-module-1-ros2`
**Created**: 2025-11-30
**Status**: Draft
**Module Number**: 1
**Estimated Duration**: 12 hours over 4 weeks (3 hours/week)
**Prerequisites**: Basic Python programming, CLI familiarity, Ubuntu/Linux basics OR Windows WSL2, No prior robotics required
**Target Audience**: Software developers, AI engineers, robotics enthusiasts new to ROS 2 who want to build Physical AI applications

## Module Description

ROS 2 (Robot Operating System 2) is the middleware that powers modern robotics. This module introduces the ROS 2 graph model: nodes publish and subscribe to topics, call services for request-response patterns, and describe robot morphology using URDF. Learners will build pub/sub systems for real-time sensor streaming, create custom message types, and visualize robot models in RViz2.

**Key Focus**: Communication patterns (publish-subscribe, services), robot modeling (URDF), visualization (RViz2), and foundation for simulation, navigation, and voice control.

## User Scenarios & Testing

### Learning Journey 1: ROS 2 Communication Fundamentals (Priority: P1)

Learners observe ROS 2 through turtlesim demo, identify nodes/topics with CLI tools, modify publisher code, create talker/listener pairs, design custom message types for sensor data.

**Why this priority**: Foundation of all ROS 2 development - required for services, actions, robot control, AI integration. Without understanding pub/sub, learners cannot progress to simulation, navigation, or VLA modules.

**Independent Test**: Create Python ROS 2 node publishing custom sensor data, subscribe with `ros2 topic echo`, visualize with `rqt_graph`. Demonstrates complete understanding of communication patterns.

**Acceptance Scenarios**:

1. **Given** ROS 2 Humble installed on Ubuntu 22.04, **When** running `ros2 run turtlesim turtlesim_node` and `ros2 topic list`, **Then** learner identifies `/turtle1/cmd_vel` and `/turtle1/pose` topics
2. **Given** turtlesim running, **When** executing `ros2 topic echo /turtle1/pose`, **Then** learner observes real-time position updates in terminal
3. **Given** template `talker.py` node, **When** modifying message content to "Hello Physical AI", **Then** `ros2 topic echo /chatter` displays modified message
4. **Given** understanding of std_msgs, **When** creating custom `SensorData.msg` with temperature/humidity fields, **Then** publisher node successfully sends data and subscriber receives it
5. **Given** multiple nodes running, **When** launching `rqt_graph`, **Then** learner identifies message flow direction and diagnoses communication issues

---

### Learning Journey 2: Robot Descriptions with URDF (Priority: P2)

Learners examine existing URDF files, modify properties (geometry, color), create articulated robots (robotic arm with joints), adapt humanoid template with torso/arms/legs.

**Why this priority**: Essential for Gazebo/Isaac simulation (Module 2-3), but secondary to communication fundamentals. Cannot visualize or simulate robots without URDF.

**Independent Test**: Create URDF for humanoid with 10+ links (torso, head, arms, legs), load in RViz2, visualize with correct parent-child relationships in TF tree.

**Acceptance Scenarios**:

1. **Given** existing `simple_robot.urdf`, **When** changing box geometry dimensions and visual color, **Then** RViz2 displays updated robot model
2. **Given** understanding of `<link>` and `<joint>` tags, **When** creating 2-joint robotic arm URDF with revolute joints, **Then** RViz2 visualizes arm structure with joint axes
3. **Given** humanoid template with torso and head, **When** adding left/right arm links with shoulder/elbow joints, **Then** RViz2 TF tree shows correct parent-child hierarchy
4. **Given** complete humanoid URDF with 12 links, **When** publishing joint states via `joint_state_publisher_gui`, **Then** robot animates smoothly in RViz2

---

### Learning Journey 3: Services for Command-Response (Priority: P2)

Learners call existing services (turtlesim spawn), create service server (AddTwoInts), design robot control service with input validation, analyze when to use topics vs services.

**Why this priority**: Complements topics for synchronous operations (robot reset, gripper control). Common in robot control but less fundamental than pub/sub.

**Independent Test**: Create service server with input validation (e.g., motor speed limits), call from client node or CLI using `ros2 service call`.

**Acceptance Scenarios**:

1. **Given** turtlesim running, **When** calling `ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 2, theta: 0, name: 'turtle2'}"`, **Then** new turtle appears at coordinates (2,2)
2. **Given** template service server code, **When** calling AddTwoInts service with `ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 3}"`, **Then** response returns sum of 8
3. **Given** understanding of `.srv` file format, **When** creating custom `SetMotorSpeed.srv` (request: int32 speed, response: bool success), **Then** service compiles and responds correctly
4. **Given** scenario analysis (real-time sensor streaming vs one-time configuration), **When** evaluating topics vs services, **Then** learner justifies choice with latency/guarantee tradeoffs

---

### Edge Cases

- What happens when a subscriber connects to a topic before any publisher exists? (ROS 2 allows late joining; subscriber waits)
- How does the system handle invalid URDF syntax? (robot_state_publisher fails with XML parsing error; check with `check_urdf`)
- What if a service server crashes while processing a request? (Client receives timeout error after default 5s; handle with try/except)
- How does ROS 2 handle network partitions in distributed systems? (DDS discovery may lose nodes; monitor with `ros2 node list`)

## Requirements

### Functional Requirements

- **FR-001**: Learners MUST install ROS 2 Humble on Ubuntu 22.04 or Windows WSL2 and verify installation with turtlesim demo
- **FR-002**: Learners MUST create Python publisher nodes using rclpy that send messages to custom topics at configurable rates (1-10 Hz)
- **FR-003**: Learners MUST create Python subscriber nodes that receive and process messages from topics with callback functions
- **FR-004**: Learners MUST design custom message types (.msg files) with primitive fields (int32, float64, string, bool) and arrays
- **FR-005**: Learners MUST build ROS 2 packages containing custom messages using colcon build system
- **FR-006**: Learners MUST create URDF files with at least 5 links and 4 joints describing articulated robots (arms or humanoids)
- **FR-007**: Learners MUST load URDF models in RViz2 and verify correct visualization with TF tree display
- **FR-008**: Learners MUST create service servers that respond to client requests with custom .srv message types
- **FR-009**: Learners MUST use ros2 CLI tools (node list, topic echo, topic info, service call) to debug running systems
- **FR-010**: Learners MUST visualize ROS 2 graph topology using rqt_graph to identify nodes, topics, and message flow

### Key Entities

- **ROS 2 Node**: Fundamental computation unit; executable process participating in ROS 2 graph (e.g., sensor driver, controller)
- **Topic**: Named bus for asynchronous many-to-many communication; publishers send, subscribers receive (e.g., `/camera/image_raw`)
- **Publisher**: Node component that sends messages to a topic at configurable rate (e.g., 10 Hz sensor data stream)
- **Subscriber**: Node component that receives messages from a topic via callback function (e.g., image processor)
- **Message Type**: Data structure defining topic payload schema (e.g., `geometry_msgs/Twist` for velocity commands)
- **Service**: Synchronous request-response communication pattern; client sends request, server returns response (e.g., `/spawn` turtle)
- **URDF (Unified Robot Description Format)**: XML-based robot model describing links (rigid bodies), joints (connections), visual/collision geometry
- **Link**: Rigid body component in URDF with geometry, inertia, visual appearance (e.g., robot arm segment)
- **Joint**: Connection between two links defining motion constraints (revolute, prismatic, fixed, continuous)
- **RViz2**: 3D visualization tool for robot models, sensor data, transforms, and navigation paths
- **rclpy**: ROS 2 Python client library providing APIs for nodes, publishers, subscribers, services
- **DDS (Data Distribution Service)**: Middleware layer beneath ROS 2 enabling real-time peer-to-peer communication

## Success Criteria

### Measurable Outcomes

- **SC-001**: Learners can install ROS 2 Humble and run turtlesim demo within 30 minutes (85% success rate)
- **SC-002**: Learners can create talker/listener node pair and verify communication with `ros2 topic echo` within 45 minutes (80% success rate)
- **SC-003**: Learners can design custom message type (3+ fields), build package, and publish/subscribe within 60 minutes (75% success rate)
- **SC-004**: Learners can create URDF with 5+ links, load in RViz2, and verify TF tree within 60 minutes (70% success rate)
- **SC-005**: Learners can create service server/client pair with custom .srv type and call successfully within 45 minutes (75% success rate)
- **SC-006**: Learners score 80%+ on ROS 2 concepts quiz (nodes, topics, messages, services, URDF) covering Remember/Understand levels
- **SC-007**: Learners complete mini-project (sensor tower with 3 sensors + monitor node) with all requirements within 90 minutes (70% success rate)
- **SC-008**: Learners can debug communication issues using ros2 CLI tools and rqt_graph within 30 minutes (75% success rate)

## Dependencies

### Within-Module Dependencies

- **Chapter 1.1** (Introduction to ROS 2) is foundational for all subsequent chapters
- **Chapter 1.2** (Publishers & Subscribers) is prerequisite for:
  - Chapter 1.3 (Custom Messages) - extends pub/sub with custom types
  - Chapter 1.4 (Services) - alternative communication pattern
  - Chapter 1.7 (Mini-Project) - requires pub/sub mastery
- **Chapter 1.5** (URDF) is prerequisite for:
  - Chapter 1.6 (RViz2) - visualizes URDF models
  - Chapter 1.7 (Mini-Project) - may include robot visualization

### Cross-Module Dependencies (Provides Foundation For)

- **Module 2 (Gazebo & Unity)** requires:
  - Chapter 1.2: Pub/sub for sensor data streaming in simulation
  - Chapter 1.5: URDF for spawning robots in Gazebo
  - Chapter 1.6: RViz2 for visualizing simulated sensor data
- **Module 3 (NVIDIA Isaac)** requires:
  - Chapter 1.2: Pub/sub for Isaac ROS nodes
  - Chapter 1.4: Services for Isaac Sim control
  - Chapter 1.5: URDF for robot models in Isaac Sim
- **Module 4 (VLA)** requires:
  - Chapter 1.2: Pub/sub for voice command streaming
  - Chapter 1.4: Services for LLM action requests
  - All core ROS 2 concepts for AI-robot integration
- **Module 5 (Capstone)** requires complete Module 1 mastery

### External Dependencies

- **ROS 2 Humble** (LTS release, Ubuntu 22.04 recommended)
- **Python 3.10+** (for rclpy nodes)
- **colcon** (ROS 2 build system)
- **Ubuntu 22.04** (recommended) OR Windows 10/11 with WSL2
- **RViz2** (included with ROS 2 desktop install)
- **rqt_graph** (included with ROS 2 desktop install)

## Out of Scope

- **ROS 2 Actions**: Deferred to Module 3 (long-running tasks like navigation)
- **ROS 1**: Legacy system; course focuses exclusively on ROS 2
- **Real Hardware Integration**: Deferred to Module 4 (simulations only in Module 1)
- **Multi-Robot Systems**: Single robot focus; swarms out of scope
- **C++ rclcpp**: Python-only course for accessibility
- **Custom DDS Configuration**: Default Fast-DDS sufficient; advanced tuning out of scope
- **ROS 2 Parameters**: Brief mention only; not core focus
- **Launch Files**: Introduced in Module 2 (simulation context)
- **tf2 Transforms**: Basic understanding only; depth deferred to Module 3 (navigation)

## Assumptions

- Learners have basic Python knowledge (functions, classes, imports, error handling)
- Learners are comfortable with command-line interfaces (bash, file navigation)
- Learners have access to Ubuntu 22.04 machine or Windows with WSL2
- Learners can install software packages with sudo privileges
- Learners have stable internet connection for ROS 2 package installation
- Learners commit 3 hours per week for 4 weeks (12 total hours)

## Chapter Breakdown

### Chapter 1.1: Introduction to ROS 2 and Physical AI

**Duration**: 2 hours
**Prerequisites**: None (module entry point)

**Learning Objectives**:
- **Remember**: Define ROS 2, node, topic, message, Physical AI, and middleware
- **Understand**: Explain why Physical AI requires middleware like ROS 2 for distributed robotics systems
- **Understand**: Compare ROS 1 vs ROS 2 architecture (centralized master vs DDS peer-to-peer)
- **Apply**: Install ROS 2 Humble on Ubuntu 22.04 and run turtlesim demo to verify installation

**Content Outline**:
1. **Introduction** (20 min): Physical AI definition, embodied intelligence examples (humanoid robots, autonomous vehicles), gap between digital AI and physical world
2. **Deep Dive** (60 min): Middleware role in robotics, ROS 2 graph architecture (nodes, topics, publishers, subscribers), DDS layer, ROS 1 vs ROS 2 comparison table
3. **Hands-On** (30 min): ROS 2 Humble installation walkthrough, environment setup (bashrc), turtlesim demo, basic teleop keyboard control
4. **Summary** (10 min): Recap middleware necessity, preview of pub/sub patterns in Chapter 1.2

**Code Examples**:
- `book_frontend/static/examples/module-1/ch1.1/install_ros2_humble.sh` - Automated installation script (bash, 50 lines, adds ROS repo, installs ros-humble-desktop, sources setup.bash)
- `book_frontend/static/examples/module-1/ch1.1/bashrc_setup.sh` - ROS 2 environment configuration (bash, 10 lines, adds source command to .bashrc)

**Diagrams & Media**:
- `book_frontend/static/img/module-1/ch1.1/ros2_graph_concept.png` - ROS 2 graph with 3 nodes (camera, detector, controller) and 2 topics (/image, /detections), arrows showing message flow (PNG, <150KB, alt: "ROS 2 graph showing three nodes: camera_node publishing to /image topic, detector_node subscribing to /image and publishing to /detections, controller_node subscribing to /detections. Arrows indicate message flow direction.")
- `book_frontend/static/img/module-1/ch1.1/ros1_vs_ros2.png` - Side-by-side comparison: ROS 1 with central rosmaster vs ROS 2 DDS peer-to-peer (PNG, <120KB, alt: "Comparison diagram: ROS 1 shows all nodes connected to central rosmaster. ROS 2 shows nodes directly discovering each other via DDS without central point of failure.")
- `book_frontend/static/img/module-1/ch1.1/turtlesim_screenshot.png` - Turtlesim window with turtle and drawing trail (PNG, <100KB, alt: "Turtlesim window showing blue turtle at center with colorful trail path. Window title shows 'TurtleSim'.")

**Assessments**:
- **Formative**: 5-question quiz on Physical AI definition, middleware role, ROS 1 vs ROS 2 (90% expected pass rate)
- **Summative**: Installation challenge - learners must screenshot turtlesim running with custom turtle path (85% expected pass rate)

---

### Chapter 1.2: Your First ROS 2 Node - Publishers & Subscribers

**Duration**: 2.5 hours
**Prerequisites**: Chapter 1.1 (ROS 2 installation)

**Learning Objectives**:
- **Remember**: Define publisher, subscriber, callback function, rclpy, and message type
- **Understand**: Explain pub/sub pattern benefits (decoupling, scalability, asynchronous communication)
- **Apply**: Create Python publisher node that sends String messages at 1 Hz rate
- **Apply**: Create Python subscriber node that receives and prints messages via callback
- **Analyze**: Use `ros2 topic echo`, `ros2 topic info`, and `rqt_graph` to debug communication issues

**Content Outline**:
1. **Introduction** (15 min): Pub/sub pattern overview, real-world analogy (newspaper publisher/subscribers), asynchronous vs synchronous communication
2. **Deep Dive** (60 min): rclpy library structure, creating nodes with `rclpy.init()`, publisher creation with `create_publisher()`, timer callbacks, QoS profiles, subscriber creation with `create_subscription()`, callback functions, message types from std_msgs
3. **Hands-On** (60 min): Write talker.py from scratch, run with `ros2 run`, modify message content, write listener.py, verify with `ros2 topic echo /chatter`, visualize with `rqt_graph`
4. **Summary** (15 min): Recap pub/sub workflow, preview custom messages in Chapter 1.3

**Code Examples**:
- `book_frontend/static/examples/module-1/ch1.2/talker.py` - Publisher node sending "Hello ROS 2" at 1 Hz (Python, 25 lines, uses `std_msgs/String`, includes node initialization, publisher creation, timer callback)
  - **Expected Output**: `ros2 topic echo /chatter` displays "Hello ROS 2" every second
  - **Testing**: Run `ros2 run demo_nodes_py talker`, verify output with topic echo
- `book_frontend/static/examples/module-1/ch1.2/listener.py` - Subscriber node printing received messages (Python, 20 lines, uses callback function, logs to terminal)
  - **Expected Output**: Terminal prints "[INFO] I heard: Hello ROS 2" every second
  - **Testing**: Run talker and listener in separate terminals, verify both see messages
- `book_frontend/static/examples/module-1/ch1.2/package.xml` - ROS 2 package metadata (XML, 15 lines, declares rclpy and std_msgs dependencies)
- `book_frontend/static/examples/module-1/ch1.2/setup.py` - Python package setup for colcon build (Python, 20 lines, defines entry points for talker/listener executables)

**Diagrams & Media**:
- `book_frontend/static/img/module-1/ch1.2/pubsub_flow.png` - Pub/sub message flow diagram showing publisher → topic → subscriber with timestamps (PNG, <130KB, alt: "Publisher node creates message at t=0, publishes to /chatter topic at t=1ms, subscriber receives via callback at t=3ms. Arrows show message flow with timing annotations.")
- `book_frontend/static/img/module-1/ch1.2/rqt_graph_talker_listener.png` - Screenshot of rqt_graph showing talker and listener nodes connected by /chatter topic (PNG, <140KB, alt: "rqt_graph screenshot: talker node (oval) connected by arrow to /chatter topic (rectangle), which connects to listener node (oval). Graph shows active communication.")

**Assessments**:
- **Formative**: 8-question quiz on pub/sub concepts, rclpy APIs, message types (85% expected pass rate)
- **Summative**: Coding challenge - create publisher/subscriber pair for custom topic name "/sensor_data", modify message rate to 5 Hz, verify with CLI tools (75% expected pass rate)

---

### Chapter 1.3: Custom Messages and Data Structures

**Duration**: 2 hours
**Prerequisites**: Chapter 1.2 (Publishers & Subscribers)

**Learning Objectives**:
- **Understand**: Explain when to use custom messages vs standard message types (std_msgs, geometry_msgs, sensor_msgs)
- **Apply**: Create .msg file with primitive types (int32, float64, string, bool) and arrays
- **Apply**: Build ROS 2 package with custom messages using colcon and verify message generation
- **Create**: Design sensor message type with 5+ fields representing real sensor data (IMU, GPS)

**Content Outline**:
1. **Introduction** (15 min): Limitations of std_msgs (lack of semantic meaning), benefits of custom types (type safety, clarity)
2. **Deep Dive** (45 min): .msg file syntax, primitive types (int8, uint16, float32, string, bool, etc.), arrays (fixed-size and dynamic), nested messages, message generation process (rosidl), package dependencies (rosidl_default_generators)
3. **Hands-On** (50 min): Create SensorData.msg with temperature/humidity/timestamp, modify package.xml/CMakeLists.txt, colcon build, import in publisher/subscriber nodes, test communication
4. **Summary** (10 min): Best practices for message design, preview of services in Chapter 1.4

**Code Examples**:
- `book_frontend/static/examples/module-1/ch1.3/msg/SensorData.msg` - Custom message with temperature (float32), humidity (float32), timestamp (int64)
- `book_frontend/static/examples/module-1/ch1.3/msg/IMU.msg` - IMU sensor message with acceleration (float64[3]), gyroscope (float64[3]), orientation (float64[4])
- `book_frontend/static/examples/module-1/ch1.3/publisher_custom.py` - Publisher using SensorData message type
- `book_frontend/static/examples/module-1/ch1.3/CMakeLists.txt` - Build configuration for message generation

**Diagrams & Media**:
- `book_frontend/static/img/module-1/ch1.3/message_generation.png` - Flowchart: .msg file → rosidl → Python/C++ classes (PNG, <110KB, alt: "Message generation pipeline: developer writes SensorData.msg file, rosidl generator creates Python and C++ classes, nodes import generated classes.")

**Assessments**:
- **Formative**: Quiz on message types, .msg syntax, build process (80% pass rate)
- **Summative**: Create GPS message type (latitude, longitude, altitude, satellites), build package, publish/subscribe (70% pass rate)

---

### Chapter 1.4: Services and Request-Response Patterns

**Duration**: 2 hours
**Prerequisites**: Chapters 1.2-1.3 (Pub/Sub fundamentals)

**Learning Objectives**:
- **Understand**: Compare topics vs services (asynchronous vs synchronous, many-to-many vs one-to-one)
- **Apply**: Create service server that responds to client requests with custom .srv type
- **Apply**: Call services from Python client node and CLI using `ros2 service call`
- **Analyze**: Evaluate scenarios to determine when to use topics vs services (e.g., sensor streaming vs robot reset)

**Content Outline**:
1. **Introduction** (15 min): Service pattern overview, request-response analogy (HTTP request), use cases (robot configuration, one-time commands)
2. **Deep Dive** (45 min): .srv file structure (request/response sections), creating service servers with `create_service()`, handling requests, creating clients with `create_client()`, synchronous vs asynchronous calls
3. **Hands-On** (50 min): Implement AddTwoInts service server, call from CLI and Python client, create SetMotorSpeed.srv with validation (speed limits), error handling
4. **Summary** (10 min): Topics vs services decision matrix, preview of URDF in Chapter 1.5

**Code Examples**:
- `book_frontend/static/examples/module-1/ch1.4/srv/AddTwoInts.srv` - Service definition (request: int64 a, int64 b | response: int64 sum)
- `book_frontend/static/examples/module-1/ch1.4/service_server.py` - AddTwoInts service server implementation
- `book_frontend/static/examples/module-1/ch1.4/service_client.py` - Python client calling AddTwoInts service
- `book_frontend/static/examples/module-1/ch1.4/srv/SetMotorSpeed.srv` - Custom service with validation (request: int32 speed | response: bool success, string message)

**Diagrams & Media**:
- `book_frontend/static/img/module-1/ch1.4/service_flow.png` - Service request-response sequence diagram (PNG, <120KB, alt: "Client sends request to /add_two_ints service at t=0, server processes at t=1-5ms, returns response at t=6ms. Arrows show synchronous blocking communication.")
- `book_frontend/static/img/module-1/ch1.4/topics_vs_services.png` - Comparison table (PNG, <100KB, alt: "Table comparing topics (many-to-many, asynchronous, continuous data) vs services (one-to-one, synchronous, one-time requests).")

**Assessments**:
- **Formative**: Quiz on service concepts, .srv syntax (80% pass rate)
- **Summative**: Implement ResetRobot service (request: none | response: bool success), call from client (75% pass rate)

---

### Chapter 1.5: Robot Descriptions with URDF

**Duration**: 2.5 hours
**Prerequisites**: Chapter 1.2 (basic ROS 2 knowledge)

**Learning Objectives**:
- **Understand**: Explain URDF structure (links, joints, visual/collision geometry, kinematic tree)
- **Apply**: Create URDF file for simple robot (two-wheeled robot with chassis)
- **Apply**: Define joints (revolute, prismatic, fixed, continuous) with limits and axes
- **Create**: Design humanoid URDF with torso, head, arms, legs (10+ links, 9+ joints)

**Content Outline**:
1. **Introduction** (15 min): Robot modeling importance for simulation and visualization, URDF vs other formats (SDF, MJCF)
2. **Deep Dive** (60 min): XML structure, `<robot>` root element, `<link>` tag (visual, collision, inertial), `<joint>` tag (type, parent, child, origin, axis, limits), coordinate systems, robot kinematic tree
3. **Hands-On** (60 min): Create simple_robot.urdf (box chassis + 2 wheels), add revolute joints for wheels, create robotic_arm.urdf (2 links with shoulder/elbow joints), extend to humanoid template
4. **Summary** (15 min): Best practices (modular design, realistic inertia), preview of RViz2 in Chapter 1.6

**Code Examples**:
- `book_frontend/static/examples/module-1/ch1.5/simple_robot.urdf` - Two-wheeled robot URDF (XML, 80 lines, chassis link + 2 wheel links + 2 continuous joints)
- `book_frontend/static/examples/module-1/ch1.5/robotic_arm.urdf` - 2-DOF robotic arm (XML, 100 lines, base + link1 + link2 + 2 revolute joints with limits)
- `book_frontend/static/examples/module-1/ch1.5/humanoid_template.urdf` - Humanoid starter template (XML, 200 lines, torso + head + 2 arms with placeholder geometry)

**Diagrams & Media**:
- `book_frontend/static/img/module-1/ch1.5/urdf_structure.png` - URDF kinematic tree diagram (PNG, <130KB, alt: "URDF tree: base_link (root) has child wheel_left_link and wheel_right_link via continuous joints. Arrows show parent-child relationships.")
- `book_frontend/static/img/module-1/ch1.5/joint_types.png` - Joint types illustration (fixed, revolute, prismatic, continuous) with motion arrows (PNG, <140KB, alt: "Four joint types: fixed (no motion), revolute (rotation around axis with limits), prismatic (linear translation), continuous (unlimited rotation).")

**Assessments**:
- **Formative**: Quiz on URDF syntax, link/joint concepts (85% pass rate)
- **Summative**: Create humanoid torso with head and 2 arms (5 links, 4 joints), verify structure (70% pass rate)

---

### Chapter 1.6: Visualizing Robots in RViz2

**Duration**: 1.5 hours
**Prerequisites**: Chapter 1.5 (URDF creation)

**Learning Objectives**:
- **Apply**: Load URDF in RViz2 using `robot_state_publisher` and visualize robot model
- **Apply**: Configure RViz2 displays (RobotModel, TF, Grid, Axes) for clear visualization
- **Analyze**: Inspect TF tree to verify correct link parent-child relationships
- **Create**: Publish joint states using `joint_state_publisher_gui` to animate robot

**Content Outline**:
1. **Introduction** (10 min): RViz2 role in robot development (debugging, testing, visualization)
2. **Deep Dive** (30 min): RobotModel display, TF (transform) tree, robot_state_publisher node, joint_state_publisher node, RViz2 configuration files (.rviz)
3. **Hands-On** (40 min): Launch RViz2, load simple_robot.urdf, add RobotModel display, inspect TF tree, use joint_state_publisher_gui to move arm joints, save .rviz config
4. **Summary** (10 min): RViz2 best practices, preview of mini-project in Chapter 1.7

**Code Examples**:
- `book_frontend/static/examples/module-1/ch1.6/launch/display.launch.py` - ROS 2 launch file to start RViz2 + robot_state_publisher (Python, 30 lines)
- `book_frontend/static/examples/module-1/ch1.6/config/robot.rviz` - RViz2 configuration file (YAML, 50 lines, displays RobotModel + TF + Grid)

**Diagrams & Media**:
- `book_frontend/static/img/module-1/ch1.6/rviz2_interface.png` - RViz2 screenshot with labeled interface (PNG, <180KB, alt: "RViz2 interface showing robot model in center, display list on left (RobotModel, TF, Grid checked), tool properties below, 3D view in center with grid and axes.")

**Assessments**:
- **Formative**: Quiz on RViz2 displays, TF tree (80% pass rate)
- **Summative**: Visualize humanoid model from Chapter 1.5, animate with joint_state_publisher_gui (75% pass rate)

---

### Chapter 1.7: Module 1 Mini-Project - Sensor Tower

**Duration**: 1.5 hours
**Prerequisites**: All prior chapters (1.1-1.6)

**Project Description**: Build a sensor tower system with 3 publisher nodes (temperature, humidity, light sensors) sending data at different rates (1 Hz, 2 Hz, 5 Hz) to separate topics. Create a monitor node that subscribes to all 3 topics and logs data to terminal. Visualize system with rqt_graph. Optional: create URDF model of sensor tower and display in RViz2.

**Requirements**:
1. Create 3 custom message types (Temperature.msg, Humidity.msg, LightLevel.msg) with sensor value and timestamp
2. Implement 3 publisher nodes with configurable rates
3. Implement monitor node subscribing to all 3 topics with separate callbacks
4. Verify communication with `ros2 topic echo` and `rqt_graph`
5. (Optional) Create sensor_tower.urdf with 3 sensor links and visualize in RViz2

**Assessment Rubric** (100 points):
- Custom messages built successfully (20 points)
- All 3 publishers send data at correct rates (30 points)
- Monitor node receives and logs all sensor data (30 points)
- rqt_graph shows correct topology (10 points)
- Code documentation and README (10 points)
- (Bonus) URDF visualization in RViz2 (10 points)

**Code Examples**:
- `book_frontend/static/examples/module-1/ch1.7/msg/Temperature.msg` - Temperature message template
- `book_frontend/static/examples/module-1/ch1.7/temperature_publisher.py` - Example publisher structure (learners implement logic)
- `book_frontend/static/examples/module-1/ch1.7/monitor_node.py` - Example subscriber structure with multiple callbacks

**Diagrams & Media**:
- `book_frontend/static/img/module-1/ch1.7/sensor_tower_graph.png` - Expected rqt_graph output (PNG, <150KB, alt: "rqt_graph showing 3 publisher nodes (temperature_node, humidity_node, light_node) connected to 3 topics (/temperature, /humidity, /light), all flowing to monitor_node.")

**Assessments**:
- **Summative**: Complete mini-project with all requirements met, submit code + screenshot of rqt_graph (70% expected pass rate)

---

## Non-Functional Requirements

### Accessibility (WCAG 2.1 AA Compliance)

- **NFR-001**: All diagrams and images MUST include descriptive alt text explaining visual content for screen readers
- **NFR-002**: Text contrast ratio MUST meet 4.5:1 minimum for body text, 3:1 for large text (18pt+)
- **NFR-003**: Code examples MUST use semantic HTML with proper syntax highlighting in Docusaurus
- **NFR-004**: Video content (if added) MUST include captions and transcripts
- **NFR-005**: Navigation MUST be keyboard-accessible (tab order, skip links)

### Code Quality

- **NFR-006**: All code examples MUST be tested and executable without errors on ROS 2 Humble + Ubuntu 22.04
- **NFR-007**: Code examples MUST include inline comments explaining ROS 2 concepts (not Python basics)
- **NFR-008**: No hardcoded secrets, API keys, or absolute file paths in example code
- **NFR-009**: Naming conventions: snake_case for Python variables/functions, PascalCase for message types, lowercase with underscores for topics
- **NFR-010**: All example code MUST include expected output documentation

### Documentation & Writing Style

- **NFR-011**: Learning objectives MUST use Bloom's Taxonomy action verbs (Remember, Understand, Apply, Analyze, Create)
- **NFR-012**: All acronyms MUST be defined on first use in each chapter (e.g., "ROS 2 (Robot Operating System 2)")
- **NFR-013**: Writing style MUST use active voice and second person ("you will create" not "one should create")
- **NFR-014**: Markdown MUST use semantic headings (h2 for chapters, h3 for sections, h4 for subsections)
- **NFR-015**: Code blocks MUST use fenced blocks with language identifiers (```python, ```xml, ```bash)

### Performance

- **NFR-016**: Docusaurus pages MUST load in <2 seconds on 10 Mbps connection
- **NFR-017**: Images MUST be compressed to <200 KB per file using PNG or optimized JPEG
- **NFR-018**: No autoplay video/audio; user-initiated media only
- **NFR-019**: Lighthouse performance score MUST be >90 for all module pages

### Usability

- **NFR-020**: Each chapter MUST include estimated completion time (30 min increments)
- **NFR-021**: Prerequisites MUST be clearly stated at chapter start with links to required chapters
- **NFR-022**: Assessments MUST include expected pass rate to set learner expectations
- **NFR-023**: Error messages in code examples MUST include troubleshooting tips (common issues, fixes)

---

## Quality Checklist

### Specification Completeness

- [ ] All 3 learning journeys include Given/When/Then acceptance scenarios
- [ ] All 7 chapters have learning objectives using Bloom's Taxonomy
- [ ] All code examples specify filename, language, purpose, expected output, testing
- [ ] All diagrams specify filename, alt text, size constraint (<200KB)
- [ ] All dependencies (within-module and cross-module) documented
- [ ] Out of scope section prevents scope creep
- [ ] Success criteria are measurable (time limits, pass rates)

### Constitution Compliance

- [ ] Markdown quality: semantic headings (h2-h4), fenced code blocks, 100-char wrapping
- [ ] Accessibility: WCAG 2.1 AA alt text for all images, 4.5:1 contrast ratio
- [ ] Code quality: tested examples, no hardcoded secrets, meaningful names
- [ ] Writing style: active voice, second person, acronyms defined
- [ ] Performance: <2s page load, <200KB images, Lighthouse >90

### Educational Best Practices

- [ ] Learning objectives span multiple Bloom's levels (not just Remember)
- [ ] Hands-on activities comprise 40%+ of chapter time (active learning)
- [ ] Assessments align with learning objectives (formative + summative)
- [ ] Code examples demonstrate one concept at a time (not overwhelming)
- [ ] Difficulty progression: foundational → intermediate → advanced

---

## Next Steps

1. **Clarification** (if needed): Run `/sp.clarify` to identify underspecified sections
2. **Planning**: Run `/sp.plan` to design implementation approach (chapter writing, code creation, diagram design)
3. **Task Breakdown**: Run `/sp.tasks` to generate granular task list (T001: Write Chapter 1.1 intro, T002: Create install script, etc.)
4. **Implementation**: Run `/sp.implement` to execute tasks (write chapters to `book_frontend/docs/module-1/`, create code examples in `book_frontend/static/examples/module-1/`)
5. **Review & Iterate**: Test code examples, verify accessibility, gather learner feedback

---

**Specification Status**: Ready for planning phase
**Estimated Implementation Time**: 40-50 hours (chapter writing, code creation, diagram design, testing)
