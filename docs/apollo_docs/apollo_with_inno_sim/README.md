# Running Apollo with Innopolis Simulator

In this section we will see how to run the Apollo Auto framework along with the Innopolis Simulator

![Autonomous driving](images/autonomous_driving.png "Autonomous driving")

## Installation steps
The steps described below were tested on Ubuntu 20.04 LTS

### Downloading the simulator

Download the Innopolis Simulator from [here](https://github.com/inno-robolab/InnoSimulator/releases). Extract it into the `simulator` folder or in any folder you prefer. We will refer to the folder to where you have extracted the simulator as the `simulator` folder

Do not forget to make the simulator executable with `chmod +x InnoSimulator.x86_64`

### Downloading Apollo

Download a zip archive with apollo code from [here](https://github.com/ApolloAuto/apollo/releases/tag/v6.0.0)

Extract it into the existing `apollo` folder. Since the `apollo` folder already contains some files, you should do extraction without replacement

### Building the Apollo

Ok, now we have everything we need. Time to build some code

Apollo uses Docker containers. So, make sure you have installed Docker. If you need additional instructions on Docker installatioin, check out [this](https://github.com/ApolloAuto/apollo/blob/master/docs/specs/prerequisite_software_installation_guide.md) link

Now we need to perform the following steps:

- `cd` into the `apollo` folder from the terminal
-  Create the Docker container with `bash docker/scripts/dev_start.sh -f`. It will pull the Apollo image and make some preparations. The process will take some time.
- Get inside the container with `bash docker/scripts/dev_into.sh`
- Build the apollo with `./apollo.sh clean && ./apollo.sh build`.You can use`./apollo.sh build_gpu` if you have GPU installed on your system. Other build options are explained [here](https://github.com/ApolloAuto/apollo/blob/master/docs/specs/apollo_build_and_test_explained.md). The build will take a while (mine took about 1 hour on an i5  CPU). Time to grab some tea and relax

If everything goes smooth, we're ready to launch the system

## Launch steps

### Simulator launch and setup

`cd` into the `simulator` folder and launch the simulator with `./InnoSimulator.x86_64`

Run the scene with **Launch** button in the bottom right corner

This should spawn the car and show you this configuration panel:

![Simulator configuration panel](images/simulator_configuration.png "Configuration panel")

You need to turn these modules on:
- **IMU**
- **GPS**
- **Perception obstacles**

### Apollo Dreamview launch
- `cd` into the `apollo` folder and get inside the Docker container with `bash docker/scripts/dev_into.sh` if you are not already there
-  From within the container run the system with `./scripts/bootstrap.sh start`. You can stop the system with `./scripts/bootstrap.sh stop`
- Within the container run the Cyber Bridge with `./scripts/bridge.sh`
- You can check if messages from the Simulator are arriving with `cyber_monitor`
- Go to `http://localhost:8888/` to open Dreamview

### Dreamview setup
On the Dreamview web page, in the top right corner select:
- **Santa Fe Simulator** setup mode
- **Hundai Santa Fe** vehicle
- **Innopolis** map

The result should look like this:

![Dreamview setup](images/dreamview_setup.png "Dreamview setup")

Now we can launch autonomous driving modules. Press the setup button. It will try to launch all modules that are defined in the setup configuration file. Here is how the button looks like:

![Setup button](images/setup_button.png "Setup button")

 If it is not active, unlock it:
 
![Unlock button](images/unlock_button.png "Unlock button")

Check the situation on the **Module Controller** tab. Turn on grayed out module buttons if necessary. It should look like this:
 
![Modules state](images/modules_state.png "Modules state")

Now you can send a target point to the vehicle. In order to do that, open **Route Editing** tab, create a target point by clicking on the desired location, and press **Send Routing Request** button.

![Routing request](images/routing_request.png "Routing request")

Car should autonomously reach the target point. Cool, you can enjoy the simulator now!

## List of useful commands:
- `bash docker/scripts/dev_start.sh -f` - to create Docker container
- `bash docker/scripts/dev_into.sh` - to get into the Docker container
- `./scripts/bootstrap.sh <start or stop or restart>` - to start/stop/restart Dreamview
-  `bash ./docker/scripts/dev_start.sh stop` - to stop all Apollo Docker containers

Note, that starting from Apollo 5.0, Apollo uses CyberRT, not ROS. So, you will have to use CyberRT. The good thing is, CyberRT tools are very much like ROS tools. 

- `cyber_monitor` - interactive topic monitoring
- `cyber_visualizer` - topic data visualization. Like `rqt`
- `cyber_channel list` - to see all available topics. Like `rostopic list`
- `cyber_launch start path/to/launch/file.launch` - to launch your module. Like `roslaunch package_name file.launch`
- `mainboard -d path/to/dag/file.dag` - to launch dag file
 
## Some tips
- Apollo creates huge log files. If you see no free space on your hard drive, consider cleaning the `apollo/data/` folder.
-  Apollo uses the Bazel build system. So, if you see `BUILD` files - it is Bazel. You can get more information about it [here](https://www.bazel.build/).
- Apollo uses its own runtime framework CyberRT. Basically, it is customized ROS. Same concepts. You can check out [this](https://cyber-rt.readthedocs.io/en/latest/CyberRT_Python_API.html) link to get how to write Reader (Subscriber in ROS) and Writer (Publisher in ROS) in Python.
- If you modify config files - restart Apollo and Dreamview with `./scripts/bootstrap.sh stop && ./scripts/bootstrap.sh start`
- If you modify protobufs or source code - rebuild with `./apollo.sh build`. You can also rebuild particular modules with `./apollo.sh build <module_name>`
