# arbocBrain

>  With an aim of tasking the robot with gait generation and goal based locomotion, we have buffed up a Beaglebone Blue with numerous packages that aid in both server-client communication and client-side computation.


## Installation

BeagleBone® Blue is an all-in-one Linux-based computer for robotics which comes preloaded with a Debian distribution (Debian jessie) which can be upgraded to a latest firmware from https://beagleboard.org/latest-images. Since most of the computation and processing is done with ROS on the server, it becomes crucial that the client has an active installation of ROS with all the necessary packages. Though ROS distros are available for Debian, its a bit finicky when it comes to installation and support. So we flashed the Beaglebone Blue with a custom ubuntu image (ubuntu 16.04) which we found at https://jh.app.box.com/v/530-707-Dropbox, which is a pre-built image courtesy of Robert Nelson, Jeff O’Brian, and louis whitcomb that comes with ROS Kinetic pre-installed. The entire process has been explained further with much detail below.


#### Expanding file system partition:
The image mentioned above is a 8GB ubuntu-16.04 image which converts the attached SD card to a file system of 8GB capacity irrespective of its native capacity. Make sure to expand the file system partition before proceeding further.

Refer to https://dev.iachieved.it/iachievedit/expanding-your-beaglebone-microsd-filesystem/.

![alt text](https://raw.githubusercontent.com/imsenthur/Arboc/master/arbocBrain/arbocSlave.png)

#### ROS network configuration requirements:
ROS has certain requirements for the network configuration:

- There must be complete, bi-directional connectivity between all pairs of machines, on all ports. 
- Each machine must advertise itself by a name that all other machines can resolve. 

To check basic connectivity, ping each machine from itself,
```html
  ~ssh arboc.slave
  ~ping arboc.slave
```
and also ping between machines,
```html
  ~ssh arboc.slave
  ~ping arboc.master
```
you should see something like this,
```html
  PING arboc.master: 56 data bytes
  64 bytes from 192.168.1.1: icmp_seq=0 ttl=63 time=1.868 ms
  64 bytes from 192.168.1.1: icmp_seq=1 ttl=63 time=2.677 ms
  64 bytes from 192.168.1.1: icmp_seq=2 ttl=63 time=1.659 ms
```
#### ROS network setup:
Set your ROS environment variables on both master (server) and the slave (client). You can perform this by editing your bashrc script to include "ROS_MASTER_URI" and "ROS_HOSTNAME" which are crucial when it comes to networking with ROS.
```html
  ~sudo subl ~/.bashrc
```
and add these lines to the end of script,
```html
  export ROS_MASTER_URI=http://arboc.master:11311
  export ROS_HOSTNAME=arboc.master
```
and source it.
```html
  ~source ~/.bashrc
```
Make sure you perform the above mentioned steps on the Beaglebone too (but with ROS_HOSTNAME=arboc.slave).
Here "arboc.master" and "arboc.slave" corresponds to the IP address of the master(PC/server) and slave(Beaglebone Blue) respectively. You can add these entries to your "/etc/hosts" to save the hassle of typing the entire IP address everytime you login into your device and it also enables multiple devices to identify each other. The hosts file tells each device how to convert specific names into an IP address.

Once the environment variables are set you must be able to get the following output, 
For the master (PC/server),
```html
  ~echo $ROS_MASTER_URI
  http://arboc.master:11311
  ~echo $ROS_HOSTNAME
  arboc.master
```
and for the slave (Beaglebone Blue),

```html
  ~echo $ROS_MASTER_URI
  http://arboc.master:11311
  ~echo $ROS_HOSTNAME
  arboc.slave
```
We are assuming that "roscore" is being run only on the master and the slave subscribes to multiple topics that are being published. 

#### Time synchronization:
You may have a discrepancy in system times between the master and slave. You can check it from the slave(Beaglebone Blue) using,
```html
  ~ssh arboc.slave
  ~ntpdate -q arboc.master
```
If there is a discrepancy, install chrony and ntpdate by running,
```html
  sudo apt-get install chrony ntpdate
```
Once the installation is successful, make sure you disable "automatic time update" so that the system time doesnot get changed during execution. In order to do disable it, run the following command,
```html
  timedatectl set-ntp false
```
Then configure chrony to query the server's system time, and slowly get the time in-sync. In order to do that, just open the file "/etc/chrony/chrony.conf" in your text editor and add these lines to it,
```html
  server arboc.master minpoll 0 maxpoll 5 maxdelay .05
```
Finally, make sure that the timezones between the master and slave are consistent. If they're not the same then you can change it using,
```html
  sudo timedatectl set-timezone Asia/Kolkata
```
Note that the above command changes the timezone to IST(Indian Standard Time).

![alt text](https://raw.githubusercontent.com/imsenthur/Arboc/master/arbocBrain/arbocDiagnostics.png)
#### Diagnostics:
To start with, query the current time offset by running,
```html
  ~ssh arboc.slave
  ~sudo ntpdate -q arboc.master
```
If they are at perfect sync, you're good to go.
Start the roscore on master (PC/server) and run a publisher node,
```html
  ~roscore
  ~rostopic pub -r 10 /hello std_msgs/String "Hello"
```
Now ssh onto your slave and list the topics being published,
```html
  ~ssh arboc.slave
  ~rostopic list
```
You must be able to see a list of all the topics that are being published,
```html
  /hello
  /rosout
  ...
```
Now try echoing the published topic,
```html
  ~rostopic echo /hello
```
you should be getting "hello"s on the output terminal at a rate of 10Hz.
![alt text](https://raw.githubusercontent.com/imsenthur/Arboc/master/arbocBrain/arbocNetwork.png)

[Back To The Top](#arbocBrain)
