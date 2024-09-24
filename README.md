# FORD Dataset


 <img src="https://img520.com/HFd7BP.jpg" width="1000" />


<p align="center">
  <img src="https://github.com/zhouwei1995/Gift/blob/main/1.gif" width="200" />
  <img src="https://github.com/zhouwei1995/Gift/blob/main/2.gif" width="200" />



  <img src="https://github.com/zhouwei1995/Gift/blob/main/3.gif" width="200" />
  <img src="https://github.com/zhouwei1995/Gift/blob/main/4.gif" width="200" />
</p>



## Recording Platform

| Sensor            | Model            | Rate | Specifications |
| ----------------- | ---------------- | ---- | -------------- |
| Mechanical LiDAR  | Velodyne VLP-16  | 10   | 16 Channels    |
| Solid-state LiDAR | Livox HAP        | 10   | 120×25         |
| IMU               | EG320N           | 125  | 6DoF           |
| GPS               | NovAtel Npos220s | 10   | Dual Antenna   |

 <img src="https://img520.com/XkIJA3.jpg" width="800" />

 ## Recording Collection

<img src="https://github.com/zhouwei1995/Gift/blob/main/5.gif" width="400" />


## Dataset Description  

| Sequence | Path Length(m) | Frame | Avg Speed(m/s) | Revisits/Total | Initial Frames      | Loop Frames         | Non-same Direction | Revisits (ratio) | Description                                    |
| -------- | -------------- | ----- | -------------- | -------------- | ------------------- | ------------------- | ------------------ | ---------------- | ---------------------------------------------- |
| FORD01   | 374.4          | 3508  | 0.99           | 1018/3508      | 798-1868,534-683    | 2168-3080,3139-3243 | Y                  | (89%,913/1018)   | Dense forest, Narrow and rough path            |
| FORD02   | 872.3          | 8216  | 1.0            | 3360/8216      | 2634-2796,4152-7329 | 2934-3096,233-3914  | Y                  | (86%,2897/3360)  | Dense forest, Narrow path, Tangled undergrowth |
| FORD03   | 654.2          | 5444  | 1.14           | 739/5444       | 0-960               | 4706-5444           | Y                  | (100%,739/739)   | Sparse forest, Wide Muddy path                 |
| FORD04   | 592.5          | 4120  | 1.34           | 762/4210       | 1175-1899,869-954   | 2119-2775,3014-3118 | Y                  | (89%,913/1018)   | Sparse forests,Wide path, Slopes               |

## Dataset Download  

The FORD can be download as follows:

<p>
链接：https://pan.baidu.com/s/1Ya0CGdfkSDmwZln-NxcyLw?pwd=s5kn 
提取码：s5kn 

## Code
#### Dependencies
The following has been verified to be compatible, although other configurations may work too:
* Ubuntu 18.04
* ROS Noetic(roscpp, rospy, std_msgs, sensor_msgs, pcl_ros)
* C++ 17
* CMake >= 3.0.2
* OpenCV >= 4.2.0
* PCL >= 1.7
* Eigen3 >= 3.3.7
* Boost >= 1.71.0
```
sudo apt install liboost-dev libpcl-dev libeigen3-dev
```
#### Compiling

```
mkdir ws && cd ws && mkdir src && cd src
git clone https://github.com/jiurobots/JORD.git
catkin make
```
#### Execution
```
./LGC
```
## License 

All datasets and code on this page are copyright by us and published under the Creative Commons Attribution-NonCommercial-ShareAlike 3.0 License.



 
