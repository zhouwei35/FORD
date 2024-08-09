# LGC: A Local and Global Combined Descriptor for Large-Scale Forest Place Recognition


 <img src="https://github.com/jiurobots/JORD/blob/main/photos.jpg" width="1000" />

## Introduction

Achieving accurate robot place recognition in large-scale forest environments is a critical task. Due to the extensive occlusion areas, repetitive terrain features, and limited remote communication in forests, the difficulty of achieving place recognition increases significantly. To address this problem, we propose a novel two-stage place recognition method, referred to as LGC, which combines local and global descriptors. In the coarse matching stage, we first design a trunk contour point cloud segmentation method based on spatial and planar clustering, and further extract trunk position coordinates. Then, a local triangle descriptor (LTD) is constructed  based on these coordinates. This descriptor correlates the structural relationships between local real-world trees and is viewpoint invariant. Next, we build a hash database and obtain a candidate frame set through the voting strategy. Finally, geometric consistency verification is performed on the candidate frames. In the fine matching stage, a global trunk descriptor (GTD) based on trunk distribution quantity information is proposed to further enhance the matching process. This method comprehensively considers the local geometric and global appearance features available in forest scenes, enabling accurate place recognition in large-scale forest scenes. Comparative verification is conducted using existing public unstructured and real-world forest datasets, comparing LGC with other advanced methods. Results show that LGC exhibits better adaptability and significant improvements in accuracy in forest scenes, even under challenging conditions with substantial viewpoint changes.

<p align="center">
  <img src="https://github.com/jiurobots/JORD/blob/main/jord_gif/1.gif" width="200" />
  <img src="https://github.com/jiurobots/JORD/blob/main/jord_gif/2.gif" width="200" />



  <img src="https://github.com/jiurobots/JORD/blob/main/jord_gif/3.gif" width="200" />
  <img src="https://github.com/jiurobots/JORD/blob/main/jord_gif/4.gif" width="200" />
</p>



## Recording Platform

| Sensor            | Model            | Rate | Specifications |
| ----------------- | ---------------- | ---- | -------------- |
| Mechanical LiDAR  | Velodyne VLP-16  | 10   | 16 Channels    |
| Solid-state LiDAR | Livox HAP        | 10   | 120×25         |
| IMU               | EG320N           | 125  | 6DoF           |
| GPS               | NovAtel Npos220s | 10   | Dual Antenna   |

 <img src="https://github.com/jiurobots/JORD/blob/main/img/sensors.jpg" width="800" />

 ## Recording Collection

<img src="https://github.com/jiurobots/JORD/blob/main/jord_gif/5.gif" width="400" />


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
链接: https://pan.baidu.com/s/1gr-3rPIEXNY0aFRnyaf1EQ?pwd=npc2 提取码: npc2 


## Folder Structure

<img src="https://github.com/jiurobots/JORD/blob/main/img/FileTree.jpg" width="200" />

## Code

Our paper is currently Undergoing Review, and our code will be released once the paper is accepted.

## License 

All datasets and code on this page are copyright by us and published under the Creative Commons Attribution-NonCommercial-ShareAlike 3.0 License.



 
