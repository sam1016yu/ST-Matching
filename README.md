# ST-Matching
MATLAB implementation of [ST-Matching ](https://www.microsoft.com/en-us/research/publication/map-matching-for-low-sampling-rate-gps-trajectories/) Algorithm for map matching problem.
## Paper Abstract

> Map-matching is the process of aligning a sequence of observed user positions with the road network on a digital map. It is a fundamental pre-processing step for many applications, such as moving object management, traffic flow analysis, and driving directions. In practice there exists huge amount of low-sampling-rate (e.g., one point every 2-5 minutes) GPS trajectories. Unfortunately, most current map-matching approaches only deal with high-sampling-rate (typically one point every 10-30s) GPS data, and become less effective for low-sampling-rate points as the uncertainty in data increases. In this paper, we propose a novel global map-matching algorithm called ST-Matching for low-sampling-rate GPS trajectories.ST-Matching considers (1) the spatial geometric and topological structures of the road network and (2) the temporal/speed constraints of the trajectories. Based on spatio-temporal analysis, a candidate graph is constructed from which the best matching path sequence is identified. We compare ST-Matching with the incremental algorithm and Average-Fréchet-Distance (AFD) based global map-matching algorithm. The experiments are performed both on synthetic and real dataset. The results show that our ST-matching algorithm significantly outperform incremental algorithm in terms of matching accuracy for low-sampling trajectories. Meanwhile, when compared with AFD-based global algorithm, ST-Matching also improves accuracy as well as running time.

## Algorithm pseudocode
![Algorithm 1,whole structore](
        st-matching.PNG
      )
![Algorithm 2,find matched sequence](
        findMatchedSequence.PNG
      )
## Implementation Steps
### Entry point:
st-matching.m
### Data preparation:
There are two sets of input files:
- road network file (containing information of edges and nodes, usually from Open Street Map)
- gps trajectory files

Format of **road network** file:

EdgeID | Node1ID | Node2ID | Longitude of Node1 | Latitude of Node1 |  Longitude of Node2 | Latitude of Node2 | Road Type
---|--- |--- |--- |--- |--- | -- | --

Format of **gps trajactory** files:

GPS Device ID | Timestamp | Longitude | Latitude | Speed (km/h) | Direction
---|--- |--- |--- |--- |--- 

### Step1: split road into smaller cells
define length of each cell(in km),then call ***splitRoadtoCell***

code sample:
```
roadnetworkfilename = 'RoadNetwork_Beijing.txt';
cell_size = 0.1;
[road_network,speed_limits,road_cells,road_ids,grid_size] = splitRoad2Cell(roadnetworkfilename, cell_size);
```
output of this code include 
- a matrix of raw road network ***road_network*** (*m* x *7*,same as input road network file except last column)
- a vector of speed limist of each roads ***speed_limits*** translated from road type (last column of input road network)
- information of each cell ***road_cells*** (*m* x *5*)

cellID | startLon | endLon | startLat | endLat
---|---|---|---|---
- size of grid ***grid_size*** (*m* x *n* cells from entire road network)

### Step2: split raw gps trajactory into smaller segments
function: **splitGPS2line**

Splitting criteria: 
- assume a standard sampling interval $\Deltat_n$  and a standard length for each trajectory segment ***l***
- the length of a trajectory should be less or equal than ***l***
- $sum \Delta t <= 5\Deltat_n*l$
- cut as few trajectory as possible

code sample:

```
gpsfilename = 'GPS_Beijing.txt';
raw_gps_points = splitGPS2line(gpsfilename, 6, 5);
```

output result will be added as a tag additional to the original gps trajectory file, so that the format of ***raw_gps_points*** (*m* x *7*) will be

GPS Device ID | Timestamp | Longitude | Latitude | Speed (km/h) | Direction | Tag
---|--- |--- |--- |--- |--- | ---

### Step3: get candidate points of each trajectory

With the cutted road network, we first find candidate edges around raw sample points, then project sample points to that edge. (for speed consideration, only the top 5 closest candidate edges are considered) 

- Script for this step can be seen in function ***getCandidatePoints***.
- Line projection is vector based and can be seen in the sub-function ***project2Line***

### Step4: cut the road network for specific trajectory
The entire road_network obtained in step 1 is considered too big for an individual trajectory, hence we implement a cutting on the original big road network. 
- script for this step can be seen in function ***cutGridforTrajactory***
- The cutted network contains all the **candidate edges** of trajectory points.
- a graph ***G*** is constructed for path search in the next step
- a table ***node_table*** recording relationship between original **NodeID** and actual node ID used in graph ***G*** is generated

### Step5: find matched sequence
see ***findMatchedSequence*** above for general procedure of this step.
both **spatial(transmission * observation)** and **temporal** probability are calculated between candidate points.
#### post-processing
In our experiment, we find some issue particularly at crossing roads. This is due to the imbalance of two probabilities. For example, when transmission probability is very close, observation probability become the dominant factor. We did a potential fix for simple crossing error.
### Step6: validation
see ***validation.m*** and picture in the **result** folder for detail. Sample points are plotted in blue points, while the matched points are plotted in red cross. The path between matched points are also plotted. 