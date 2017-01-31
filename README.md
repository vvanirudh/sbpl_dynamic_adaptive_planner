# Path Planning in Dynamic Environments with Adaptive Dimensionality

This repository maintains the code for the [paper](http://arxiv.org/pdf/1605.06853v1.pdf) **Path planning in Dynamic Environments with Adaptive Dimensionality** published in the proceedings of the International Symposium on Combinatorial Search (SoCS) 2016. If you use this code, please cite the paper as follows:

`@inproceedings{vemula2016path,
  title={Path Planning in Dynamic Environments with Adaptive Dimensionality},
  author={Vemula, Anirudh and Muelling, Katharina and Oh, Jean},
  booktitle={Ninth Annual Symposium on Combinatorial Search},
  year={2016}
}`


## How to use this code

### Dependencies
1. ROS Indigo (with catkin, std_msgs, tf packages installed)
2. sbpl package (installation instructions can be found [here](https://github.com/sbpl/sbpl))

### Building
1. Clone the repo into your catkin workspace
2. Go to the catkin workspace root directory and then run `catkin_make`

### Running the executable
1. The executable can be run using `rosrun sbpl_dynamic_adaptive_planner test_ad_exact [env_file] [motprim_file] [dynObsFile]`
2. For example, in the root directory it can be run as `rosrun sbpl_dynamic_adaptive_planner test_ad_exact env/test1_indoor.exp env/unicycle_noturninplace.mprim env/dynObs1_indoor_30.dob`
3. You can visualize the resulting path and the environment by running RViz using the `rviz/visualization.rviz` config file


**Maintained by** : Anirudh Vemula
