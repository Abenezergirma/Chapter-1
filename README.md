# Chapter-1

## Overview of the Repository:

This repository contains the code and results from the first chapter of my dissertation on **Urban Air Mobility (UAM)** trajectory planning. The goal is to provide a real-time planning framework that's both safe and scalable.

### Key Features:

1. **Decentralized Free-Flight Concept**: 
   - Every aircraft independently handles conflict resolution and ensures safety by considering the predicted positions of nearby planes.
   
2. **Two-Part Framework**:
   - **Reachability Analysis Tool**: Predicts potential aircraft future states using data from simulated trajectories.
   - **Decision Maker**: Uses a Markov Decision Process and 6-degree freedom model for fixed-wing aircraft to plan collision-free paths.

3. **Safety Enhancements**: 
   - Incorporates techniques like reward shaping and action shielding to boost the safety of the planner.
   
4. **Performance Testing**: 
   - Simulated tests with up to 32 aircraft showed the effectiveness of this framework, measured by reduced Near Mid Air Collisions (NMAC) and computational efficiency.

# Illustrations

# Images
<table>
  <tr>
    <td><img src="img/state_x.png" alt="Image 1"></td>
    <td><img src="img/xy_proj.png" alt="Image 2"></td>
  </tr>
  <tr>
    <td><img src="img/state_y.png" alt="Image 3"></td>
    <td><img src="img/xz_proj.png" alt="Image 4"></td>
  </tr>
  <tr>
    <td><img src="img/state_z.png" alt="Image 5"></td>
    <td><img src="img/yz_proj.png" alt="Image 6"></td>
  </tr>
</table>

![Snopshot of the simulation environment](img/frame_116.png)

# Videos

- [Video 1](https://youtu.be/7JmT2nph9Ws)
- [Video 2](https://youtu.be/vlUZZLl_uGY)
- [Video 3](https://youtu.be/C8PxhUiaKJE)





### Code Overview:

To ensure a seamless reproduction of the results discussed in the chapter, follow the guide below:

**Repository Structure**:
- The repository is organized into four distinct branches:
  1. `main`: Contains all the techniques discussed in the chapter.
  2. `baseline`: Implements the baseline technique discussed in the chapter.
  3. `reward_shaping`: Implementation of the reward shaping technique.
  4. `action_shielding`: Implementation of the action shielding technique.

**Steps to Run the Code**:
1. Navigate to the `main.m` script.
2. Configure the desired number of agents you wish to simulate.
3. Execute the script.
4. Once completed, collect key flight statistics, including NMAC and Computation time.
5. Save your environment as a `.mat` file.
6. To visualize the results, import the saved `.mat` file into the `animate.m` script and run it.
