# Scalable-Framework-Cooperative-Localization

This repository contains the official Matlab implementation for our paper submitted to Sensor with title "A Scalable Framework for Map Matching based Cooperative Localization".

<img align="center" src="https://github.com/wvu-irl/Scalable-Framework-Cooperative-Localization/blob/main/docs/overall_approach.png">
Illustration of the proposed framework for map matching based cooperative localization. (Left) A large group of agents is divided in subgroups based on communication constraints where one subgroup is created for each agent. (Upper Right) The geometry of the subgroups (i.e., the relative positions) are estimated using range-only measurements, then the geometry is used to extract measurements of the scalar field to estimate the pose and associated uncertainty. (Lower Right) An agent receives multiple copies of its pose estimate through its membership in several subgroups and fuses them to reduce pose error.

## Overview
- [News](#news)
- [Introduction](#dependencies)
- [Implementation](#implementation)
- [Acknowledgement](#acknowledgement)

## News
- July. 18, 2021 Paper submitted to Sensor with Special Issue on "Sensor Fusion for Vehicles Navigation and Robotic Systems"

## Introduction
Localization based on scalar field map matching (e.g., using gravity anomaly, magnetic anomaly, topographics, or olfaction maps) is a potential solution for navigating Global Navigation Satellite System (GNSS)-denied environments. In this paper, a scalable framework is presented for cooperatively localizing a group of agents based on map matching given a prior map modeling the scalar field. In order to satisfy the communication constraints, each agent in the group is assigned to different subgroups. A locally centralized cooperative localization method is performed in each subgroup to estimate the poses and covariances of all agents inside the subgroup. Each agent in the group, at the same time, could belong to multiple subgroups, which means multiple pose and covariance estimates from different subgroups exist for each agent. The improved pose estimate for each agent at each time step is then solved through an information fusion algorithm. The proposed algorithm is evaluated with two different types of scalar fields based simulations. The simulation results show that the proposed algorithm is able to deal with large group sizes (e.g., 128 agents), achieve 10-meter level localization performance with 180 km traveling distance, while under restrictive communication constraints.

## Implementation

### Pre-requirements
This code was tested with MATLAB 2020b. The following toolboxs are required with the code:

- Parallel Computing Toolbox
- Control System Toolbox

### Simulation with Magnetic anomaly map
#### Plot map
- run plot_magnetic_map.m under folder magnetic_maps/

#### Perform the proposed algorithm (Once)
- set parameters in main.m under folder Proposed_algorithm_mag_map/
- run main.m under folder Proposed_algorithm_mag_map/

#### Perform the proposed algorithm (Monte Carlo Simulations)
- set parameters in monte_carlo_parallel.m under folder Proposed_algorithm_mag_map/
- run monte_carlo_parallel.m under folder Proposed_algorithm_mag_map/

#### Perform the full communication case
- set parameters in main.m under folder Full_communication_mag_map/
- run main.m under folder Full_communication_mag_map/
- set parameters in monte_carlo_parallel.m under folder Full_communication_mag_map/
- run monte_carlo_parallel.m under folder Full_communication_mag_map/


### Simulation with Bathymetric map
#### Plot map
- download map data at: [Google Drive link](https://drive.google.com/file/d/14npOMaTV6z6uZIB4Uet7j0KISWl9a9CN/view?usp=sharing)
- copy the chancen_bath.asc file to the folder bathymetric_map/
- run plot_bathymetric_map.m under folder bathymetric_map/

#### Perform the proposed algorithm (Once)
- set parameters in main.m under folder Proposed_algorithm_bathy_map
- run main.m under folder Proposed_algorithm_mag_map/

#### Perform the proposed algorithm (Monte Carlo Simulations)
- set parameters in monte_carlo_parallel.m under folder Proposed_algorithm_bathy_map/
- run monte_carlo_parallel.m under folder Proposed_algorithm_bathy_map/

#### Perform the full communication case

- set parameters in main.m under folder Full_coomunication_bathy_map/
- run main.m under folder Full_communication_bathy_map/
- set parameters in monte_carlo_parallel.m under folder Full_communication_bathy_map/
- run monte_carlo_parallel.m under folder Full_communication_bathy_map/

## Acknowledgement
Thanks for the scalar field map data provided by [U.S. Geological Survey](https://www.usgs.gov/).

