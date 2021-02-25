# Scalable-Framework-Cooperative-Localization

This repository contains the official Matlab implementation for our paper submitted to RA-L and IROS 2021 with title "A Scalable Framework for Map Matching based Cooperative Localization"

<img align="center" src="https://github.com/wvu-irl/Scalable-Framework-Cooperative-Localization/blob/docs/overall_approach_thick_borders.pdf">

## Overview
- [News](#news)
- [Introduction](#dependencies)
- [Implementation](#implementation)
- [Acknowledgement](#acknowledgement)

## News
- Feb. 24, 2021 Paper submitted to RA-L with IROS 2021 option

## Introduction
Localization based on map matching (e.g., using gravity anomaly, magnetic anomaly, topographics, or olfaction maps) is a potential solution for navigating Global Navigation Satellite System (GNSS)-denied environments. In this paper, a scalable framework is presented for cooperatively localizing a group of agents based on map matching given a prior map modeling the scalar field. In order to satisfy the communication constraints, each agent in the group is assigned to different subgroups. A locally centralized cooperative localization method is performed in each subgroup to estimate the poses and covariances of all agents inside the subgroup. Each agent in the group, at the same time, could belong to multiple subgroups, which means multiple pose and covariance estimates from different subgroups exist for each agent. The improved pose estimate for each agent at each time step is then solved through an information fusion algorithm. The proposed algorithm is evaluated with two different types of scalar fields based simulations. The simulation results show that the proposed algorithm is able to deal with large group sizes (e.g., 128 agents), achieve 10-meter level localization performance with 180 km traveling distance, while under restrictive communication constraints.

## Implementation


## Acknoledgement

