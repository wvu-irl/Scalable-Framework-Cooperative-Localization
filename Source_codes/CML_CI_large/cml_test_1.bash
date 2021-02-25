#!/bin/bash
 
# Name of job:
#PBS -N cml_1
 
# Where to write stderr:
#PBS -e cml_1.err
 
# Where to write stdout: 
#PBS -o cml_1.out
 
# Specify number of nodes, processors (really threads) per node, and the maximum allowed run time for the job
# Can also specify max memory requested with something like mem=10gb
#PBS-l nodes=1:ppn=80,walltime=36:00:00

# Keep job output and joint output and error
#PBS -k o
#PBS -j oe

# Change directory to the directory the job was submitted from
cd $PBS_O_WORKDIR

# Run the program
matlab -r monte_carlo_parallel
