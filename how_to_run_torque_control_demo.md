Below are instructions for running Joint Torque Demo, script attached for reference:

SSH into G2 (LAN):

ssh agi@10.42.1.101

Password: 1


Enable root user:

sudo su

Password: 1


Move to script directory:

cd app/gdk/examples/python

Source environment:

source /home/agi/app/env.sh

Export required dependencies:
export COSINE_BUS_CONFIG_PATH=/home/agi/app/conf/dds/cosine_ipc.prototxt
export COSINE_BUS_ENABLE_TOPIC_CONFIG_MERGE=1

Run script with the confirmed URDF, genie_g2_crs_omnipicker.urdf. It ships with this repo in
g2_torque_subscriber/urdf/ and reaches the robot with the repo copy (README, "Run on the robot's
computer"):

python3 joint_torque.py /home/agi/g2_torque/g2_torque_subscriber/urdf/genie_g2_crs_omnipicker.urdf

The vendor's original command used
/home/agi/app/share/genie_robot_description/urdf/G2_t2_crsB/G2_t2_crsB.urdf, which is NOT the
right model for this robot.
