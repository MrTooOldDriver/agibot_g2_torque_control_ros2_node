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

Run script:

python3 joint_torque.py /home/agi/app/share/genie_robot_description/urdf/G2_t2_crsB/G2_t2_crsB.urdf