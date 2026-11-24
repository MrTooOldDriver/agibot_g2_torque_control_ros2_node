#!/usr/bin/env python3
"""Entry point of the Python G2 torque subscriber (see g2_torque_subscriber/torque_subscriber_node.py)."""

import sys

from g2_torque_subscriber.torque_subscriber_node import main

if __name__ == "__main__":
    sys.exit(main(sys.argv))
