#!/usr/bin/env python3

import rosbag

if __name__ == "__main__":
    bag = rosbag.Bag('base.bag')
    for (topic,msg,t) in bag.read_messages():
        print (topic,msg,t)
