import lcm
import time

from exlcm import example_t

# This might not be needed when runnning in localhost
lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")

msg = example_t()
msg.timestamp = int(time.time() * 1000000)
msg.position = (1, 2, 3)
msg.orientation = (1, 0, 0, 0)
msg.ranges = range(15)
msg.num_ranges = len(msg.ranges)
msg.name = "example string"
msg.enabled = True

# Publish msg in the channel VECTOR_NODE_ASV
lc.publish("VECTOR_NODE_ASV", msg.encode())
