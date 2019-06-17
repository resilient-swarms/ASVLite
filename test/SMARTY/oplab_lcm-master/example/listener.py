import lcm

from exlcm import example_t

def my_handler(channel, data):
    msg = example_t.decode(data)
    print("Received message on channel \"%s\"" % channel)
    print("   timestamp   = %s" % str(msg.timestamp))
    print("   position    = %s" % str(msg.position))
    print("   orientation = %s" % str(msg.orientation))
    print("   ranges: %s" % str(msg.ranges))
    print("   name        = '%s'" % msg.name)
    print("   enabled     = %s" % str(msg.enabled))
    print("")

# TODO: Check if we really need explicit URL definition when running in localhost (also applies for send_message.py)
lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")

# Modified channel to VECTOR_NODE_ASV
subscription = lc.subscribe("VECTOR_NODE_ASV", my_handler)

try:
    while True:
        lc.handle()
except KeyboardInterrupt:
    pass

lc.unsubscribe(subscription)
