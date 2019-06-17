import lcm,sys

from exlcm import example_t

time = 0

class proc_lcm(object):
    def __init__(self):
        self.lc = lcm.LCM()
        self.sub = self.lc.subscribe("EXAMPLE",self.proc_callback)
    
    def proc_callback(self, channel, data):
        msg = example_t.decode(data)
        global time
        time = msg.timestamp

        
    def do_work(self):
        ok=0
        while ok==0:
            ok=self.lc.handle_timeout(1000000)
        
        print time
        self.lc.unsubscribe(self.sub)

if __name__=="__main__":
    
    proc=proc_lcm()
    proc.do_work()
    
    sys.exit(0)
        
