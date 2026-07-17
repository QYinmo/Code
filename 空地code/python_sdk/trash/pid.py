import simple_pid
class pid():
    def __init__(self,target) -> None:
        self.p=0.1
        self.i=0.002
        self.d=0.00
        self.limit=15
        self.target=target
        self.xpid=simple_pid.PID(self.p,self.i,self.d,self.target[0])
        self.xpid.output_limits=(-self.limit,self.limit)
        self.ypid=simple_pid.PID(self.p,self.i,self.d,self.target[1])
        self.ypid.output_limits=(-self.limit,self.limit)
    def get_cv_pid(self,current:list):
            return[int(self.ypid(current[1])),int(self.xpid(current[0]))]