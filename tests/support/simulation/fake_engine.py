"""Deterministic unit-mass engine, test-only; never loads simulator libraries."""
import math
from simulation.engine import Feedback


class FakeEngine:
    def initialize(self, config):
        self.timestep_s=config.get('physics_dt_us',1000)/1000000
        self.q=tuple(config.get('q',(0.,)))
        self.dq=(0.,)*len(self.q)
        self.tau=(0.,)*len(self.q)
        self.physics_step=0
        self.closed=False
        self.released=False

    def read_feedback(self):
        if self.closed: raise RuntimeError('closed engine')
        return Feedback(self.physics_step,self.physics_step*self.timestep_s,
                        self.q,self.dq,self.tau,(1.,0.,0.,0.),(0.,)*3,
                        (0.,0.,9.81),(0.,)*3,(0.,)*6,(1.,0.,0.,0.))

    def apply_torque(self, torque):
        if len(torque)!=len(self.q) or not all(math.isfinite(v) for v in torque):
            raise ValueError('torque')
        self.tau=tuple(torque)

    def step(self, substeps):
        if type(substeps) is not int or substeps<=0: raise ValueError('substeps')
        for _ in range(substeps):
            self.dq=tuple(v+t*self.timestep_s for v,t in zip(self.dq,self.tau))
            self.q=tuple(q+v*self.timestep_s for q,v in zip(self.q,self.dq))
            self.physics_step+=1
        return self.read_feedback()

    def release_base(self):
        self.released=True

    def close(self):
        self.closed=True
