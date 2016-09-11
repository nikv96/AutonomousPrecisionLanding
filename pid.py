'''

    Synopsis: PID controller code
    Author: Daniel Nugent
    Code Available at https://github.com/djnugent/Precland/blob/master/Common/PID.py used under the GNU license

'''

#Python Imports
import math
import time

class pid(object):

    def __init__(self, initial_p=0, initial_i=0, initial_d=0, initial_imax=0):
        self.p_gain = initial_p
        self.i_gain = initial_i
        self.d_gain = initial_d
        self.imax = abs(initial_imax)
        self.integrator = 0
        self.last_error = None
        self.last_update = time.time() 

    def __str__(self):
        return "P:%s,I:%s,D:%s,IMAX:%s,Integrator:%s" % (self.p_gain, self.i_gain, self.d_gain, self.imax, self.integrator)

    def get_dt(self, max_dt):
        now = time.time()
        time_diff = now - self.last_update
        self.last_update = now
        if time_diff > max_dt:
            return 0.0
        else:
            return time_diff

    def get_p(self, error):
        return self.p_gain * error

    def get_i(self, error, dt):
        self.integrator = self.integrator + error * self.i_gain * dt
        self.integrator = min(self.integrator, self.imax)
        self.integrator = max(self.integrator, -self.imax)
        return self.integrator

    def get_d(self, error, dt):
        if self.last_error is None:
            self.last_error = error
        ret = (error - self.last_error) * self.d_gain * dt
        self.last_error = error
        return ret

    def get_pi(self, error, dt):
        return self.get_p(error) + self.get_i(error,dt)

    def get_pid(self, error, dt):
        return self.get_p(error) + self.get_i(error,dt) + self.get_d(error, dt)

    def get_integrator(self):
        return self.integrator

    def reset_I(self):
        self.integrator = 0

    def main(self):
        print "Test PID: %s" % test_pid
    	result = 1000000000000
    	while not (result > -2.220446049250313e-16 and result < 2.220446049250313e-16):
    		for i in [-0.0212854090294,-0.037991442082]:
    		    result_p = test_pid.get_p(i)
    		    result_i = test_pid.get_i(i, 0.1)
    		    result_d = test_pid.get_d(i, 0.1)
    		    result = result_p + result_i + result_d
    		    print(str(i))
    		    print("Error: %s, Result: %f (P:%f, I:%f, D:%f, Int:%f)" % (i, result, result_p, result_i, result_d, self.get_integrator()))

if __name__ == "__main__":
    test_pid = pid(2.0, 0.5, 0.01, 50)
    test_pid.main()
