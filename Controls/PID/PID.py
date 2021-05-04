# Taken from this tutorial on Autopilot: https://erlerobot.github.io/erle_gitbook/en/beaglepilot/SimpleAutopilot.html

import time

class PID:
    """ Simple PID control.
        This class implements a simplistic PID control algorithm. When first
        instantiated all the gain variables are set to zero, so calling
        the method GenOut will just return zero.
    """
    def __init__(self, Kp = 1, Kd = 0, Ki = 0):
        # initialze gains
        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki

        # realistic adaptation
        # self.windup_guard = 20.0

        self.Initialize()

    def SetKp(self, invar):
        """ Set proportional gain. """
        self.Kp = invar

    def SetKi(self, invar):
        """ Set integral gain. """
        self.Ki = invar

    def SetKd(self, invar):
        """ Set derivative gain. """
        self.Kd = invar

    def SetPrevErr(self, preverr):
        """ Set previous error value. """
        self.prev_err = preverr

    def SetWindUp(self, windup):
        """Set windup value to prevent the integrator from overshooting"""
        self.windup_guard = windup

    def Initialize(self):
        # initialize delta t variables
        self.currtm = time.time()
        self.prevtm = self.currtm

        self.prev_err = 0

        # term result variables
        self.Cp = 0
        self.Ci = 0
        self.Cd = 0


    def update(self, error, debug = 0):
        """ Performs a PID computation and returns a control value based on
            the elapsed time (dt) and the error signal from a summing junction
            (the error parameter).
        """
        if debug:
            print "   ****************************"
            print "   PID Debug"
            print "   ****************************"
            print "   error:"+str(error)

        self.currtm = time.time()               # get t
        dt = self.currtm - self.prevtm          # get delta t
        de = error - self.prev_err              # get delta error
        # P term
        self.Cp = error               
        if debug:
            print "   Proportional term (Cp*Kp):"+str(self.Cp * self.Kp)

        # I term
        self.Ci += error * dt                   
        if debug:
            print "   Integral term (Ci*Ki):"+str(self.Ci * self.Ki)

        # D term
        self.Cd = 0
        if dt > 0:                              # no div by zero
            self.Cd = de/dt                     
            if debug:
                print "   Derivative term (Cd*Kd):"+str(self.Cd * self.Kd)

        # Update for next loop
        self.prevtm = self.currtm               
        self.prev_err = error                   #

        # Windup guard
        # if (self.Ci < -self.windup_guard):
        #    self.Ci = -self.windup_guard
        # elif (self.Ci > self.windup_guard):
        #    self.Ci = self.windup_guard

        # sum the terms and return the result
        terms_sum = self.Cp * self.Kp  + (self.Ki * self.Ci) + (self.Kd * self.Cd)
        if debug:
            print "   Terms sum (self.Cp + (self.Ki * self.Ci) + (self.Kd * self.Cd)):"+str(terms_sum)        
            print "   ****************************"
        return terms_sum