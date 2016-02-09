#!/usr/bin/env python
# -*- Python -*-

"""
 \file TrajectoryEstimation.py
 \brief Trajectory Estimation
 \date $Date$


"""
import sys
import time
sys.path.append(".")

import math
import numpy as np
from scipy.optimize import fsolve

# Import RTM module
import RTC
import OpenRTM_aist

# Import Service implementation class
# <rtc-template block="service_impl">

# </rtc-template>

# Import Service stub modules
# <rtc-template block="consumer_import">
# </rtc-template>


# This module's spesification
# <rtc-template block="module_spec">
trajectoryestimation_spec = ["implementation_id", "TrajectoryEstimation",
		 "type_name",         "TrajectoryEstimation",
		 "description",       "Trajectory Estimation",
		 "version",           "1.0.0",
		 "vendor",            "yuguchi",
		 "category",          "Category",
		 "activity_type",     "STATIC",
		 "max_instance",      "0",
		 "language",          "Python",
		 "lang_type",         "SCRIPT",
		 ""]
# </rtc-template>

class TrajectoryEstimation(OpenRTM_aist.DataFlowComponentBase):

	"""
	\class TrajectoryEstimation
	\brief Trajectory Estimation

	"""
	def __init__(self, manager):
		"""
		\brief constructor
		\param manager Maneger Object
		"""
		OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)

		self._d_MarkerIn_var = RTC.TimedDoubleSeq(RTC.Time(0,0),[])
		"""
		"""
		self._MarkerInIn = OpenRTM_aist.InPort("MarkerIn", self._d_MarkerIn_var)
		self._d_PositionOut_var = RTC.TimedDoubleSeq(RTC.Time(0,0),[])
		"""
		"""
		self._PositionOutOut = OpenRTM_aist.OutPort("PositionOut", self._d_PositionOut_var)





		# initialize of configuration-data.
		# <rtc-template block="init_conf_param">

		# </rtc-template>



	def onInitialize(self):
		"""

		The initialize action (on CREATED->ALIVE transition)
		formaer rtc_init_entry()

		\return RTC::ReturnCode_t

		"""
		# Bind variables and configuration variable

		# Set InPort buffers
		self.addInPort("MarkerIn",self._MarkerInIn)

		# Set OutPort buffers
		self.addOutPort("PositionOut",self._PositionOutOut)

		# Set service provider to Ports

		# Set service consumers to Ports

		# Set CORBA Service Ports

		return RTC.RTC_OK

	def onFinalize(self, ec_id):
		"""

		The finalize action (on ALIVE->END transition)
		formaer rtc_exiting_entry()

		\return RTC::ReturnCode_t

		"""

		return RTC.RTC_OK

	#def onStartup(self, ec_id):
	#	"""
	#
	#	The startup action when ExecutionContext startup
	#	former rtc_starting_entry()
	#
	#	\param ec_id target ExecutionContext Id
	#
	#	\return RTC::ReturnCode_t
	#
	#	"""
	#
	#	return RTC.RTC_OK

	#def onShutdown(self, ec_id):
	#	"""
	#
	#	The shutdown action when ExecutionContext stop
	#	former rtc_stopping_entry()
	#
	#	\param ec_id target ExecutionContext Id
	#
	#	\return RTC::ReturnCode_t
	#
	#	"""
	#
	#	return RTC.RTC_OK

	def onActivated(self, ec_id):
		"""

		The activated action (Active state entry action)
		former rtc_active_entry()

		\param ec_id target ExecutionContext Id

		\return RTC::ReturnCode_t

		"""
		# Defines parameters and global variables
		initialize_controller()

		return RTC.RTC_OK

	def onDeactivated(self, ec_id):
		"""

		The deactivated action (Active state exit action)
		former rtc_active_exit()

		\param ec_id target ExecutionContext Id

		\return RTC::ReturnCode_t

		"""

		return RTC.RTC_OK

	def onExecute(self, ec_id):
		"""

		The execution action that is invoked periodically
		former rtc_active_do()

		\param ec_id target ExecutionContext Id

		\return RTC::ReturnCode_t

		"""

		### main process ###

		DEBUG = False
		SHOWTIMESTAMP = True

		global hrp_plane
		global pnum

		while not self._MarkerInIn.isNew():
			print "wait" #waiting to receive data
		if DEBUG:
			print "Getting data from MoCap"
		if SHOWTIMESTAMP:
			starttime = time.clock()
		try:
        		self._d_MarkerIn_var = self._MarkerInIn.read()	
        		t_now = int(self._d_MarkerIn_var.data[0])
			x_now = self._d_MarkerIn_var.data[1]
			y_now = self._d_MarkerIn_var.data[2]
			z_now = self._d_MarkerIn_var.data[3]
			print t_now,x_now,y_now,z_now

         		tC, xC, yC, zC, vel_vector, success = controller_loop(t_now, x_now, y_now, z_now, DEBUG)

        	except Exception as inst:
	         print type(inst)
	         print inst.args
	         print inst

	 	if DEBUG:
			print "success = " + str(success)

		if success:
			try:
				if DEBUG:
					print "C3. Sending data to HRP4"
					print xC, yC, zC, tC, vel_vector

				X_BIAS = 0.168142 - 0.307469
				Y_BIAS = 1.0 - 0.22622
				Z_BIAS = -0.312781 - (-0.287506)

				X_DIRECTION = 1.0
				Y_DIRECTION = 1.0
				Z_DIRECTION = 1.0

				xHRP = (X_DIRECTION*xC) - X_BIAS
				yHRP = -((Z_DIRECTION*zC) - Z_BIAS)
				zHRP = (Y_DIRECTION*yC) - Y_BIAS

				xHRP = float(xHRP)
				yHRP = float(yHRP)
				zHRP = float(zHRP)

				self._d_PositionOut_var.data = [xHRP,yHRP,zHRP]
				print xHRP,yHRP,zHRP
				self._PositionOutOut.write()
			except Exception as inst:
				print type(inst)
				print inst.args
				print inst
		else:
			print "debug"

		if SHOWTIMESTAMP:
                	endtime = time.clock()
                	print "Seconds required in this loop:"
           		print endtime - starttime

		return RTC.RTC_OK

	#def onAborting(self, ec_id):
	#	"""
	#
	#	The aborting action when main logic error occurred.
	#	former rtc_aborting_entry()
	#
	#	\param ec_id target ExecutionContext Id
	#
	#	\return RTC::ReturnCode_t
	#
	#	"""
	#
	#	return RTC.RTC_OK

	#def onError(self, ec_id):
	#	"""
	#
	#	The error action in ERROR state
	#	former rtc_error_do()
	#
	#	\param ec_id target ExecutionContext Id
	#
	#	\return RTC::ReturnCode_t
	#
	#	"""
	#
	#	return RTC.RTC_OK

	#def onReset(self, ec_id):
	#	"""
	#
	#	The reset action that is invoked resetting
	#	This is same but different the former rtc_init_entry()
	#
	#	\param ec_id target ExecutionContext Id
	#
	#	\return RTC::ReturnCode_t
	#
	#	"""
	#
	#	return RTC.RTC_OK

	#def onStateUpdate(self, ec_id):
	#	"""
	#
	#	The state update action that is invoked after onExecute() action
	#	no corresponding operation exists in OpenRTm-aist-0.2.0
	#
	#	\param ec_id target ExecutionContext Id
	#
	#	\return RTC::ReturnCode_t
	#
	#	"""
	#
	#	return RTC.RTC_OK

	#def onRateChanged(self, ec_id):
	#	"""
	#
	#	The action that is invoked when execution context's rate is changed
	#	no corresponding operation exists in OpenRTm-aist-0.2.0
	#
	#	\param ec_id target ExecutionContext Id
	#
	#	\return RTC::ReturnCode_t
	#
	#	"""
	#
	#	return RTC.RTC_OK

#####################################################
## add functions of trajectory estimation ##
#####################################################

# Global variables initialization.
# Valid values for trajectory estimation method: polyfit
def initialize_controller(method = "polyfit", p_num = 5):
    global HRP4_PLANE
    global P_NUM
    global METHOD

    global t_all
    global x_all
    global y_all
    global z_all

    t_all = []
    x_all = []
    y_all = []
    z_all = []

    # Set the HRP-4 work plane. Form: ax + by + cz + d = 0.
    HRP4_PLANE = [0, 1, 0, -1]

    # Number of frames used for trajectory estimation.
    P_NUM = p_num
    METHOD = method

def controller_loop(t_now, x_now, y_now, z_now, DEBUG = False, full_output = False):
    global HRP4_PLANE
    global P_NUM
    global METHOD

    global t_all
    global x_all
    global y_all
    global z_all


    if DEBUG:
            print "--------------------------------------------------------------------------------"
            print "Current point (t, x, y, z):"
            print t_now, x_now, y_now, z_now

    # Check if new point is identical to last. If yes, ignore. Only check after first point.
    if x_all:
        if abs(x_now - x_all[-1]) < 0.01 or abs(y_now - y_all[-1]) < 0.01 or abs(z_now - z_all[-1]) < 0.01:
            if DEBUG:
                print "Current point identical to last, skipped."
            if full_output:
                return 0, 0, 0, 0, 0, False, []
            else:
                return 0, 0, 0, 0, 0, False

    t_all.append(t_now)
    x_all.append(x_now)
    y_all.append(y_now)
    z_all.append(z_now)

    if len(t_all) > 100:
        t_all = t_all[-P_NUM:]
        x_all = x_all[-P_NUM:]
        y_all = y_all[-P_NUM:]
        z_all = z_all[-P_NUM:]

    # Estimate point of arrival based on last seen points.
    if len(t_all) > P_NUM-1:
        t_c, x_c, y_c, z_c, v_vect, a_1, a_0, b_2, b_1, b_0, c_1, c_0, success = get_ball_arrival_point(t_all[-P_NUM:], x_all[-P_NUM:], y_all[-P_NUM:], z_all[-P_NUM:], HRP4_PLANE, METHOD, DEBUG)
        if full_output:
            curve_params = [a_1, a_0, b_2, b_1, b_0, c_1, c_0]
            return t_c, x_c, y_c, z_c, v_vect, success, curve_params
        else:
            return t_c, x_c, y_c, z_c, v_vect, success
    # When not enough points available, return failure.
    else:
        if full_output:
            return 0, 0, 0, 0, 0, False, []
        else:
            return 0, 0, 0, 0, 0, False

# Function to compute the ball arrival point.
def get_ball_arrival_point(t_in, x_in, y_in, z_in, plane, METHOD, DEBUG):
    if len(x_in) < 3:
        print "Error: 3 points or more necessary."
        return -1

    t_diff = t_in[-1]
    for t in range(0, len(t_in)):
        t_in[t] = t_in[t] - t_diff

    if METHOD is "polyfit":
        a_1, a_0, b_2, b_1, b_0, c_1, c_0 = estimate_trajectory_polyfit(t_in, x_in, y_in, z_in)

    elif METHOD is "newton":
        a_1, a_0, b_2, b_1, b_0, c_1, c_0 = estimate_trajectory_newton(t_in, x_in, y_in, z_in)
    elif METHOD is "stokes":
        a_1, a_0, b_2, b_1, b_0, c_1, c_0 = estimate_trajectory_stokes(t_in, x_in, y_in, z_in)

    else:
    	   print "Error: trajectory estimation method does not exist."
    	   raise

    if DEBUG:
        print "Trajectory estimation method is " + METHOD + " with at least " + str(P_NUM) + " points."
        print "Found x coefficients (a1, a0):"
        print a_1, a_0
        print "Found y coefficients (b2, b1, b0):"
        print b_2, b_1, b_0
        print "Found z coefficients (c1, c0):"
        print c_1, c_0

    # Estimate intersection of trajectory with plane
    x = np.poly1d([a_1, a_0])
    y = np.poly1d([b_2, b_1, b_0])
    z = np.poly1d([c_1, c_0])

    def curve(t_s):
        return x(t_s), y(t_s), z(t_s)

    t_c, success = find_plane_intersection(t_in[-1], plane, curve, a_1, a_0, b_2, b_1, b_0, c_1, c_0)
    if not success:
        return t_c, [],[],[],[], a_1, a_0, b_2, b_1, b_0, c_1, c_0, False

    x_c, y_c, z_c = curve(t_c)

    v_vect = [a_1, 2*b_2*t_c + b_1, c_1] # Velocity vector at point of intersection.

    t_c = t_c + t_diff # Renormalize to match time scale outside function.

    #if DEBUG:
    #    add_estimated_trajectory_to_csv_file(t_diff, a_1, a_0, b_2, b_1, b_0, c_1, c_0, x_c[0], y_c[0], z_c[0], t_c[0])
    #    print "Trajectory estimation results saved to CSV file."

    return t_c, x_c, y_c, z_c, v_vect, a_1, a_0, b_2, b_1, b_0, c_1, c_0, True

# Use all captured points to estimate ball trajectory. LSQ approximation.
def estimate_trajectory_polyfit(t_in, x_in, y_in, z_in):
    a_1, a_0 = np.polyfit(t_in, x_in, 1)
    b_2, b_1, b_0 = np.polyfit(t_in, y_in, 2)
    c_1, c_0 = np.polyfit(t_in, z_in, 1)
    return a_1, a_0, b_2, b_1, b_0, c_1, c_0

def estimate_trajectory_newton(t_in, x_in, y_in, z_in):
    g = 2*5e-6 # Gravity constant was empirically defined because time scale units are not seconds.

    # Linear approximation of the velocity vector.
    v_x = (x_in[-1] - x_in[-2]) / (t_in[-1] - t_in[-2])
    v_y = (y_in[-1] - y_in[-2]) / (t_in[-1] - t_in[-2])
    v_z = (z_in[-1] - z_in[-2]) / (t_in[-1] - t_in[-2])

    # Equations of a parabolic motion.
    a_1 = v_x
    a_0 = x_in[-2]

    b_2 = -g/2
    b_1 = v_y
    b_0 = y_in[-2]

    c_1 = v_z
    c_0 = z_in[-2]

    return a_1, a_0, b_2, b_1, b_0, c_1, c_0

def estimate_trajectory_stokes(t_in, x_in, y_in, z_in):
    # Gravity at Tokyo
    g_tokyo = 9.7976

    # radius
    a = 0.06583 / 2

    #[Pa*s]
    u = 17.9*0.000001

    k = 6*(math.pi)*a*u
    #[kg]
    m = 0.0114

    dx = x_in[-1] - x_in[-2]
    dy = y_in[-1] - y_in[-2]
    dz = z_in[-1] - z_in[-2]
    dt = t_in[-1] - t_in[-2]

    vx = dx / dt
    vy = dy / dt
    vz = dz / dt

    a = -g_tokyo + (k / m) * vy

    a_1 = vx
    a_0 = x_in[-2]

    b_2 = a/2*(0.001)*(0.001)
    b_1 = vy *(math.exp(-((k/m)*dt)))
    b_0 = y_in[-2]

    c_1 = vz
    c_0 = z_in[-2]

    return a_1, a_0, b_2, b_1, b_0, c_1, c_0

# Function to compute intersection between work plane and estimated trajectory.

def find_plane_intersection(t_c, plane, curve, a_1, a_0, b_2, b_1, b_0, c_1, c_0):
    def f(t):
        x_c, y_c, z_c = curve(t)
        return plane[0]*x_c + plane[1]*y_c + plane[2]*z_c + plane[3] # Distance of point to plane.

    res = fsolve(f, t_c, maxfev = 20, full_output = True)

    t_ca = res[0]

    # If there is distance to the plane, there is no intersection. Degenerate case.
    if abs(res[1]['fvec']) > 0.1:
        return t_ca, False

    # Check if point occurs before saddle point (feasible points cannot be). This could maybe be done by derivative and become faster, but probably not by much, if at all.
    t_root = - (b_1 / (2*b_2))  # Equivalent of np.roots([2*b_2, b_1]).
    if t_ca > t_root:
        return t_ca, True
    else:
        t_c = t_root + (t_root - t_ca)
        return fsolve(f, t_c), True # Solve again with iteration starting closer to desired point.

###################################################################


def TrajectoryEstimationInit(manager):
    profile = OpenRTM_aist.Properties(defaults_str=trajectoryestimation_spec)
    manager.registerFactory(profile,
                            TrajectoryEstimation,
                            OpenRTM_aist.Delete)

def MyModuleInit(manager):
    TrajectoryEstimationInit(manager)

    # Create a component
    comp = manager.createComponent("TrajectoryEstimation")

def main():
	mgr = OpenRTM_aist.Manager.init(sys.argv)
	mgr.setModuleInitProc(MyModuleInit)
	mgr.activateManager()
	mgr.runManager()

if __name__ == "__main__":
	main()
