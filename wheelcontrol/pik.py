import pickle
import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np
mapmsg = rospy.wait_for_message('/map', OccupancyGrid, 30.0)
imdat = np.array(mapmsg.data).reshape(300,300)
with ('mat.pickle', 'wb') as handle:
	pickle.dump(handle, imdat)	
