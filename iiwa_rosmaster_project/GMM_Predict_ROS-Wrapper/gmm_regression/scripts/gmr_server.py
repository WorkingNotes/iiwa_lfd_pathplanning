#!/usr/bin/env python3
'''gmr ROS Node'''

from gmm_regression.srv import *
import rospy
from std_msgs.msg import String
import numpy as np
from gmr import GMM

gmm_model = GMM(n_components=3)


# define a global GMM object

# load parameters

def gmm_param_load(gmm_model, filename):
    ''' load GMM parameters from file '''
    print("load parameters from %s"%filename)
    with open(filename) as fd:
         loaded_data_array=fd.readlines()
#          unpack data
         num_features, num_components = (int(val) for val in loaded_data_array[0].split())
         priors_raw = [float(val) for val in loaded_data_array[1].split()]
         priors = np.asarray(priors_raw)

         offset = 2
         mu_raw = [[float(val) for val in line.split()] for line in loaded_data_array[2:2+num_components]]
         mu = np.asarray(mu_raw)
         offset = offset + num_components
         sigma0_raw = [[float(val) for val in line.split()] for line in loaded_data_array[offset:offset+num_features]]
         offset = offset + num_features
         sigma1_raw = [[float(val) for val in line.split()] for line in loaded_data_array[offset:offset+num_features]]
         offset = offset + num_features
         sigma2_raw = [[float(val) for val in line.split()] for line in loaded_data_array[offset:offset+num_features]]
         sigma0 = np.asarray(sigma0_raw) +  np.eye(num_features)*0.0001
         sigma1 = np.asarray(sigma1_raw) +  np.eye(num_features)*0.0001
         sigma2 = np.asarray(sigma2_raw) +  np.eye(num_features)*0.0001

         sigma = np.zeros((num_components, num_features, num_features))
         sigma[0,:,:] = sigma0
         sigma[1,:,:] = sigma1
         sigma[2,:,:] = sigma2

         gmm_model.n_components = num_components
         gmm_model.priors = priors
         gmm_model.means = mu
         gmm_model.covariances = sigma


def handle_gmr_srv(req):
    '''
    handle gmr, return gmr request
    '''
    # print("I got what you say: %f , I will give you a feedback!"%(req.pose[0]))
    #
    indices = np.array(range(6))
    testdata = np.array(req.pose)[:,np.newaxis].T
    # print(testdata)
    velocity = gmm_model.predict(indices, testdata)
    # print(velocity)

    # we should array to list
    vel = velocity.ravel().tolist()
    return GmmRegressionResponse(vel)


def gmr():
    '''
    gmr return the regression data of the input data,
    you should input a valid float64[6] data.
    '''
    rospy.init_node('gmr')

    gmm_params_file = rospy.get_param("~gmm_param_path")

    # gmm_param_load(gmm_model,"pose-velctl-gmm_parameters.txt")
    gmm_param_load(gmm_model, gmm_params_file)
    s = rospy.Service('gmr_srv', GmmRegression, handle_gmr_srv)
    rospy.spin()


if __name__ == '__main__':
    try:
        gmr()
    except rospy.ROSInterruptException:
        pass
