#!/usr/bin/env python
#
#
#  
#
#
#

import sys, time
import numpy as np
from sklearn.preprocessing import StandardScaler
from sklearn.decomposition import PCA 
from scipy.ndimage import filters
import pandas as pd
from sklearn.neighbors import KernelDensity
import cv2
import roslib
import rospy
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import JointState
from spot_msgs.msg import BatteryStateArray




VERBOSE=False

class image_feature:

    def __init__(self):
        self.subscriber = rospy.Subscriber("/camera/color/image_raw/compressed",
            CompressedImage, self.callback,  queue_size = 1)
        if VERBOSE :
            print("subscribed to /camera/image/compressed")
        
        #xj = 0
        #print("init")

    def callback(self, ros_data):
        if VERBOSE :
            print('received image of type: "%s"' % ros_data.format)
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        #image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:
        timef = ros_data.header.stamp
        #cv2.imwrite(''+str(timef)+'.jpeg', image_np)
        #cv2.imshow('cv_img', image_np)
       
        
        cv2.waitKey(1)

class JointData:
    def __init__(self):
        self.Edata = []
        self.counter = 0
        self.max_effort_len = 12

        self.Pdata = []
        self.Vdata = []
        self.max_position_len = 10
        self.pop_window = 5

        self.current = []

        self.pca_data = []

        self.variance_list_pc1 = []
        self.variance_list_pc2 = []

        for i in range(len(df_sgb)):
            pca_values = [df_sgb.loc[i,'t1'],
                          df_sgb.loc[i,'t2'],
                          df_sgb.loc[i,'t3'],
                          df_sgb.loc[i,'t4'],
                          df_sgb.loc[i,'t5'],
                          df_sgb.loc[i,'t6'],
                          df_sgb.loc[i,'t7'],
                          df_sgb.loc[i,'t8'],
                          df_sgb.loc[i,'t9']]
            self.pca_data.append(pca_values)
        print(len(self.pca_data))

        self.prior_pca_len = len(self.pca_data)

    def joint_callback(self, msg):
        counter1 = self.counter
        print(counter1)

        # Recording Joint data
        effort_data = [msg.effort[i] for i in range(12)]
        position_data = [msg.position[i] for i in range(12)]
        velocity_data = [msg.velocity[i] for i in range(12)]

        # Record Battery Data
        battery_data = rospy.wait_for_message('/spot/status/battery_states', BatteryStateArray, timeout=None)
        self.current.append(battery_data.battery_states[0].current * -1)
        #print(battery_data.battery_states[0].current)

        if len(self.Edata) < self.max_effort_len:
            self.Edata.append(effort_data)
        else:
            self.Edata[counter1 % self.max_effort_len] = effort_data

        if len(self.Pdata) < self.max_position_len:
            self.Pdata.append(position_data)
            self.Vdata.append(velocity_data)
        else:
            self.Pdata[counter1 % self.max_position_len] = position_data
            self.Vdata[counter1 % self.max_position_len] = velocity_data

        # Analysis
        if counter1 >= 10:
            effort_analysis_results = self.analyze_effort_data()
            #print(analysis_results[0]['V'], analysis_results[0]['Q'])


            # Sanity Check
            """
            dfPCA.loc[counter1 - 11, 't1'] = analysis_results[0]['V']
            dfPCA.loc[counter1 - 11, 't2'] = analysis_results[0]['AC']
            dfPCA.loc[counter1 - 11, 't3'] = analysis_results[0]['AD']
            dfPCA.loc[counter1 - 11, 't4'] = analysis_results[0]['AE']
            dfPCA.loc[counter1 - 11, 't5'] = analysis_results[0]['AF']
            dfPCA.loc[counter1 - 11, 't6'] = analysis_results[0]['AG']
            dfPCA.loc[counter1 - 11, 't9'] = battery_data.battery_states[0].current * -1
            """

        if counter1 >= self.pop_window - 1:
            position_analysis_results = self.analyze_position_data()
            #print(analysis_results[0]['AC'])

            # Sanity Check
            """
            if counter1 >= 10:
                dfPCA.loc[counter1 - 11, 't7'] = analysis_results[0]['AC']
            """

            velocity_analysis_results = self.analyze_velocity_data()
            #print(analysis_results[0]['AC'])

            # Sanity Check
            """
            if counter1 >= 10:
                dfPCA.loc[counter1 - 11, 't8'] = analysis_results[0]['AC']
            

        dfPCA.to_csv("apr2_1_PCA_ready_Analysis.csv", index=False)

        """
            
        if counter1 >= 10:
            pcaList = [
                effort_analysis_results[0]['V'],
                effort_analysis_results[0]['AC'],
                effort_analysis_results[0]['AD'],
                effort_analysis_results[0]['AE'],
                effort_analysis_results[0]['AF'],
                effort_analysis_results[0]['AG'],
                position_analysis_results[0]['AC'],
                velocity_analysis_results[0]['AC'],
                battery_data.battery_states[0].current * -1
            ]

            #print(pcaList)

            self.pca_data.append(pcaList)

            if len(self.pca_data) >= self.prior_pca_len + 10:
                self.pca_data.pop(self.prior_pca_len)

            #if len(self.pca_data) >= 2:
            if counter1%counter1 == 0:
                data_array = np.array(self.pca_data)

                scaled_data = StandardScaler().fit_transform(data_array)

                pca = PCA(n_components=2)
                principal_components = pca.fit_transform(scaled_data)

                principalDf = pd.DataFrame(data = principal_components
                    , columns = ['principal component 1', 'principal component 2'])
                
                principalDf.to_csv("PCA_ready.csv", index=False)



                """
                # KDE algorithem

                kde = KernelDensity(kernel='gaussian', bandwidth=0.5)
                kde.fit(principal_components)

                # Estimate the density at specific points using the 'score_samples' method
                # For example, to estimate the density at the point (0, 0):
                density = kde.score_samples([[0, 0]])

                print(density)

                """
                
                # check mean and variance
                mean_principal_component_1 = np.mean(principal_components[:, 0])
                mean_principal_component_2 = np.mean(principal_components[:, 1])

                variance_principal_component_1 = np.var(principalDf['principal component 1'][len(principalDf)-10:len(principalDf)])
                variance_principal_component_2 = np.var(principalDf['principal component 2'][len(principalDf)-10:len(principalDf)])

                """
                # Print the mean and variance
                print("Mean of Principal Component 1:", mean_principal_component_1)
                print("Mean of Principal Component 2:", mean_principal_component_2)
                print("Variance of Principal Component 1:", variance_principal_component_1)
                print("Variance of Principal Component 2:", variance_principal_component_2)
                """

                # Append variances to their respective lists
                self.variance_list_pc1.append(variance_principal_component_1)
                self.variance_list_pc2.append(variance_principal_component_2)

                # Save lists to a CSV file
                variance_df = pd.DataFrame({'Variance PC1': self.variance_list_pc1, 'Variance PC2': self.variance_list_pc2})
                variance_df.to_csv("variance_pca_mixed.csv", index=False)

               
                

        self.counter += 1

        if len(self.Edata) > self.max_effort_len:
            self.Edata.pop(0)
        if len(self.Pdata) > self.max_position_len:
            self.Pdata.pop(0)
            self.Vdata.pop(0)


    def analyze_effort_data(self):
        first, second, third, forth = 2, 5, 8, 11
        analysis_results = []
        

        for i in range(10, len(self.Edata)):
            dataMax = [0, 0, 0, 0]
            counters = [0, 0, 0, 0]
            for j in range(11):
                #print(dataMax)
                data = [self.Edata[i - j][first], self.Edata[i - j][second],
                        self.Edata[i - j][third], self.Edata[i - j][forth]]
                for k in range(4):
                    if data[k] > dataMax[k]:
                        counters[k] += 1
                        dataMax[k] = data[k]

            Q = sum(dataMax) / 4.0
            R, S, T, U = counters
            V = sum(counters)

            differences = [dataMax[k] - Q for k in range(4)]
            ratios = [differences[k] / Q for k in range(4)]
            abs_sum_ratios = sum([abs(ratio) for ratio in ratios])

            X, Y, Z, AA = differences
            AC, AD, AE, AF = ratios
            AG = abs_sum_ratios

            t1 = V
            t2, t3, t4, t5 = ratios
            t6 = abs_sum_ratios

            result = {'Q': Q, 'R': R, 'S': S, 'T': T, 'U': U, 'V': V,
                      'X': X, 'Y': Y, 'Z': Z, 'AA': AA, 'AC': AC, 'AD': AD, 'AE': AE, 'AF': AF, 'AG': AG,
                      't1': t1, 't2': t2, 't3': t3, 't4': t4, 't5': t5, 't6': t6}

            analysis_results.append(result)

        return analysis_results
    

    def analyze_position_data(self):
        analysis_results = []

        for i in range(self.pop_window - 1, len(self.Pdata)):
            diff_values = []
            for idx, (start, end) in enumerate([(0, 1), (3, 4), (6, 7), (9, 10)]):
                max_val = max([row[start] for row in self.Pdata[i - self.pop_window + 1:i + 1]])
                min_val = min([row[start] for row in self.Pdata[i - self.pop_window + 1:i + 1]])
                diff = max_val - min_val
                diff_values.append(diff)

            total_diff_sum = sum(diff_values)

            result = {'R': diff_values[0], 'U': diff_values[1], 'X': diff_values[2], 'AA': diff_values[3], 'AC': total_diff_sum}
            analysis_results.append(result)

        return analysis_results
    
    def analyze_velocity_data(self):
        analysis_results = []

        for i in range(self.pop_window - 1, len(self.Vdata)):
            diff_values = []
            for idx, (start, end) in enumerate([(0, 1), (3, 4), (6, 7), (9, 10)]):
                max_val = max([row[start] for row in self.Vdata[i - self.pop_window + 1:i + 1]])
                min_val = min([row[start] for row in self.Vdata[i - self.pop_window + 1:i + 1]])
                diff = max_val - min_val
                diff_values.append(diff)

            total_diff_sum = sum(diff_values)

            result = {'R': diff_values[0], 'U': diff_values[1], 'X': diff_values[2], 'AA': diff_values[3], 'AC': total_diff_sum}
            analysis_results.append(result)

        return analysis_results




if __name__ == '__main__':

    ic = image_feature()
    rospy.init_node('image_feature', anonymous=True)

    df_sgb = pd.read_csv('mar26_SGB_test_PCA2.csv')

    config = JointData()
    #rospy.init_node("joint_subscriber")
    sub = rospy.Subscriber("/joint_states", JointState, callback = config.joint_callback)
    sub2 = rospy.Subscriber("/spot/status/battery_states", BatteryStateArray)
    rospy.loginfo("Node has been started")

    #dfPCA= pd.read_csv('PCA.csv')
    #dfCount = pd.read_csv('Counter.csv')

    #dfCount.loc[0, 'count'] = 0

    
    



    rospy.spin()
    

    """
    while not rospy.is_shutdown():
        fg = JointState()
        print(fg.position)

    """