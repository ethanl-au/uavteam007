import math
import numpy as np

class Localise():

    def localisation(self,centroid_img, pose):
        #Vertical and Horizontal FOV
        FOV = 38 # degrees
        resolution = 416
        yaw = - math.radians(pose[3])

        norm_coord = np.array([centroid_img[0]/resolution,centroid_img[1]/resolution])
        
        width_meters = 2*pose[2]*math.tan(math.radians(FOV/2))

        coord_img_frame = np.array([norm_coord[0]*width_meters,norm_coord[1]*width_meters])
        coord_img_frame = np.array([coord_img_frame[0],coord_img_frame[1],-pose[2],1])

        transformation_drone = np.array([[0,-1,0,width_meters/2],
        [-1,0,0,width_meters/2],
        [0,0,-1,0],
        [0,0,0,1]])
        
        coord_drone_frame = np.matmul(transformation_drone,coord_img_frame)

        transformation_global = np.array([[math.cos(yaw),math.sin(yaw),0,pose[0]],
        [-math.sin(yaw),math.cos(yaw),0,pose[1]],
        [0,0,1,-pose[2]],
        [0,0,0,1]])

        global_coord = np.matmul(transformation_global,coord_drone_frame)

        

        return global_coord
        #Drone to Global




# def main():
#     x_img = 208 #[pixels]
#     y_img = 83.2

#     x = 0 #[meters]
#     y = 0 #[meters]
#     z = 3 #[meters]
#     yaw = 90 #[degrees]
#     centroid_img = [x_img,y_img]
#     optitrack = [x,y,z,yaw]
#     localisation(centroid_img,optitrack)








# if __name__ == '__main__':
#     main()