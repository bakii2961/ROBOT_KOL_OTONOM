import rospy
from std_msgs.msg import Float64 , Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import threading 
import time 

class RobotArmController:
    def __init__(self):
        self.ilk=1
        self.a1 = 30.1
        self.a2 = 32
        self.a3 = 49.3
        self.kilit=0
        #self.donus=0
        self.pz=-15 #15
        self.py =48 #45 
 

        self.rad=182 #182
       
        self.kilit2=0
        self.kilit1=0
        self.cv_bridge = CvBridge()
        self.bakı=0
        self.motor1=0
        self.motor2=0
        self.motor3=0
        self.donus_camera=0
        self.memet=0
        rospy.init_node('joint_controller')
       
        self.pub1 = rospy.Publisher('/rrbot/joint1_position_controller/command', Float64, queue_size=10) 
        self.pub2 = rospy.Publisher('/rrbot/joint2_position_controller/command', Float64, queue_size=10)  
        self.pub3 = rospy.Publisher('/rrbot/joint5_position_controller/command', Float64, queue_size=10)
        self.pub4 = rospy.Publisher('/Motor1', Int32, queue_size=10)
        self.pub5 = rospy.Publisher('/Motor2', Int32, queue_size=10)
        self.pub6 = rospy.Publisher('/Motor3', Int32, queue_size=10)
        self.pub7 = rospy.Publisher('/bekle/motor', Float64, queue_size=10)
        rospy.Subscriber("/uzunluk", Float64, self.uzunluk_callback)
        rospy.Subscriber("/io_test", Float64, self.io_test)
        rospy.Subscriber("/kinematic/acı", Float64, self.kinematic_acı)
        rospy.Subscriber("/baslangic/konum", Float64, self.baslangic_konum)

     
    def baslangic_konum(self,data):
        self.py=data.data
        self.pz=-15
     
 
    def kinematic_acı(self, data):

        self.rad = data.data 

    def uzunluk_callback(self, data):
        self.pub7.publish(Float64(1))
        if data.data>30:
            self.pz = -43

        
        if data.data==30:
            self.pz = -36
        

        if 0<data.data<15:
            self.pz = -39
        
        if 15<data.data<30:
            self.pz = -43

        if data.data<0:
            self.pz = -36

        self.py = data.data + 40
        
      


    def io_test(self,data):
        self.py = data.data
        if self.py==0:
            self.pz=105
        
        if self.py==50:
            self.pz=20
        


    def inverse_kinematics(self):
        self.phi = np.deg2rad(self.rad)
        wx = self.pz - self.a3 * np.cos(self.phi)
        wy = self.py - self.a3 * np.sin(self.phi)

        delta = wx**2 + wy**2
        c2 = (delta - self.a1**2 - self.a2**2) / (2 * self.a1 * self.a2)
        s2 = np.sqrt(1 - c2**2) 
        theta_2 = np.arctan2(s2, c2)

        s1 = ((self.a1 + self.a2 * c2) * wy - self.a2 * s2 * wx) / delta
        c1 = ((self.a1 + self.a2 * c2) * wx + self.a2 * s2 * wy) / delta
        theta_1 = np.arctan2(s1, c1)
        theta_3 = self.phi - theta_1 - theta_2
        self.kilit = 1
        return theta_1, theta_2, theta_3

    def forward_kinematics(self, theta_1, theta_2, theta_3):
        x = self.a1 * np.cos(theta_1) + self.a2 * np.cos(theta_1 + theta_2) + self.a3 * np.cos(self.phi)
        y = self.a1 * np.sin(theta_1) + self.a2 * np.sin(theta_1 + theta_2) + self.a3 * np.sin(self.phi)
        return x, y

    def run(self):
        while not rospy.is_shutdown():
            theta_1, theta_2, theta_3 = self.inverse_kinematics()
            x, y = self.forward_kinematics(theta_1, theta_2, theta_3)
            self.pub1.publish(Float64(np.rad2deg(theta_1)/58.064516129))
            self.pub3.publish(Float64(np.rad2deg(theta_2)/58.064516129))
            self.pub2.publish(Float64(np.rad2deg(theta_3)/58.064516129))

            self.motor1 = int(float(np.rad2deg(theta_1)/0.00264705882))
            self.motor2 = int(np.rad2deg(theta_2)/0.00209302326)
            self.motor3 = int(np.rad2deg(theta_3)/0.006)
            self.motor1=int(self.motor1)
            self.motor2=int(self.motor2)
            self.motor3=int(self.motor3)

            
            self.pub4.publish(Int32(self.motor1))
            self.pub5.publish(Int32(self.motor2))
            self.pub6.publish(Int32((self.motor3)))
            
            print(self.motor1)#"ben en altdaki motorum ",
            print(self.motor2)#"ben ortadaki motorum "
            print(self.motor3)#"ben en ustdeki motorum "

    


class ImageProcessor:
    def __init__(self, controller):
        self.controller = controller
        self.bridge = CvBridge()
        self.sayac = 0
        self.kilit =0
        self.motor=0
        self.kilit2=1
        self.deneme=1
        self.pub = rospy.Publisher('/rrbot/joint0_position_controller/command', Float64, queue_size=10)
        rospy.Subscriber("/multisense_sl/camera/right/image_raw", Image, self.image_callback)
        rospy.Subscriber("/pozition1/joint0", Float64, self.donus_camera)
        #rospy.Subscriber("/uzak_icin_don", Float64, self.uzak_icin_don)
        self.pub1 = rospy.Publisher('/sifirlama', Float64, queue_size=10)
        self.pub2 = rospy.Publisher('/Alt_motor', Float64, queue_size=10)  
       
        
        self.cap = cv2.VideoCapture(4)
    def uzak_icin_don(self,data):
        if data.data==1:
            if self.donus<0:
                self.donus=self.donus+-750
            if self.donus>0:
                self.donus=self.donus+750
    
    def baslangic_konum(self,data):
        self.py=data.data
        self.pz=-15
        threading.Thread(target=self.basadon).start()

    def basadon(self):
        
        time.sleep(0)
        self.deneme=0
        self.sayac =0
        self.kilit=1
    
       

    def donus_camera(self,data):
       
        self.donus_camera= data.data
        self.pub.publish(Float64(self.donus_camera))
        print("ben donus kamerayimmmmmm ")          


    def find_yellow_contour(self, image):
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            yellow_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(yellow_contour)
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            return cx, cy
        else:
            return None

    def draw_coordinate_system(self, image, center, scale_factor=45):
        cv2.line(image, (0, center[1]), (image.shape[1], center[1]), (0, 0, 255), 2)
        cv2.line(image, (center[0], 0), (center[0], image.shape[0]), (0, 0, 255), 2)
        cv2.putText(image, "X", (image.shape[1] - 20, center[1] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 385), 2)
        cv2.putText(image, "Y", (center[0] + 20, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 45), 2)


        for i in range(0, image.shape[1], scale_factor):
            cv2.line(image, (i, center[1] - 5), (i, center[1] + 5), (0, 0, 255), 1)
            cv2.putText(image, str(i - center[0]), (i, center[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 255), 1)


        for i in range(0, image.shape[0], scale_factor):
            cv2.line(image, (center[0] - 5, i), (center[0] + 5, i), (0, 0, 255), 1)
            cv2.putText(image, str(center[1] - i), (center[0] + 10, i), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 255), 1)


    def image_callback(self, msg):
        try:

                #cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                self.ret, self.frame = self.cap.read()


                cv_image = self.frame.copy()
                image_center = (cv_image.shape[1] // 2, cv_image.shape[0] // 2)
                radius = 160  
                color = (0, 0, 255) 
                thickness = 2
                threading.Thread(target=self.io_cam).start() 
                cv2.circle(cv_image, image_center, radius, color, thickness)
                yellow_center = self.find_yellow_contour(cv_image)
            
                if yellow_center:
                    adjusted_center = (yellow_center[0] - image_center[0], image_center[1] - yellow_center[1])
                    self.draw_coordinate_system(cv_image, image_center)
                    cv2.circle(cv_image, yellow_center, 5, (0, 255, 255), -1)
                    cv2.circle(cv_image, (image_center[0] + adjusted_center[0], image_center[1] - adjusted_center[1]), 5, (0, 255, 0), -1)
                    cv2.putText(cv_image, f"Yellow object: ({yellow_center[0]}, {yellow_center[1]})", 
                                (yellow_center[0] + 10, yellow_center[1] + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                    self.a=(yellow_center[1] - 235)
                    self.b = (yellow_center[0]-315 )
                    #a = -a
                    self.b=-self.b
                    
                    uzunluk =  self.a**2 + self.b**2
                    uzunluk = (uzunluk**0.5)
                    uzunluk = int(uzunluk-3)
    
                    acı = math.atan(self.a/self.b)
                    arctan_derece = math.degrees(acı)
                    print(arctan_derece,"ben burda arctan derece g0steriyorum")
                    print(self.a,"ben aaa yim ")
                    print(self.b,"ben bbbb yim")

                if self.sayac < 14:
                    
                    if self.a > 0 and self.b < 0: 
                        deger = 90 + arctan_derece
                        self.donus = deger / 57.1428571429

                        print("ben donus degeriyim 1",self.donus)

                        if 0>self.b>=-30:   
                            self.donus = self.donus + - 0.13
                        if -30>self.b>=-45:   
                            self.donus = self.donus + -0.15

                        if -45>self.b>=-60:   
                            self.donus = self.donus + -0.14


                        if -60>self.b>=-65:                       
                            self.donus = self.donus + -0.2
                        
                        if -65>self.b>=-80:                       
                            self.donus = self.donus + -0.27
                        if -80>self.b>=-100:
                            self.donus = self.donus + -0.23
                        self.pub.publish(Float64(self.donus))
                        if -100>self.b>=-125:
                            self.donus = self.donus + -0.21

                        if -125>self.b>=-150:
                            self.donus = self.donus + -0.24

                        if -150>self.b>=-200:
                            self.donus = self.donus 
                        if -200>self.b>=-300:
                            self.donus = self.donus  + -0.1     
                        print(self.donus)
                        self.motor =  self.donus/ 0.00473684211
                        print(self.motor)                           
                    if self.a  > 0 and self.b > 0: 
                        deger = -90 +  arctan_derece
                        self.donus = deger / 57.1428571429  
                        if 30>self.b>=0:   
                            self.donus = self.donus + -0.12
                        if 60>self.b>=30:   
                            self.donus = self.donus + -0.17

                        if 90>self.b>=60:                       
                            self.donus = self.donus + -0.21
                        
                        if 120>self.b>=90:                       
                            self.donus = self.donus + -0.27
                        if 120>self.b>=150:
                            self.donus = self.donus + -0.30
                        self.pub.publish(Float64(self.donus))
                        if 150>self.b>=180:
                            self.donus = self.donus + -0.33

        

                        print("ben donus degeriyim 2" ,self.donus)
                        self.pub.publish(Float64(self.donus))
                        self.motor =  self.donus/ 0.00473684211
                        print(self.motor)
                    if self.a  < 0 and self.b > 0: 
                        deger = 90 - arctan_derece
                        self.donus = deger / 57.1428571429
                        self.donus = -self.donus
                        self.pub.publish(Float64(self.donus))
                        print("ben donus degeriyim 3" ,self.donus)
                        self.motor =  self.donus/ 0.00473684211
                        print(self.motor)
                    if self.a  < 0 and self.b < 0: 
                        deger = 90 + arctan_derece
                        self.donus = deger / 57.1428571429
                        self.motor =  self.donus/ 0.00473684211
                        print(self.motor)
                        self.pub.publish(Float64(self.donus))
                        print("ben donus degeriyim 4" ,self.donus)

                    self.pub2.publish(Float64((-1*self.motor)))
                    self.sayac = self.sayac + 1
                    if self.sayac == 14:
                        self.sayac = 15 
                    print(self.a,"ben aaa yim ")
                    print(self.b,"ben bbbb yim")

                cv2.imshow("Image window", cv_image)
                cv2.waitKey(1)
        except Exception as e:
            print("Error in image processing")


    def io_cam(self):
        time.sleep(10)
        self.cap.release()
        
if __name__ == '__main__':
    controller = RobotArmController()
    image_processor = ImageProcessor(controller)
    controller.run()
    rospy.spin()
