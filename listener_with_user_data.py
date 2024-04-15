import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import Float64
import time
import math
import threading  


class ImageProcessor:
    def __init__(self):

        self.k=0
        """
        if self.k ==0:
            time.sleep(6)
            self.k =1
        """
        rospy.init_node('image_processor', anonymous=True)
        self.bridge = CvBridge()
        # Kamera başlatılıyor
        self.cap = cv2.VideoCapture(2)  # 0, birincil kamerayı temsil eder. Eğer birden fazla kamera varsa, numarasını değiştirebilirsiniz.
        self.image_sub = rospy.Subscriber("/rrbot/camera1/image_raw", Image, self.image_callback)
        self.pub0 = rospy.Publisher('/uzunluk', Float64, queue_size=10)  # Taban
        self.pub1 = rospy.Publisher('/sag_sol', Float64, queue_size=10)  # Omuz
        self.pub2 = rospy.Publisher('/taban_yon', Float64, queue_size=10)  # Dirsek
        #self.pub3 = rospy.Publisher('/config', Float64, queue_size=10)  # Omuz
        self.pub = rospy.Publisher('/altkamera', Float64, queue_size=10)
        self.pub4 = rospy.Publisher('/rrbot/joint3_position_controller/command', Float64, queue_size=10) 
        self.pub5 = rospy.Publisher('/rrbot/joint4_position_controller/command', Float64, queue_size=10) 
        self.pub6 = rospy.Publisher('/io_test', Float64, queue_size=10) 
        self.pub7 = rospy.Publisher('/pozition1/joint0', Float64, queue_size=10)
        self.pub8 = rospy.Publisher('/kinematic/acı', Float64, queue_size=10)
        self.pub9 = rospy.Publisher('/baslangic/konum', Float64, queue_size=10)
        self.pub10 = rospy.Publisher('/gripper', Float64, queue_size=10)
        self.pub11 = rospy.Publisher('/Alt_motor', Float64, queue_size=10)  
        self.pub12 = rospy.Publisher('/rrbot/joint1_position_controller/command', Float64, queue_size=10) 
        self.pub13 = rospy.Publisher('/rrbot/joint2_position_controller/command', Float64, queue_size=10)  
        self.pub14= rospy.Publisher('/rrbot/joint5_position_controller/command', Float64, queue_size=10)
        self.pub15= rospy.Publisher('/bilek', Float64, queue_size=10)
        rospy.Subscriber("/sifirlama", Float64, self.sifirlama)
        rospy.Subscriber("/object_detection", Float64, self.object_detection)
        self.sifir=0
        self.kır1=0
        self.kır=0    
        self.onceki_deger = 0
        self.onceki_deger1=0
        self.sonDeger = 0
        self.kilit = 0 
        self.kilit1=0
        self.kilit3=0
        self.sayac=0
        self.sayac1=0
        self.sayilar = []
        self.sayilar1 = []
        self.pozition1=1    
        self.ayiklama1=0
        self.a=0
        self.lock=0
        self.ol=0
   

    def sifirlama(self, data):

        self.sifir=data.data
  

    def object_detection(self,data):

        self.ayiklama1=data.data

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
    
    def draw_coordinate_system(self, image, center, scale_factor=20):
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
            ret, frame = self.cap.read()
            cv_image = frame.copy()
            yellow_center = self.find_yellow_contour(cv_image)
            if yellow_center:
                image_center = (cv_image.shape[1] // 2, cv_image.shape[0] // 2)
                adjusted_center = (yellow_center[0] - image_center[0], image_center[1] - yellow_center[1])
                self.draw_coordinate_system(cv_image, image_center)
                cv2.circle(cv_image, yellow_center, 5, (0, 255, 255), -1)
                cv2.circle(cv_image, (image_center[0] + adjusted_center[0], image_center[1] - adjusted_center[1]), 5, (0, 255, 0), -1)
                cv2.putText(cv_image, f"Yellow object: ({yellow_center[0]}, {yellow_center[1]})", (yellow_center[0] + 10, yellow_center[1] + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                a = yellow_center[1] -235
                b = yellow_center[0] 
               
                a=-a
                self.sayilar.append(a)
                a = self.sayilar[2] 
                
                #  kamerayı kola montelettir
                # kamera  boy oranlarını kıyasla ve a değerini bir çarpan ile böl 
                # önce simde dene ve sonra kol üzerinde uygula 


          
                a = -a/0.5
                acı = math.atan(a/b)
                arctan_derece = math.degrees(acı)
                arctan_derece=float(arctan_derece)
                
                if self.sayac<4:
                        print("Yellow object centroid coordinates relative to origin (0,0):", a, b)
                        if -20<=a<0:
                            #a =a/200
                            a =0
                            self.pub0.publish(Float64(a)) 
  
                        if -40<=a<-20:
                            #a =a/200
                            a =-1
                            self.pub0.publish(Float64(a)) 
  

                        if -60<=a<-40:
                            #a =a/200
                            a =-2
                            self.pub0.publish(Float64(a)) 
  
                        if -80<=a<-60:
                            #a =a/200
                            a =-2.5
                            self.pub0.publish(Float64(a)) 
  
                        if -100<=a<-80:
                            #a =a/200
                            a =-3
                            self.pub0.publish(Float64(a)) 

                        if -120<=a<-100:
                            #a =a/200
                            a =-4
                            self.pub0.publish(Float64(a)) 

                        if -140<=a<-120:
                            a =-5
                            self.pub0.publish(Float64(a))
                            print(a)
                        """
                        if -160<=a<-140:
                            a =-8
                            self.pub0.publish(Float64(a))
                            print(a)

                        if -180<=a<-160:
                            a =-9
                            self.pub0.publish(Float64(a))
                            print(a)

                        if -200<=a<-180:
                            a =-10
                            self.pub0.publish(Float64(a))
                            print(a)

                        if -220<=a<-200:
                            a =-13
                            self.pub0.publish(Float64(a))
                            print(a)

                        if -240<=a<-220:
                            a =-15
                            self.pub0.publish(Float64(a))
                            print(a)

                        if -260<=a<-240:
                            a =-16.4
                            self.pub0.publish(Float64(a))
                            print(a)

                        if -280<=a<-260:
                            a =-17
                            self.pub0.publish(Float64(a))
                            print(a)

                        if -300<=a<-280:
                            a =-17.5
                            self.pub0.publish(Float64(a))
                            print(a)


                        if -320<=a<-300:
                            a =-16.8
                            self.pub0.publish(Float64(a))
                            print(a)

                        if -340<=a<-320:
                            a =-18.5
                            self.pub0.publish(Float64(a))
                            print(a)

                        """

                        if 0<a<=20:
                            a =1
                            self.pub0.publish(Float64(a))
                            print(a)

                        if 20<a<=40:
                            a =2
                            self.pub0.publish(Float64(a))
                            print(a)

                        if 40<a<=60:
                            a =3.2
                            self.pub0.publish(Float64(a))
                            print(a)

                        if 60<a<=80:
                            a =4.2
                            self.pub0.publish(Float64(a))
                            print(a)
                        if 80<a<=100:
                            a =5.2
                            self.pub0.publish(Float64(a))
                            print(a)

                        if 100<a<=120:
                            a =6.4
                            self.pub0.publish(Float64(a))
                            print(a)
                        
                        if 120<a<=140:
                            a =8.5
                            self.pub0.publish(Float64(a))
                            print(a)

                        if 140<a<=160:
                            a =10
                            self.pub0.publish(Float64(a))
                            print(a)
                        
                        if 160<a<=180:
                            a = 11.5
                            self.pub0.publish(Float64(a))
                            print(a)

                        if 180<a<=200:
                            a = 13
                            self.pub0.publish(Float64(a))
                            print(a)

                        if 200<a<220:
                            a=15
                            self.pub0.publish(Float64(a))
                            print(a)
                        
                        if 220<a<240:
                            a=18
                            self.pub0.publish(Float64(a))
                            print(a)                        

                        if 240<a<260:
                            a=18
                            self.pub0.publish(Float64(a))
                            print(a)
                        
                        
                        if 260<a<280:
                            a=20
                            self.pub8.publish(Float64(160))
                            self.pub0.publish(Float64(a))
                            print(a)

                        if 280<a<300:
                            a=22
                            self.pub8.publish(Float64(160))
                            self.pub0.publish(Float64(a))
                            print(a)
                        
                        if 300<a<320:
                            a=22
                            self.pub8.publish(Float64(160))
                            self.pub0.publish(Float64(a))
                            print(a)

                        if 320<a<340:
                            a=25
                            self.pub8.publish(Float64(160))
                            self.pub0.publish(Float64(a))
                            print(a)  

                        if 340<a<360:
                            a=28
                            self.pub8.publish(Float64(160))
                            self.pub0.publish(Float64(a))
                            print(a)  
                    
                        if 360<a<380:
                            a=31
                            self.pub8.publish(Float64(160))
                            self.pub0.publish(Float64(a))
                            print(a)
                                                  
                        if 380<a<400:
                            a=31
                            self.pub8.publish(Float64(160))
                            self.pub0.publish(Float64(a))
                            print(a)            

                        if 400<a<420:
                            a=32
                            self.pub8.publish(Float64(160))
                            self.pub0.publish(Float64(a))
                            print(a)                                     
                           

                        if 420<a<470:
                            a=32
                            self.pub8.publish(Float64(160))
                            self.pub0.publish(Float64(a))
                            print(a)      

                        self.sayac=self.sayac+1
                        self.pub15.publish(Float64(1))
                        if self.sayac==3:
                            self.kilit3=0
                            self.sayac=4
            threading.Thread(target=self.gripper).start()  
            threading.Thread(target=self.io_test).start()
            threading.Thread(target=self.ayiklama).start()
            threading.Thread(target=self.basadon).start()
              
            cv2.imshow("Image window", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            print(e)

    def gripper(self):  
        time.sleep(9)

        
    
        if self.pozition1==1:
            self.pub4.publish(Float64(0.01))
            self.pub10.publish(Float64(2750))
            self.pub5.publish(Float64(0.01))


       
    def io_test(self):
        time.sleep(20)
        if self.kır ==0:
            """
            self.pub12.publish(Float64(0))
            self.pub13.publish(Float64(0))
            self.pub14.publish(Float64(0))

            """

            self.pub8.publish(Float64(0))
            self.pub6.publish(Float64(0))
            #self.pub7.publish(Float64(0))   
            a = int(-0.75/ 0.00473684211)
            #self.pub11.publish(Float64(0))  

    def ayiklama(self):

        time.sleep(40)
        self.kır=1
        if self.kır1==0:

            if self.ayiklama1 ==1 :
                self.ol=1
                self.pub7.publish(Float64(-1.57))
                a = int(-1.57/ 0.00473684211)
                self.pub11.publish(Float64(a))  
           
                self.pozition1=0     
                        
               
                self.pub4.publish(Float64(0.0))
                self.pub5.publish(Float64(0.0)) 
      
     
            if self.ayiklama1 ==2 : 
                self.ol=1
                self.pub7.publish(Float64(0.57)) 
                b = int(0.57/ 0.00473684211)
                self.pub11.publish(Float64(b))  
            
                self.pozition1=0     
               
                self.pub4.publish(Float64(0.0))
                self.pub5.publish(Float64(0.0)) 

            if self.ol==1:
                time.sleep(3)
                self.pub8.publish(Float64(140))
                self.pub6.publish(Float64(50))
                time.sleep(6)
                self.pub10.publish(Float64(0))

 
    def basadon(self):
        time.sleep(55)
        self.kır1=1
        self.kır1

        self.pub8.publish(Float64(160))
        self.pub9.publish(Float64(45))
        self.pub7.publish(Float64(0)) 
        self.pub11.publish(Float64(0))        



    def run(self):
        rospy.spin()
        
     

if __name__ == '__main__':
    image_processor = ImageProcessor()
    image_processor.run()  
