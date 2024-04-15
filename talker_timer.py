import rospy
from std_msgs.msg import String, Float64,Int32
import serial
import threading 
import time

class SimpleSubscriber:
    def __init__(self):
        self.motor1 = 0
        self.motor2 = 0
        self.motor3 = 0
        self.motor4 = 0
        self.motor5 = 0
        self.motor6 = 0
        self.deger=0

        self.ser = serial.Serial('/dev/ttyACM0', 115200)
        rospy.init_node('simple_listener', anonymous=True)
        self.subscriber1 = rospy.Subscriber("/Alt_motor", Float64, self.alt_motor)
        self.subscriber2 = rospy.Subscriber("/gripper", Float64, self.gripper_motor)
        self.subscriber3 = rospy.Subscriber("/Motor1", Int32, self.Motor1_motor)
        self.subscriber4 = rospy.Subscriber("/Motor2", Int32, self.Motor2_motor)
        self.subscriber5 = rospy.Subscriber("/Motor3", Int32, self.Motor3_motor)
        threading.Thread(target=self.print_hello_world, daemon=True).start()
        self.subscriber6 = rospy.Subscriber("/bilek", Float64, self.Motor5_motor)
        #self.subscriber7 = rospy.Subscriber("/bekle/motor", Float64, self.bekle_motor)
        # Registering the shutdown callback
        rospy.on_shutdown(self.shutdown_callback)

    def alt_motor(self, data):
        self.motor1 = int((data.data)/0.01655)
        
        
    def bekle_motor(self,data):
        self.deger=data.data


    def Motor1_motor(self, data):
        if self.deger==0:
            self.motor2 = int(data.data)
            self.motor2 = int(self.motor2)

        if self.deger==1:
            time.sleep(1.5)    
            self.motor2 = int(data.data)
            self.motor2 = int(self.motor2)
    def Motor2_motor(self, data):
        self.motor3 = int(data.data)
        self.motor3 = int(self.motor3)
    def Motor3_motor(self, data):
        self.motor4 = int(data.data)

    def Motor5_motor(self, data):
        if data.data==1:
           time.sleep(0.5)
           self.motor5=int(self.motor1/-200) 

    def gripper_motor(self, data):
        self.motor6 =int(data.data)

    def print_hello_world(self):
        while not rospy.is_shutdown():
            self.veri = "{},{},{},{},{},{}\n".format(str(self.motor1), str(self.motor2), str(self.motor3),
                                                    str(self.motor4), str(self.motor5), str(self.motor6))
            print(self.veri)
            self.ser.write(self.veri.encode())
            time.sleep(0.5)  # Bir saniye bekle

    def start_listening(self):
        rospy.spin()

    def shutdown_callback(self):
        print("Shutting down...")
        self.ser.close()  # Closing the serial port

if __name__ == '__main__':
    simple_sub = SimpleSubscriber()
    simple_sub.start_listening()
