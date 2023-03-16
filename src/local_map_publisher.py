  #!/usr/bin/env python
  import rospy
  from std_msgs.msg import Float32
  from publisher_controlvalue import ControlValuePub

  class MySubscriber(object):

   def __init__(self):

    # TODO : do a wait topic        
    self.encoder = 0.0
    self.desiredcounts = 0.0


    rospy.Subscriber('desiredcounts', Float32, self.desiredcounts_callback)
    rospy.Subscriber('encoder', Float32, self.encoder_callback)
    self.contr_obj = ControlValuePub(kp=1, kd=0.1, dt=10)


def encoder_callback(self,msg):
    rospy.loginfo('got encoder %f', msg.data)
    self.encoder = msg.data

def desiredcounts_callback(self,msg):
    # This callback is the boss, this one dictates the publish rate
    rospy.loginfo('got desiredcounts %f', msg.data)
    self.desiredcounts = msg.data
    self.contr_obj.publish_controlvalue(desiredv=self.desiredcounts,
                                        encoderv=self.encoder)


def loop(self):
    rospy.logwarn("Starting Loop...")
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('subscriber_node', anonymous=True, log_level=rospy.WARN)
    my_subs = MySubscriber()
    my_subs.loop()