from roslibpy import Topic, Message
from math import pi, atan2

class TwistPublisher:
    def __init__(self, ros, topic_name):
        self.topic_name = topic_name
        self.topic = Topic(ros=ros, name=topic_name, message_type='geometry_msgs/Twist')

    def publish(self, angle, speed):
        if angle.lengthSquared() < 0.01:
            rot = 0
        else:
            angle = angle.normalized()
            rot = atan2(angle.y(), angle.x()) - pi/2

        twist = {
            'linear': {
                'x': speed.y(),
                'y': 0,
                'z': 0,
            },
            'angular': {
                'x': 0,
                'y': 0,
                'z': rot
            },
        }
        self.topic.publish(Message(twist))

    def destroy(self):
        self.topic.unadvertise()