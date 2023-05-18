from roslibpy import Topic, Message

class TwistPublisher:
    def __init__(self, ros, topic_name):
        self.topic_name = topic_name
        self.topic = Topic(ros=ros, name=topic_name, message_type='geometry_msgs/Twist')

    def publish(self, angle, speed):
        from random import random
        twist = {
            'linear': {
                'x': random(),
                'y': random(),
                'z': random()
            },
            'angular': {
                'x': random(),
                'y': random(),
                'z': random()
            },
        }
        self.topic.publish(Message(twist))

    def destroy(self):
        self.topic.unadvertise()