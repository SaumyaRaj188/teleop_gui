import roslibpy

if __name__ == '__main__':
    client = roslibpy.Ros(host='localhost', port=9090)
    client.run()

    listener = roslibpy.Topic(client, '/cmd_vel', 'geometry_msgs/Twist')
    listener.subscribe(lambda message: print('Received: ', message))

    try:
        while True:
            pass
    except KeyboardInterrupt:
        client.terminate()