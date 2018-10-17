import socket

class tcp_client:
    '''
    Class which creates a TCP client meant to send data to a server.
    Note that it is the user's responsibility to format the sent data correctly.
    '''

    def __init__(self, HOSTNAME, PORT):

        try:
            self.s = socket.create_connection((HOSTNAME, PORT))
        except OSError:
            print('Unable to connect to reader. Check connection and IP address of reader.')
            quit()

        # Set the socket to non-blocking
        #self.s.setblocking(1)

        # Make a file pointer from the socket
        self.fs=self.s.makefile(mode='rw')


    def read_msg(self):
        while 1:
            line = fs.readline()
            # If data was received, print it
            if (len(line)):
                n = n+1
                print(str(n)+': '+str(datetime.datetime.now().time())+'     '+line)

    def send_msg(self,msg):
        self.s.send(msg)

if __name__ == '__main__':
    my_client = tcp_client('0.0.0.0',14150)
    countx = 0
    county = 0
    countz = 0
    while True:
        msg = '{},{},{} '.format(countx,county,countz)
        my_client.send_msg(msg.encode('utf-8'))
        input(msg)
        countx = (countx+1) % 10
        county = (county+2) % 10
        countz = (countz+3) % 10
