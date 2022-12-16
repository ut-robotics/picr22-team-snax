import websocket
import time
import json
import multiprocessing

class Referee:
    def __init__(self, ip, robotName = "snax"):
        self.robotName = robotName
        self.serverURL = "ws://" + ip + ":8222"
        self.ws = websocket.WebSocket()
        self.commands = multiprocessing.Queue()

    def connect(self):
        print("trying to connect to " + self.serverURL)
        for i in range(10):
            try:
                self.ws.connect(self.serverURL)
                print("connection to referee successful")
                return True
            except:
                print("connection failed. retrying in 5 s")
                time.sleep(5)
        return False

    def listen(self):
        while True:
            try:
                command = self.ws.recv()
            
            #reconnecting
            except:
                if self.connect():
                    continue
                else:
                    self.disconnect()
                    break
            self.commands.put(json.loads(command))

    def getCommand(self):
        try:
            cmd = self.commands.get_nowait()
            if self.robotName in cmd['targets']:
                if cmd['signal'] == 'start':
                    return ("START", cmd['baskets'][cmd['targets'].index(self.robotName)])
                else:
                    return ("STOP", 0)
        except:
            return None

    def disconnect(self):
        self.listener.join()
        self.ws.close()

    def startReferee(self):
        if self.connect():
                self.listener = multiprocessing.Process(target = self.listen, args=())
                self.listener.start()


if __name__ == "__main__":
    testRef = Referee(ip="localhost")
    testRef.startReferee()
    try:
        while True:
            cmd = testRef.getCommand()
            if (cmd != None):
                print(cmd)

    except KeyboardInterrupt:
        testRef.disconnect()
    finally:
        print('connection closed')
