
from datetime import datetime

class Log(object):
    def __init__(self) -> None:
        self.name_file = self.get_name_file()
        self.mode = "w+"
        self.file = open(self.name_file, self.mode)

    def get_name_file(self):
        now = datetime.now()
        return "log/" + str(now.strftime("%d%m%Y-%H;%M;%S")) + ".txt"
    
    def log(self, key, text):
        """
        Possible values for key: DEBUG, SYSTEM, COMMAND, SENSOR, LOG
        text: string value to write in the file on a new line
        """
        now = datetime.now()
        time = now.time()
        
        add_space = ""
        for i in range(9 - len(key)):
            add_space += " "
        key += add_space
        
        self.file.write("Time: {}| {}| {}\n".format(time, key, text))

    def close_log(self):
        self.file.close()
