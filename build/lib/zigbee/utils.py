import threading

class zigBeeError(Exception):
    pass

class InvalidSerializeArg(Exception):
    pass

class InvalidParsingArgument(Exception):
    pass

def parse(raw_data):
    func = str(int(raw_data[:2], 16))
    raw_data = raw_data[2:]
    wr = str(int(raw_data[:2], 16))
    raw_data = raw_data[2:]
    size = str(int(raw_data[:2], 16))
    raw_data = raw_data[2:]
    data = bytes.fromhex(raw_data).decode('utf-8')
    
    return func, wr, size, data

def serialize(func, wr, size, data):
    if int(func) < 1 or int(func) > 11:
        raise InvalidSerializeArg
    if int(wr) != 0 and int(wr) != 1:
        raise InvalidSerializeArg
    
    data = str(func) + str(wr) + str(size) + str(data)

    return data
