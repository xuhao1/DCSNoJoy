import os.path
from Configs.configs import *
from yaml import load, dump
try:
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
    from yaml import Loader, Dumper

class ParameterServer:
    def __init__(self, path):
        self.path = path/ "Configs/"
        print("path", path, self.path)
        self.data = {}

    def load_aircraft(self, name):
        fname = name + ".yaml"
        print(f"Attemp Read config file from {self.path / fname}")
        if os.path.isfile(self.path / fname):
            stream = open(self.path / fname, 'r')
            _data = load(stream, Loader=Loader)
            for k in _data:
                self.data[k] = _data[k]
                setattr(self, k, self.data[k])
            # Enable this make config sync to default if the value is not set
            # self.dump_aircraft(fname)
        else:
            print(f"Config file {self.path / fname} not exist, create from default")
            if os.path.isfile(self.path / (default_config_file + ".yaml")):
                self.load_aircraft(default_config_file)
                self.dump_aircraft(fname)
            else:
                print(f"Default config file {self.path / (default_config_file / '.yaml')} not exist!")
                exit(-1)

    def dump_aircraft(self, name):
        stream = open(self.path / name, 'w')
        output = dump(self.data, Dumper=Dumper)
        print(f"Write config file to {self.path / name}")
        print(output, file=stream)