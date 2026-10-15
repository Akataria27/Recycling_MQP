import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/merlabros2/Recycling_MQP/src/install/beckhoff_good'
