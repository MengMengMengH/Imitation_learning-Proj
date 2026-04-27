import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/hanmg/Imitation_learning-Proj/infer_ws/install/infer_client'
