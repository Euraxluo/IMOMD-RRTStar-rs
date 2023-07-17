import unittest
from IMOMD_RRTStar import *


class Test(unittest.TestCase):
    def test_demo(self):
        result = sum_as_string(2, 3)
        print(result)
        assert result == '5'
