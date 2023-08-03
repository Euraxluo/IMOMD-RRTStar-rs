import unittest
from IMOMD_RRTStar import *


class Test(unittest.TestCase):
    def test_demo(self):
        result = sum_as_string(2, 3)
        print(result)
        assert result == '5'

    def test_df(self):
        import polars as pl
        data = {
            'Name': ['Alice', 'Bob', 'Charlie'],
            'Age': [25, 30, 22],
            'City': ['New York', 'San Francisco', 'Seattle']
        }
        print(data)
        df = pl.DataFrame(data)
        print(df)
        arrow_table = df.to_arrow()
        record_batches = arrow_table.to_batches()
        print("")
        print("arrow_table")
        print("")
        print(arrow_table)
        print("")
        print("schema")
        print("")
        print(arrow_table.schema)
        print("")
        print("columns")
        print("")
        print(arrow_table.columns)
        print("")
        print("record_batches")
        print("")
        print(record_batches)


        print("")
        print("#"*100)
        print("call in rust")
        print("")
    
        result = process_pyarrow_table(arrow_table)
        
        
        print("")
        print("#"*100)
        print("")
        print(result)
