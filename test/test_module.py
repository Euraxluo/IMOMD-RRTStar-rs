import unittest

# Legacy PyArrow demo tests — require `maturin develop --features pyarrow-demo`
try:
    from IMOMD_RRTStar import sum_as_string, process_pyarrow_table
    HAS_PYARROW_DEMO = True
except ImportError:
    HAS_PYARROW_DEMO = False


@unittest.skipUnless(HAS_PYARROW_DEMO, "pyarrow-demo feature not enabled")
class TestLegacyPyArrow(unittest.TestCase):
    def test_demo(self):
        result = sum_as_string(2, 3)
        self.assertEqual(result, "5")

    def test_df(self):
        import polars as pl

        data = {
            "Name": ["Alice", "Bob", "Charlie"],
            "Age": [25, 30, 22],
            "City": ["New York", "San Francisco", "Seattle"],
        }
        df = pl.DataFrame(data)
        arrow_table = df.to_arrow()
        process_pyarrow_table(arrow_table)


if __name__ == "__main__":
    unittest.main()
