from typing import Any, Union, List, Iterator, Tuple, Dict
import pyarrow as pa


def sum_as_string(a: int, b: int) -> str: ...


def process_pyarrow_table(df: pa.Table) -> int: ...


__all__ = ["sum_as_string",
           "process_pyarrow_table"
           ]
