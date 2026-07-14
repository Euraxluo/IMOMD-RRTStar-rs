# Contributing

Thank you for interest in improving this reproduction. Contributions that
strengthen **correctness**, **reproducibility**, or **documentation clarity**
are especially welcome.

## Development setup

```bash
# Rust toolchain (stable)
cargo test --all-targets
cargo clippy --all-targets -- -D warnings

# Python bindings (3.8–3.13)
python -m venv .venv
.venv/bin/pip install maturin
PYO3_USE_ABI3_FORWARD_COMPATIBILITY=1 \
  .venv/bin/maturin develop --release --features python,extension-module
.venv/bin/python -m unittest discover test -v
```

Optional demo dependencies are listed under `demo/`.

## Guidelines

1. Prefer small, focused pull requests with a clear problem statement.
2. Add or update tests when changing planner, navigation, or verification logic.
3. Keep the public Rust / Python API stable unless the PR explicitly documents a
   breaking change.
4. Do not commit large OSM extracts, binary maps, or local experiment dumps
   (`tmp/`, `experiments/`).
5. Match existing code style; run `cargo fmt` before submitting.

## Reporting issues

Please include:

- OS and Rust / Python versions;
- minimal config or map that reproduces the issue;
- expected vs actual cost / path / error message;
- whether the C++ reference behaves differently on the same instance (if known).

## Research use

If results from this repository appear in a paper or report, please cite the
original IMOMD-RRT* publication and this software (see `CITATION.cff` and the
README citation section).
