# IMOMD-RRTStar-rs

Informable Multi-Objective and Multi-Directional RRT* System for Path Planning.This work reimplemented an anytime
iterative system to concurrently solve the multi-objective path planning problem and determine the visiting order of
destinations using rust-lang.related paper:https://arxiv.org/abs/2205.14853

## build

build by [Maturin](https://www.maturin.rs/)

## dev

1. create project `maturin new project`
2. publish project `maturin publish`
3. build python wheel `maturin build`
4. Custom python source directory, set by cargo.toml`package.metadata.maturin.python-source` 指定
    ```toml
   [package.metadata.maturin]
   python-source = "python"
   ```
5. set `lib.ceate-type` Make it also as a rust lib
   ```toml
   [lib]
   crate-type = ["cdylib","rlib"]
   ```
6. set pyproject.toml,set build system
   ```toml
   [build-system]
    requires = ["maturin>=0.13,<0.14"]
    build-backend = "maturin"
   ```

## Run and Build

1. project dev mod build. require py>3.7
   ```bash
   maturin develop
   ```
2. rust code test
   `cargo test`
3. python code test
   `python -m unittest discover test`