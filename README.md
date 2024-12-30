<div align="center"><img alt="panda-py Logo" src="https://raw.githubusercontent.com/JeanElsner/panda-py/main/logo.jpg" /></div>

<h1 align="center">panda-py for Jad</h1>

<p align="center">
  <a href="https://github.com/JeanElsner/panda-py/actions/workflows/main.yml"><img alt="Continuous Integration" src="https://img.shields.io/github/actions/workflow/status/JeanElsner/panda-py/main.yml" /></a>
  <a href="https://github.com/JeanElsner/panda-py/blob/main/LICENSE"><img alt="Apache-2.0 License" src="https://img.shields.io/github/license/JeanElsner/panda-py" /></a>
  <a href="https://pypi.org/project/panda-python/"><img alt="PyPI - Published Version" src="https://img.shields.io/pypi/v/panda-python"></a>
  <img alt="PyPI - Python Version" src="https://img.shields.io/pypi/pyversions/panda-python">
  <a href="https://jeanelsner.github.io/panda-py"><img alt="Documentation" src="https://shields.io/badge/-Documentation-informational" /><a/>
</p>

Finally, Python bindings for the Panda. These will increase your productivity by 1000%, guaranteed[^1]! 

## New Stuff
Forked [panda-py](https://github.com/JeanElsner/panda-py) and added joint and cartesian controllers using inbuilt generators. Also made the calls async using [trio](https://github.com/python-trio/trio).



## Install
```
pip install git+https://github.com/jc211/panda-py
```

# Citation

If you use panda-py in published research, please consider citing the [original software paper](https://www.sciencedirect.com/science/article/pii/S2352711023002285).

```
@article{elsner2023taming,
title = {Taming the Panda with Python: A powerful duo for seamless robotics programming and integration},
journal = {SoftwareX},
volume = {24},
pages = {101532},
year = {2023},
issn = {2352-7110},
doi = {https://doi.org/10.1016/j.softx.2023.101532},
url = {https://www.sciencedirect.com/science/article/pii/S2352711023002285},
author = {Jean Elsner}
}
```

[^1]: Not actually guaranteed. Based on a sample size of one.
