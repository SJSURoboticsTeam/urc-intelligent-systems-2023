# DepthAI Camera Installation

### Ubuntu
```bash
sudo wget -qO- https://docs.luxonis.com/install_dependencies.sh | bash
```
> Note! If opencv fails with illegal instruction after installing from PyPi, add:
    >echo "export OPENBLAS_CORETYPE=ARMV8" >> ~/.bashrc
    >source ~/.bashrc

### Windows (All Requirements)
Reccomeneded to use Python 3.9-3.10
```bash
git clone https://github.com/luxonis/depthai.git
cd depthai
python -m pip install -r requirements.txt --user
python depthai_demo.py
```


### Raspberry Pi
```bash
sudo curl -fL https://docs.luxonis.com/install_dependencies.sh | bash
```

# Requirements Installation for RPI and Ubuntu
To install this package, run the following command in your terminal window.
**Use either pip or pip3 depending on your Python version**
```bash
pip3 install -r requirements.txt
python3 -m pip install numpy opencv-python depthai blobconverter --user
```

# Issues Running
If you have issues running the code, or recieve  a system error such as *Attempted to start Color camera - NOT detected!* , try running the following command in your terminal window.
```bash
python3 -m pip install --extra-index-url https://artifacts.luxonis.com/artifactory/luxonis-python-snapshot-local depthai==2.16.0.0.dev+9d866d9fa131a151ea592edb59b3d5e60649aef1
```

# Documentation
For more information on the DepthAI API, please visit the [API documentation](https://docs.luxonis.com/projects/api/en/latest/) or the [SDK Documentation](https://docs.luxonis.com/projects/sdk/en/latest/) page.