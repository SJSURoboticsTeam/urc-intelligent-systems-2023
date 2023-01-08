# RPI-GPS

## Configuration
Open the run.py file and edit the port `"/dev/ttyACM0"` to match your connection:
> If you are on Windows, you will need to change the `"/dev/ttyACM0"` port to something like `"COM0"`.
```python
    data = gpsRead("/dev/ttyACM0",9600)
```

## RUN
```bash
python3 run.py
```