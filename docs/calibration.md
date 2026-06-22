# Calibration

# Compiling ApertureComputeModule code after adjusting for calibration
```
clear; ~/DASPi/software/scripts/build-aperturecomputemodule.sh
```
```
meson compile -C ~/DASPi/software/build/computemodule-native
```
# ChArUco board
[Parametric 3D-Printable ChArUco Calibration Target](https://github.com/jywilson2/charuco)

#Gather ChArUco calibration board captures
```
rm -f ~/DASPi/software/config/charuco-captures/test_retry_001-*

SESSION=test_retry_001 \
POSES=2 \
SECONDS_PER_POSE=20 \
MIN_ROWS_PER_POSE=300 \
MIN_CORNERS_PER_FRAME=30 \
MAX_RETRIES_PER_POSE=5 \
~/DASPi/software/scripts/seam_image_collection_calibration.sh
```
# Running calibration solver
```
NOMINAL_RVEC_DEG="64.227976159,0.000000000,-168.151024613"

~/DASPi/software/build/computemodule-native/computemodule/daspi-calibrate-charuco \
  --observations=$HOME/DASPi/software/config/daspi-charuco-observations-module0.csv,$HOME/DASPi/software/config/daspi-charuco-observations-module1.csv \
  --poseCsv=$HOME/DASPi/software/config/charuco-pose-report-calibrated.csv \
  --relativePoseCsv=$HOME/DASPi/software/config/relative-camera-pose-report.csv \
  --writeCalibration=$HOME/DASPi/software/config/camera-calibration-generated.txt \
  --nominalRelativeRvecDeg="$NOMINAL_RVEC_DEG" \
  --squaresX=15 \
  --squaresY=15 \
  --squareLength=0.030 \
  --markerLength=0.023 \
  --dictionary=DICT_4X4_250 \
  --fx=600 \
  --fy=600 \
  --cx=728 \
  --cy=544 \
  --minCorners=20 \
  --calibrateIntrinsics \
  --imageWidth=1456 \
  --imageHeight=1088 \
  --intrinsicsPrefix=$HOME/DASPi/software/config/camera-intrinsics
```


# Compiling 

Copy and check calibration output
```
cp ~/DASPi/software/config/camera-calibration-generated.txt    ~/DASPi/software/config/camera-calibration.txt
cat ~/DASPi/software/config/camera-calibration.txt
```

# Copying calibration output to ApertureComputeModule nodes
```
for host in 10.0.2.4 10.0.2.5; do   scp ~/DASPi/software/config/camera-calibration.txt daspi@$host:/tmp/;   scp ~/DASPi/software/config/camera-intrinsics-module0.yml daspi@$host:/tmp/;   scp ~/DASPi/software/config/camera-intrinsics-module1.yml daspi@$host:/tmp/;    ssh daspi@$host '
    sudo mkdir -p /opt/daspi/config
    sudo cp /tmp/camera-calibration.txt /opt/daspi/config/
    sudo cp /tmp/camera-intrinsics-module0.yml /opt/daspi/config/
    sudo cp /tmp/camera-intrinsics-module1.yml /opt/daspi/config/
    sudo chown daspi:daspi /opt/daspi/config/camera-calibration.txt /opt/daspi/config/camera-intrinsics-module*.yml
    ls -lh /opt/daspi/config/
  '; done
```

# Running on ApertureComputeModule000
```
/opt/daspi/aperturecomputemodule   --clientIp=10.0.2.1   --port=5000   --moduleIndex=1   --cameraCalibration=/opt/daspi/config/camera-calibration.txt --cameraIntrinsicsPrefix=/opt/daspi/config/camera-intrinsics
```

# Running on ApertureComputeModule0001
```
/opt/daspi/aperturecomputemodule   --clientIp=10.0.2.1   --port=5010   --moduleIndex=0   --cameraCalibration=/opt/daspi/config/camera-calibration.txt --cameraIntrinsicsPrefix=/opt/daspi/config/camera-intrinsics
```

# Running on laptop
```
~/DASPi/software/build/computemodule-native/computemodule/computemodule   --nApertureComputeModules=2   --usbBaseIp=10.0.2.1   --serverIps=10.0.2.4,10.0.2.5   --port=5000   --cameraCalibration=$HOME/DASPi/software/config/camera-calibration.txt
```

