"# Inverse-Kinematics" 

Uses gradient descent with the pseudoinverse of the jacobian to solve the inverse kinematic problem for finding robot angles to reach a point in space. 

## Installation

```
git clone git@github.com:viktorgus/Inverse-Kinematics.git
pip install -r requirements.txt
python robot.py
```

## Info
Used for project course TSBB17. The project was to make a robot sort lego fully automatically using the following pipeline:
- Detection of centre of lego pieces in picture with color segmentation
- Translation of image coordinates to real world coordinates using the estimated homography matrix
- Translate point into robot axis rotations using inverse kinematics to move robot to lego pieces (THIS FILE)


