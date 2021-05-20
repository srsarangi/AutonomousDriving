## Requirements:
	python3.x
	opencv-contrib-python==3.4.4.19

Download goturn.caffemodel from
https://www.dropbox.com/sh/77frbrkmf9ojfm6/AACgY7-wSfj-LIyYcOgUSZ0Ua?dl=0 into this folder.

Can't add caffemodel in git repo due to its large size

## Run GoTurn Tracker

```
python3 goturnTracker.py path/to/video/file
```

## Run Kalman Tracker

```
python3 kalman_tracker.py path/to/video/file
```