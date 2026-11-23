.PHONY: venv modelo basico v21 test lint firmware

PY ?= .venv/bin/python

venv:
	python3 -m venv .venv && $(PY) -m pip install -r requirements.txt

modelo:
	curl -L -o src/yolo11n.pt https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.pt

basico:
	$(PY) src/gear_ai_rover_basico.py

v21:
	$(PY) src/gear_ai_rover_v21.py

test:
	python3 -m pytest -q

lint:
	python3 -m ruff check . || true

firmware:
	cd firmware/arduino-uno/SmartRobotCar_Main_V1 && arduino-cli compile --profile uno
