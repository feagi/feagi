# To run Godot-Bridge as a standalone python application

## Windows
1) `git clone https://github.com/feagi/feagi-connector.git`
2) `cd embodiments/godot-bridge`
3) `python -m venv venv`
4) `venv\Scripts\activate`
5) `pip install -r requirements.txt`
6) `python bridge_godot_python.py`

## Linux or Mac
1) `git clone https://github.com/feagi/feagi-connector.git`
2) `cd embodiments/godot-bridge`
3) `python3 -m venv venv`
4) `source venv/bin/activate`
5) `pip3 install -r requirements.txt`
6) `python3 bridge_godot_python.py`

# Important information
This depends on FEAGI. If FEAGI is not started, the bridge will not do anything. To launch FEAGI, see here: https://github.com/feagi/feagi/wiki/Deployment#deployment-options

To launch Brain visualizer, see here:  https://github.com/feagi/brain-visualizer/blob/staging/DEPLOY.md