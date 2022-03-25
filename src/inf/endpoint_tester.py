import requests
import contextlib

url = "http://127.0.0.1:7000/v1/feagi"
request_obj = {'begin': True}

x = requests.post(url, data=request_obj)
print(x.text)

test = Config()