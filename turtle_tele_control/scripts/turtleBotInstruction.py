#!/usr/bin/env python
import requests
url = r"http://wangtong15.com:10005/static/data/control.json"

def getControl():
    hd = {'user-agent':'Chrome/10'}
    r = requests.get(url,headers=hd)
    r.raise_for_status()
    r.encoding=r.apparent_encoding
    data = eval(r.text)
    return data

if __name__ == "__main__":
    while 1:
        print(getControl())
