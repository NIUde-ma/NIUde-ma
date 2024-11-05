import os

host_name = os.popen("hostname").read().strip()
if host_name == "testServer":
    print("True")
else:
    print("False")
