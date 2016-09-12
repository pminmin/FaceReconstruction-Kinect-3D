import subprocess as sub
import sys
import time

personName = sys.argv[1]

kinectPro = 'demo.exe ' + personName
registerPro = 'register.exe ' + personName

start = time.time()
kinect_code = sub.call(kinectPro, shell = True)
if(kinect_code == 0):
	print "kinect valid"
	register_code = sub.call(registerPro, shell = True)
	if(register_code == 0):
		print "register valid"
end = time.time()
print "Running time is: %fs" % (end - start)