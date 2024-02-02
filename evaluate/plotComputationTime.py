import glob
import numpy as np
from matplotlib import pyplot


txt_file = 'log.txt'



# Using readlines() 
file = open(txt_file, 'r') 
Lines = file.readlines() 
tracking_time = []
M2DP_time = []
localMapping_time = []
loopClosing_time = []
count = -1
# Strips the newline character 
for line in Lines: 
	if line.startswith('The total number of images is'):
		line = line.replace('The total number of images is','')
		number_of_frame = int(line)
		tracking_time = np.zeros(number_of_frame-1)
		M2DP_time = np.zeros(number_of_frame)
		localMapping_time = np.zeros(number_of_frame)
		loopClosing_time = np.zeros(number_of_frame)

	if line.startswith('cart_image_path:'):
		count+=1	
    # print(line.strip()) 
    # print("Line{}: {}".format(count, line.strip()))
	if line.startswith('Tracking computation time is:'):
		line = line.replace('Tracking computation time is:', '')
		line = line.replace('milliseconds.', '')		# print line
		tracking_time[count] = (float(line))
	# elif line.startswith('LocalMapping computation time is:'):
	# 	line = line.replace("milliseconds.", "")		
	# 	line = line.replace("LocalMapping computation time is:", "")
	# 	localMapping_time[count] = (float(line))

	# elif line.startswith('M2DP computation time is:'):
	# 	line = line.replace("M2DP computation time is:", "")
	# 	line = line.replace("milliseconds.", "")
	# 	M2DP_time[count] = (float(line))

	# elif line.startswith('LoopClosing computation time is:'):
	# 	origin_line = line
	# 	line = line.replace("LoopClosing computation time is:", "")
	# 	line = line.replace("milliseconds.", "")
	# 	loopClosing_time[count] = (float(line))
	# 	if float(line) > 200:
	# 		print count
	# 		print origin_line

	else:
		continue

# print t.shape
# print tracking_time.shape
# print tracking_time
bins = np.linspace(1, 1000, number_of_frame)
# print bins
# print bins.shape

# pyplot.hist(list(tracking_time), bins, alpha=0.5, label='x')
# pyplot.hist(list(loopClosing_time), bins, alpha=0.5, label='y')
# pyplot.hist(list(localMapping_time), bins, alpha=0.5, label='z')
t = np.arange(1, number_of_frame, 1)

fontSize = 32
pyplot.rcParams.update({'font.size': 22})

pyplot.plot(t,list(tracking_time))
pyplot.ylabel('Milliseconds', fontsize=fontSize)
pyplot.xlabel('Frame', fontsize=fontSize)
pyplot.title("Tracking thread computation time", fontsize=fontSize) 

pyplot.show()

