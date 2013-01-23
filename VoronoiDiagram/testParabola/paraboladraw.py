INTERVAL = 1000


def parabolafun(focusx, focusy, ly, x):
	assert(focusy>ly)
	distance = focusy - ly
	result = 1.0*(x-focusx)*(x-focusx)/(2*distance) + 0.5*(ly+focusy)
	#print "%d  %d"  %(x,  result)
	return result

def paraboladraw(filename, focusx, focusy, ly, xmin, xmax):
	fo = open(filename,'w');
	x = xmin;
	deltax = 1.0*(xmax-xmin)/INTERVAL
	
	for i in range(INTERVAL):
		y = parabolafun(focusx, focusy, ly, x)
		fo.write("%f    %f\n" %(x, y))
		x = x+deltax
		
	
	fo.close()
	

paraboladraw("p0--1.txt", 0, 4, 1, -4, 4)

paraboladraw("p0--0.txt", 0, 4, 0, -4, 4)

paraboladraw("p2--0.txt", 2, 1, 0, -4, 4)

ly3 = -0.2023796
paraboladraw("p0--0.2.txt", 0, 4, ly3, -40, 40)
paraboladraw("p1--0.2.txt", 2, 1, ly3, -40, 40)
paraboladraw("p2--0.2.txt", 1, 0, ly3, -40, 40)


ly3 = -1
paraboladraw("p0---1.txt", 0, 4, ly3, -40, 40)
paraboladraw("p1---1.txt", 2, 1, ly3, -40, 40)
paraboladraw("p2---1.txt", 1, 0, ly3, -40, 40)