from getch import getch

while True:
	key1 = getch()
	if (ord(key1) != 27):
		continue
	#print('First char: {}'.format(ord(key)))
	key2 = getch()
	if (ord(key2) != 91):
		continue
	#print('Second char: {}'.format(ord(key)))
	key3 = getch()
	#print('Third char: {}'.format(ord(key)))

	if (ord(key3) == 65):
		print('Up')
		continue
	if (ord(key3) == 66):
		print('Down')
		continue
	if (ord(key3) == 67):
		print('Right')
		continue
	if (ord(key3) == 68):
		print('Left')
