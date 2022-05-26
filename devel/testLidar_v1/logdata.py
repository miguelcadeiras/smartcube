import sys
from io import StringIO

runningStandalone=False
logPipe=None


def log(*argv):
	outStrIO=StringIO()
	if (runningStandalone==False):
		sys.stdout=outStrIO
		print (*argv)
		logPipe.send (outStrIO.getvalue())
		outStrIO.close()
	else:
		print ("STANDALONE:", *argv)
	sys.stdout=sys.__stdout__
