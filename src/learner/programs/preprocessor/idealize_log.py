'''
Created on Nov 26, 2012

@author: adam
Requires a processed log and of slightly larger resolution than output. 
Extracts two sub images from the Y0 image and two sub images from the Y1 image   
'''
import sys
from optparse import OptionParser

# class IdealizeLog():
#    def __init__(self):
#        pass
    
def main(args):
    parser = OptionParser()
    parser.add_option("-s", "--size", help="Output resolution of log")
    parser.add_option("-i", "--inputs",
                      help="Input log files, separated by comma")
    options = parser.parse_options()
    inputs = options.inputs.split(',') 
    for x in inputs:
        outname = (input[:x.index('-')] + '-ideal' + 
                   input[x.index('-'):])
        
        
    
if __name__ == '__main__':
    main(sys.argv)
