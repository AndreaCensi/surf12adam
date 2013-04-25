from PIL import Image  # @UnresolvedImport
from boot_agents.diffeo.diffeo_display import (diffeo_to_rgb_angle,
    diffeo_to_rgb_norm)
from optparse import OptionParser
import numpy as np
import pdb
import pickle

#
# def test_diffeo(argv):
    

usage = "usage: %prog args"
parser = OptionParser(usage=usage, version="%prog 1.0")
parser.add_option("-i", "--input", default='/media/data/learned-diffeo/', help="Diffeomorphism pickle")
parser.add_option("--im", "--image", default='', help="Image file")
parser.add_option("-s", "--size", default='/media/data/processed-data/', help="Image size")
parser.add_option("-o", "--outpath", default='/media/data/out_diff/', help='Output path')
parser.add_option("-l", "--levels", default=5, help="Number of steps to apply to image.")
parser.add_option("-p", "--prefix", default='', help="Output files prefix")
options, _ = parser.parse_args()
dfile = options.input
outpath = options.outpath
infile = options.im
levels = options.levels
prefix = options.prefix
size = eval(options.size)


#    # Find diffeomorphisms file
#    try:
#        dfile = argv[argv.index('-dl')+1]
#    except ValueError:
#        dfile = '/media/data/learned_diffeo/camera_ptz.dynamics.pickle'
print 'Using diffeomorphism from file:    ', dfile

#    # Find diffeomorphisms file
#    try:
#        outpath = argv[argv.index('-o')+1]
#    except ValueError:
#        outpath = '/media/data/tempimages/'
print 'Saving images to path:            ', outpath

#    # Find output prefix file
#    try:
#        prefix = argv[argv.index('-p')+1]
#    except ValueError:
#        prefix = 'logitech_cam_ptz'
print 'Using prefix for output files:    ', prefix

#    # Find Input image
#    try:
#        infile = argv[argv.index('-im')+1]
#    except ValueError:
#        infile = '/home/adam/git/surf12adam/diffeo_experiments/lighttower640.jpg'
print 'Using image file:                ', infile

#    # Number of levels to apply
#    try:
#        levels = int(argv[argv.index('-l')+1])
#    except ValueError:
#        levels = 5
print 'Applying diffeomorphism ', levels, ' times'

#    # Image size
#    try:
#        size = eval(argv[argv.index('-size')+1])
#    except ValueError:
#        size = [160,120]
print 'Image size:    ', size


logfile = open(outpath + prefix + 'log.html', 'w')
logfile.write('<html><body><h1>Diffeomorphism test log: ' + dfile + '</h1>')
logfile.write('Diffeomorphism file: ' + dfile + '<br/>')
logfile.write('Test image: <br/><img src="' + infile + '.png"/><br/><br/>')
logfile.write('<table>\n')
logfile.write('<tr align="center">\n <td>Command</td>\n')
logfile.write('<td>Diffeomorphism<br/>Angle</td>')
logfile.write('<td>Diffeomorphism<br/>Norm</td>')
logfile.write('<td>Diffeomorphism<br/>Variance</td>')
for i in range(levels + 1):
    logfile.write('<td>' + str(i) + '</td>')
logfile.write('</tr>')

im = np.array(Image.open(infile).resize(size).getdata(), np.uint8).reshape((size[1], size[0], 3))
# Y0,Y1 = get_Y_pair((30,30),(0,0),(160,120),im)
#    pdb.set_trace()
diffeo_list = pickle.load(open(dfile, 'rb')).actions

pdb.set_trace()
for i in range(len(diffeo_list)):
    D = diffeo_list[i].diffeo
    pdb.set_trace()
    logfile.write('<tr>')
    
    cmdstr = str(diffeo_list[i].original_cmd).replace(' ', '')
    
#        outpath+prefix+'diffeo'+cmdstr+'angle'+'.png'
    
    logfile.write('<td>')
    logfile.write(cmdstr)
    logfile.write('</td>')
    Image.fromarray(diffeo_to_rgb_angle(D.d)).save(outpath + prefix + 'diffeo' + cmdstr + 'angle' + '.png')
    logfile.write('<td>')
    logfile.write('<img src="' + outpath + prefix + 'diffeo' + cmdstr + 'angle' + '.png' + '"/>')
    logfile.write('</td>')
    Image.fromarray(diffeo_to_rgb_norm(D.d)).save(outpath + prefix + 'diffeo' + cmdstr + 'norm' + '.png')
    logfile.write('<td>')
    logfile.write('<img src="' + outpath + prefix + 'diffeo' + cmdstr + 'norm' + '.png' + '"/>')
    logfile.write('</td>')
    Image.fromarray((D.variance * 255).astype(np.uint8)).save(outpath + prefix + 'diffeo' + cmdstr + 'variance' + '.png')
    logfile.write('<td>')
    logfile.write('<img src="' + outpath + prefix + 'diffeo' + cmdstr + 'variance' + '.png' + '"/>')
    logfile.write('</td>')
    
    
    Y = im
    Image.fromarray(Y).save(outpath + prefix + cmdstr + str(0) + '.png')
    logfile.write('<td>')
    logfile.write('<img src="' + outpath + prefix + cmdstr + str(0) + '.png' + '"/>')
    logfile.write('</td>')
    for i in range(levels):
        Y, _ = D.apply(Y)
        Image.fromarray(Y).save(outpath + prefix + cmdstr + str(i + 1) + '.png')
        logfile.write('<td>')
        logfile.write('<img src="' + outpath + prefix + cmdstr + str(i + 1) + '.png' + '"/>')
        logfile.write('</td>')
    logfile.write('</tr>')
logfile.write('</table></body></html>')
logfile.flush()
logfile.close()


