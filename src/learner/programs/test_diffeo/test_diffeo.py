from PIL import Image #@UnresolvedImport
from boot_agents.diffeo.diffeo_display import (diffeo_to_rgb_angle,
    diffeo_to_rgb_norm)
import numpy as np
import pickle

def test_diffeo_main(argv):
    # Find diffeomorphisms file
    try:
        dfile = argv[argv.index('-dl') + 1]
    except ValueError:
        dfile = '/media/data/learned_diffeo/camera_ptz.dynamics.pickle'
    print 'Using diffeomorphism from file:    ', dfile
    
    # Find diffeomorphisms file
    try:
        outpath = argv[argv.index('-o') + 1]
    except ValueError:
        outpath = '/media/data/tempimages/'
    print 'Saving images to path:            ', outpath
    
    # Find output prefix file
    try:
        prefix = argv[argv.index('-p') + 1]
    except ValueError:
        prefix = 'logitech_cam_ptz'
    print 'Using prefix for output files:    ', prefix
    
    # Find Input image
    try:
        infile = argv[argv.index('-im') + 1]
    except ValueError:
        infile = '/home/adam/git/surf12adam/diffeo_experiments/lighttower640.jpg'
    print 'Using image file:                ', infile
    
    # Number of levels to apply
    try:
        levels = int(argv[argv.index('-l') + 1])
    except ValueError:
        levels = 5
    print 'Applying diffeomorphism ', levels, ' times'

    # Image size
    try:
        size = eval(argv[argv.index('-size') + 1])
    except ValueError:
        size = [160, 120]
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
    #Y0,Y1 = get_Y_pair((30,30),(0,0),(160,120),im)
#    pdb.set_trace()
    diffeo_list = pickle.load(open(dfile, 'rb'))
#    pdb.set_trace()
    for i in range(len(diffeo_list.actions)):
#    for D in diffeo_list:
        D = diffeo_list.actions[i].diffeo
#        pdb.set_trace()
        logfile.write('<tr>')
        cmdstr = str(diffeo_list.actions[i].original_cmd).replace(' ', '_')
#        cmdstr = str(D.command).replace(' ','')
        
#        outpath+prefix+'diffeo'+cmdstr+'angle'+'.png'
        
        logfile.write('<td>')
        logfile.write(str(cmdstr))
        logfile.write('</td>')
        pim = Image.fromarray(diffeo_to_rgb_angle(D.d)).
        pim.save(outpath + prefix + 'diffeo' + cmdstr + 'angle' + '.png')
        logfile.write('<td>')
        logfile.write('<img src="' + outpath + prefix + 'diffeo' + cmdstr + 'angle' + '.png' + '"/>')
        logfile.write('</td>')
        pim = Image.fromarray(diffeo_to_rgb_norm(D.d))
        pim.save(outpath + prefix + 'diffeo' + cmdstr + 'norm' + '.png')
        logfile.write('<td>')
        logfile.write('<img src="' + outpath + prefix + 'diffeo' + cmdstr + 'norm' + '.png' + '"/>')
        logfile.write('</td>')
        pim = Image.fromarray((D.variance * 255).astype(np.uint8))
        pim.save(outpath + prefix + 'diffeo' + cmdstr + 'variance' + '.png')
        logfile.write('<td>')
        logfile.write('<img src="' + outpath + prefix + 'diffeo' + cmdstr + 'variance' + '.png' + '"/>')
        logfile.write('</td>')
        
        Y = im
        Image.fromarray(Y).save(outpath + prefix + cmdstr + str(0) + '.png')
        logfile.write('<td>')
        logfile.write('<img src="' + outpath + prefix + cmdstr + str(0) + '.png' + '"/>')
        logfile.write('</td>')
        for i in range(levels):
            Y, var = D.apply(Y)
            Image.fromarray(Y).save(outpath + prefix + cmdstr + str(i + 1) + '.png')
            logfile.write('<td>')
            logfile.write('<img src="' + outpath + prefix + cmdstr + str(i + 1) + '.png' + '"/>')
            logfile.write('</td>')
        logfile.write('</tr>')
    logfile.write('</table></body></html>')
    logfile.flush()
    logfile.close()



if __name__ == '__main__':
    test_diffeo_main()
