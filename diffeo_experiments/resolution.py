import pylab




#ress = range(40,321,20)
#ress = [30, 40, 60, 80, 120, 160, 240, 320]
ress = [40, 80, 160, 320]

fracts = [0.10, .35, .5]
search_grids = [5, 9, 15]
for i, fract in enumerate(fracts): 
    for j, g in enumerate(search_grids):
        ref_factor = g / 3.

        maxr = 6
        pylab.subplot2grid((len(search_grids), len(fracts)), (j, i))
        pylab.hold(True)
        pylab.title('Grid = %s, fraction = %s' % (g, fract))
        for res in ress:
            resolutions = [g / (res * fract) * (ref_factor ** k) for k in range(maxr)]
            pylab.plot(range(1, maxr + 1), resolutions)
        pylab.ylim(0, 25)
        pylab.gca().set_yscale('log')
        pylab.xticks(range(1, maxr))
        pylab.grid()

pylab.show()
