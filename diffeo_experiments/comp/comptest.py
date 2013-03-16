from compmake import comp, draw



def func(x):
    return 2 * x


res1 = comp(func, 2)
res2 = comp(func, 4)
draw(res1)
