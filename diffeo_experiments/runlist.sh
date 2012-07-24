#!/bin/bash
delta='4,0'
size='8'
python learn_demo.py -d $delta -n 10 -s $size -i random
python learn_demo.py -d $delta -n 10 -s $size -i balboa
python learn_demo.py -d $delta -n 25 -s $size -i balboa
python learn_demo.py -d $delta -n 100 -s $size -i balboa
delta='3,3'
size='8'
python learn_demo.py -d $delta -n 10 -s $size -i random
python learn_demo.py -d $delta -n 10 -s $size -i balboa
python learn_demo.py -d $delta -n 25 -s $size -i balboa
python learn_demo.py -d $delta -n 100 -s $size -i balboa
delta='0,4'
size='8'
python learn_demo.py -d $delta -n 10 -s $size -i random
python learn_demo.py -d $delta -n 10 -s $size -i balboa
python learn_demo.py -d $delta -n 25 -s $size -i balboa
python learn_demo.py -d $delta -n 100 -s $size -i balboa
delta='-3,3'
size='8'
python learn_demo.py -d $delta -n 10 -s $size -i random
python learn_demo.py -d $delta -n 10 -s $size -i balboa
python learn_demo.py -d $delta -n 25 -s $size -i balboa
python learn_demo.py -d $delta -n 100 -s $size -i balboa
delta='-4,0'
size='8'
python learn_demo.py -d $delta -n 10 -s $size -i random
python learn_demo.py -d $delta -n 10 -s $size -i balboa
python learn_demo.py -d $delta -n 25 -s $size -i balboa
python learn_demo.py -d $delta -n 100 -s $size -i balboa
delta='-3,-3'
size='8'
python learn_demo.py -d $delta -n 10 -s $size -i random
python learn_demo.py -d $delta -n 10 -s $size -i balboa
python learn_demo.py -d $delta -n 25 -s $size -i balboa
python learn_demo.py -d $delta -n 100 -s $size -i balboa
delta='0,-4'
size='8'
python learn_demo.py -d $delta -n 10 -s $size -i random
python learn_demo.py -d $delta -n 10 -s $size -i balboa
python learn_demo.py -d $delta -n 25 -s $size -i balboa
python learn_demo.py -d $delta -n 100 -s $size -i balboa
delta='3,-3'
size='8'
python learn_demo.py -d $delta -n 10 -s $size -i random
python learn_demo.py -d $delta -n 10 -s $size -i balboa
python learn_demo.py -d $delta -n 25 -s $size -i balboa
python learn_demo.py -d $delta -n 100 -s $size -i balboa
