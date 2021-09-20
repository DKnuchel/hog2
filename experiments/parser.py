#! /usr/bin/env python

from lab.parser import Parser

parser = Parser()
parser.add_pattern('initial', 'InitialH: (\d+)', required=False, type=int)
parser.add_pattern('length', 'Length: (\d+)', required=False, type=int) # "adsfakj alsdjfasdf solution length: 23123 adslfkjasdkfjasd"
parser.add_pattern('expansions', 'Expanded: (\d+)', required=False, type=int)
parser.add_pattern('generated', 'Generated: (\d+)', required=False, type=int)
parser.add_pattern('time', 'Time: (.+)', required=False, type=float) # runtime: 123.123s

def parse_lines(content, props):
    hog2_return_value_solution = None
    for line in content.splitlines():
        if "ida" in line:
            hog2_return_value_solution = True

    if hog2_return_value_solution is None:
        hog2_return_value_solution = False

    props["hog2_return_value_solution"] = hog2_return_value_solution

parser.add_function(parse_lines)

parser.parse()
