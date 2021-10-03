#! /usr/bin/env python

from lab.parser import Parser

parser = Parser()
parser.add_pattern('constraints', 'constraints: (\d+)', required=True, type=int)

def parse_lines(content, props):
    hog2_return_value_solution = None
    for line in content.splitlines():
        if "constraints" in line:
            hog2_return_value_solution = True

    if hog2_return_value_solution is None:
        hog2_return_value_solution = False

    props["hog2_return_value_solution"] = hog2_return_value_solution


parser.add_function(parse_lines)

parser.parse()
