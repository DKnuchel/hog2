#! /usr/bin/env python

"""
Example experiment using a simple vertex cover solver.
"""

import glob
import os
import platform

from downward.reports.absolute import AbsoluteReport
from lab.environments import BaselSlurmEnvironment, LocalEnvironment
from lab.experiment import Experiment
from lab.reports import Attribute, Report, geometric_mean

NODE = platform.node()
REMOTE = NODE.endswith(".scicore.unibas.ch") or NODE.endswith(".cluster.bc2.ch")
TIME_LIMIT = 1800 
MEMORY_LIMIT = 6354

if REMOTE:
    ENV = BaselSlurmEnvironment(partition="infai_2", memory_per_cpu="6354M", email="damian.knuchel@unibas.ch")
    SUITE = range(1000)
else:
    ENV = LocalEnvironment(processes=2)
    # Use smaller suite for local tests.
    SUITE = ['korf10', 'korf20', 'korf30', 'korf40']


# Create a new experiment.
exp = Experiment(environment=ENV)
# Add solver to experiment and make it available to all runs.
exp.add_resource("hog2", "../expBuild/Hog2", symlink=True) # revision hash als doc; # potential als symlink
exp.add_resource("pdb", "../expBuild/pdbs", symlink=True)
exp.add_resource("input", "../expBuild/15puzzles", symlink=True)


ALGORITHMS = {
   "md": ["-m", "MD"] ,
   "lc": ["-m", "LC"] ,
   "static 7-8": ["-m", "PDB", "--static", "-p", "((1,2,3,4,5,6,7)(8,9,10,11,12,13,14,15))"] ,
   "static 5-5-5": ["-m", "PDB", "--static", "-p", "((1,2,4,5,8)(3,6,7,10,11)(9,12,13,14,15))"] ,
   "static 6-6-3": ["-m", "PDB", "--static", "-p", "((1,2,4,5,8,9)(3,6,7,10,11,15)(12,13,14))"] ,


   "pho-primal": ["-m", "PDB", "--pho", "-p", "4"] ,
   "pho-dual": ["-m", "PDB", "--pho", "-p", "4", "--dual"] ,
   "wvc-primal": ["-m", "PDB", "--pho", "-p", "4", "--integer"] ,
   "wvc-dual": ["-m", "PDB", "--pho", "-p", "4", "--integer", "--dual"] ,
}

for algo_name, algo_params in ALGORITHMS.items():
    for puzzle_instance in SUITE:
        run = exp.add_run()
        # Create a symbolic link and an alias. This is optional. We
        # could also use absolute paths in add_command().
        # print(os.path.dirname(os.path.realpath(__file__)))
        run.add_command(
            "solve",
            ["{hog2}", "-i", "{input}/" + str(puzzle_instance) + ".pzl" , "-d", "{pdb}/", "--resolve", *algo_params],
            time_limit=TIME_LIMIT,
            memory_limit=MEMORY_LIMIT,
        )
        run.set_property("algorithm", algo_name)
        # Every run has to have a unique id in the form of a list.
        run.set_property("id", [algo_name, puzzle_instance])
        #run.set_property("args", [["-i", puzzle_instance + ".pzl" , "-d", "{pdb}"] + algo_params])


        # AbsoluteReport needs the following attributes:
        # 'domain', 'problem' and 'algorithm'.
        run.set_property("domain", "15puzzle")
        run.set_property("problem", puzzle_instance)
        # BaseReport needs the following properties:
        # 'time_limit', 'memory_limit', 'seed'.
        run.set_property("time_limit", TIME_LIMIT)
        run.set_property("memory_limit", MEMORY_LIMIT)
        run.set_property("seed", -1)

# Add custom parser.
exp.add_parser("parser.py")

# Add step that writes experiment files to disk.
exp.add_step("build", exp.build)

# Add step that executes all runs.
exp.add_step("start", exp.start_runs)

# Add step that collects properties from run directories and
# writes them to *-eval/properties.
exp.add_fetcher(name="fetch")

# Create custom report class with suitable info and error attributes.
class BaseReport(AbsoluteReport):
    INFO_ATTRIBUTES = ["time_limit", "memory_limit", "seed"]
    ERROR_ATTRIBUTES = [
        "domain",
        "problem",
        "algorithm",
        "unexplained_errors",
        "error",
        "node",
    ]



ATTRIBUTES = [
    Attribute("initial", min_wins=False),
    Attribute("length", min_wins=True),
    Attribute("expansions", min_wins=True, function=geometric_mean),
    Attribute("generated", min_wins=True, function=geometric_mean),
    Attribute("time", min_wins=True, function=geometric_mean),
    Attribute("coverage", min_wins=False, absolute=True, scale='linear'),
]
# Make a report.
exp.add_report(BaseReport(attributes=ATTRIBUTES), outfile="report.html")

# Parse the commandline and run the given steps.
exp.run_steps()
