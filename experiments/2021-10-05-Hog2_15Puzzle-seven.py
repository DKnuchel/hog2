#! /usr/bin/env python

"""
Example experiment using a simple vertex cover solver.
"""

import glob
import os
import platform

from downward.reports.absolute import AbsoluteReport
from downward.reports.scatter import ScatterPlotReport
from lab.environments import BaselSlurmEnvironment, LocalEnvironment
from lab.experiment import Experiment
from lab.reports import Attribute, Report, geometric_mean, arithmetic_mean

NODE = platform.node()
REMOTE = NODE.endswith(".scicore.unibas.ch") or NODE.endswith(".cluster.bc2.ch")
TIME_LIMIT = 1800 
MEMORY_LIMIT = 6354

if REMOTE:
    ENV = BaselSlurmEnvironment(partition="infai_2", memory_per_cpu="6354M", email="damian.knuchel@unibas.ch",setup='export PATH=/scicore/soft/apps/binutils/2.32-GCCcore-8.3.0/bin:/scicore/soft/apps/CMake/3.15.3-GCCcore-8.3.0/bin:/scicore/soft/apps/cURL/7.66.0-GCCcore-8.3.0/bin:/scicore/soft/apps/bzip2/1.0.8-GCCcore-8.3.0/bin:/scicore/soft/apps/ncurses/6.1-GCCcore-8.3.0/bin:/scicore/soft/apps/GCCcore/8.3.0/bin:/export/soft/lua_lmod/centos7/lmod/lmod/libexec:/usr/local/bin:/usr/bin:/usr/local/sbin:/usr/sbin:/infai/nokyri32/bin:/scicore/soft/apps/binutils/2.32-GCCcore-8.3.0/lib:/scicore/soft/apps/cURL/7.66.0-GCCcore-8.3.0/lib:/scicore/soft/apps/bzip2/1.0.8-GCCcore-8.3.0/lib:/scicore/soft/apps/zlib/1.2.11-GCCcore-8.3.0/lib:/scicore/soft/apps/ncurses/6.1-GCCcore-8.3.0/lib:/scicore/soft/apps/GCCcore/8.3.0/lib64:/scicore/soft/apps/GCCcore/8.3.0/lib')
    SUITE = range(1000)
else:
    ENV = LocalEnvironment(processes=2)
    # Use smaller suite for local tests.
    SUITE = ['0', '1', '2']#, 'korf30', 'korf40']


# Create a new experiment.
exp = Experiment(environment=ENV)
# Add solver to experiment and make it available to all runs.
exp.add_resource("hog2", "../expBuild/Hog2", symlink=True) # revision hash als doc; # potential als symlink
exp.add_resource("pdb", "../expBuild/pdbs", symlink=True)
exp.add_resource("input", "../expBuild/15puzzles", symlink=True)


ALGORITHMS = {
   #"md": ["-m", "MD"] ,
   #"lc": ["-m", "LC"] ,
   "static 7-8": ["-m", "PDB", "--static", "-p", "((1,2,3,4,5,6,7)(8,9,10,11,12,13,14,15))"] ,
   #"static 5-5-5": ["-m", "PDB", "--static", "-p", "((1,2,4,5,8)(3,6,7,10,11)(9,12,13,14,15))"] ,
   #"static 6-6-3": ["-m", "PDB", "--static", "-p", "((1,2,4,5,8,9)(3,6,7,10,11,15)(12,13,14))"] ,


   #"pho-primal": ["-m", "PDB", "--pho", "-p", "4"] ,
   "pho-dual": ["-m", "PDB", "--pho", "-p", "4", "--dual", "--seven"] ,
   #"wvc-primal": ["-m", "PDB", "--pho", "-p", "4", "--integer"] ,
   "wvc-dual": ["-m", "PDB", "--pho", "-p", "4", "--integer", "--dual", "--seven"] ,
}

for algo_name, algo_params in ALGORITHMS.items():
    for puzzle_instance in SUITE:
        puzzle_instance = str(puzzle_instance)
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

matplotlib_options = {
    "font.family": "serif",
    "font.weight": "normal",
    # Used if more specific sizes not set.
    "font.size": 20,
    "axes.labelsize": 20,
    "axes.titlesize": 30,
    "legend.fontsize": 22,
    "xtick.labelsize": 10,
    "ytick.labelsize": 10,
    "lines.markersize": 10,
    "lines.markeredgewidth": 0.25,
    "lines.linewidth": 1,
    # Width and height in inches.
    "figure.figsize": [8, 8],
    "savefig.dpi": 100,
}
report_initial_phod_static = ScatterPlotReport(
        attributes=["initial"],
        filter_algorithm=["pho-dual", "static 7-8"],
        matplotlib_options=matplotlib_options,
        format="tex",
        scale="log",
    )

report_expanded_phod_static = ScatterPlotReport(
        attributes=["expansions"],
        filter_algorithm=["pho-dual", "static 7-8"],
        matplotlib_options=matplotlib_options,
        format="tex",
        scale="log",
    )

report_initial_phod_dynd = ScatterPlotReport(
        attributes=["initial"],
        filter_algorithm=["pho-dual", "wvc-dual"],
        matplotlib_options=matplotlib_options,
        format="tex",
        scale="log",
    )

report_expanded_phod_dynd = ScatterPlotReport(
        attributes=["expansions"],
        filter_algorithm=["pho-dual", "wvc-dual"],
        matplotlib_options=matplotlib_options,
        format="tex",
        scale="log",
    )


ATTRIBUTES = [
    Attribute("initial", min_wins=False),
    Attribute("length", min_wins=True),
    Attribute("expansions", min_wins=True, function=geometric_mean),
    Attribute("generated", min_wins=True, function=geometric_mean),
    Attribute("time", min_wins=True, function=arithmetic_mean),
    Attribute("coverage", min_wins=False, absolute=True, scale='linear'),
]
# Make a report.
exp.add_report(BaseReport(attributes=ATTRIBUTES), outfile="report.html")
exp.add_report(report_initial_phod_static, name="report_initial_phod_static")
exp.add_report(report_expanded_phod_static, name="report_expanded_phod_static")
exp.add_report(report_initial_phod_dynd, name="report_initial_phod_dynd")
exp.add_report(report_expanded_phod_dynd, name="report_expanded_phod_dynd")

# Parse the commandline and run the given steps.
exp.run_steps()
