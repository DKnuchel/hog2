if [ $# -ge 2 ] 
then
 	if [ $# -eq 3 ]
	then
		#printf "\nArg 3: $3\n"
		if [ "$3" = "-c" ]
		then
			rm -rf $1;
		fi
	fi
#save current directory for later usage
cwd=$(pwd)
#generate directory, path given by args
mkdir $1
cd $1
#run cmake
cmake $cwd -D TMP_SIZE:STRING="$2" ..
#generate needed directories
mkdir 15puzzles
mkdir 24puzzles
mkdir pdbs

make
else
	printf "\nInvalid number of arguments: [PATH] [PUZZLE_SIZE]\n"
fi;