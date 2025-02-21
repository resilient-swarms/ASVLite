#!/bin/sh
GTCT_DIR=${GTCT_DIR:-${HOME}/devel/parallel-peak-pruning/ContourTree/SweepAndMergeSerial/out}
RED=""
GREEN=""
NC=""
if [ -t 1 ]; then
# If stdout is a terminal, color Pass and FAIL green and red, respectively
    RED=$(tput setaf 1)
    GREEN=$(tput setaf 2)
    NC=$(tput sgr0)
fi

echo "Removing previously generated files"
rm *.log *.dat

echo "Copying target file "$1 "into current directory"
filename=${1##*/}
fileroot=${filename%.txt}
cp $1 ${filename}

echo "Splitting data into "$2" x "$2" parts"
./split_data_2d.py ${filename} $2
rm ${filename}

echo "Running HACT"
n_parts=$(($2*$2))
echo mpirun -np 4  ./ContourTree_Distributed -d Any --numBlocks=${n_parts} ${fileroot}_part_%d_of_${n_parts}.txt
mpirun -np 4  ./ContourTree_Distributed -d Any --preSplitFiles --saveTreeCompilerData --numBlocks=${n_parts} ${fileroot}_part_%d_of_${n_parts}.txt
rm ${fileroot}_part_*_of_${n_parts}.txt

echo "Compiling Outputs"
./TreeCompiler TreeCompilerOutput_*.dat | sort > outsort${fileroot}_$2x$2.txt

echo "Diffing"
diff outsort${fileroot}_$2x$2.txt ${GTCT_DIR}/outsort${fileroot}.txt
if test $? -eq 0; then echo "${GREEN}Pass${NC}"; rm outsort${fileroot}_$2x$2.txt; else echo "${RED}FAIL${NC}"; fi;

# echo "Generating Dot files"
# ./makedot.sh
