#!/bin/bash

#$1 - domain $2- testing $3 - planner script $4 dom suffix

ulimit -t 900
ulimit -v 4194304

curdir=`pwd`/benchmarks
#dirn=`dirname $0`
#dir="$curdir/$dirn"

#domains="AUV"
#domains="AUV-deep"
#domains="ServiceRobot"
domains="AUV-w HomeRobot AUV AUV-deep AUV-deep-bf ServiceRobot"
num=6



for d in $domains
do 


for (( i=1;i<=$num;i++ ))
do
/usr/bin/time -p -o $curdir/results/$d/res_waitfor$i.time ./waitfor $curdir/$d/domain.pddl $curdir/$d/problem$i.pddl > $curdir/results/$d/out_waitfor$i.out
/usr/bin/time -p -o $curdir/results/$d/res_gles$i.time ./robust_plans $curdir/$d/domain.pddl $curdir/$d/problem$i.pddl $curdir/$d/plan$i-gles.sol -l > $curdir/results/$d/out_gles$i.out
done



done

