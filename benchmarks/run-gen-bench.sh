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
domains="AUV AUV-deep ServiceRobot"
num=6



for d in $domains
do 


for (( i=1;i<=$num;i++ ))
do
/usr/bin/time -p -o $curdir/results/$d/res_relax-gen$i-bfs.time ./robust_plans $curdir/$d/domain.pddl $curdir/$d/problem$i.pddl $curdir/$d/plan$i-gen-bfs.sol -g bfs > $curdir/results/$d/out_relax-gen$i-bfs.out
/usr/bin/time -p -o $curdir/results/$d/res_relax-gen$i-hmax.time ./robust_plans $curdir/$d/domain.pddl $curdir/$d/problem$i.pddl $curdir/$d/plan$i-gen-hmax.sol -g hmax > $curdir/results/$d/out_relax-gen$i-hmax.out
/usr/bin/time -p -o $curdir/results/$d/res_relax-gen$i-hadd.time ./robust_plans $curdir/$d/domain.pddl $curdir/$d/problem$i.pddl $curdir/$d/plan$i-gen-hadd.sol -g hadd > $curdir/results/$d/out_relax-gen$i-hadd.out
done



done

