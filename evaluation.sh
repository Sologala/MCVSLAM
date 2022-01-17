#! /bin/bash


echo "------------------------------"
RED='\E[1;31m'       # 红
GREEN='\E[1;32m'    # 绿
YELOW='\E[1;33m'    # 黄
BLUE='\E[1;34m'     # 蓝
PINK='\E[1;35m'     # 粉红
RES='\E[0m'          # 清除颜
#echo -e "input : ${YELOW}[file Name]${RED}"
#read filename
echo -e "input : ${YELOW}[comment]${RES}"
read comment
echo ${comment}

# python3 calTractaryDistance.py /home/wen/SLAM/Datasets/data/trajectory.txt


# if [ "${comment}" == '' ];then
# 	evo_rpe tum ./trajectory.txt ./GroundTruth.txt  -a;
# else
# 	evo_rpe tum ./trajectory.txt  ./GroundTruth.txt  -as;
# fi

echo "--------------------------------------------------------"
# python3 ../evaluation/evaluate_ate_scale.py /home/wen/SLAM/Datasets/data/trajectory.txt ./trajectory.txt --plot outputPdf.pdf
evo_rpe tum ./app/trajectory.txt  /home/wen/SLAM/Datasets/data/trajectory.txt  -a 
evo_ape tum ./app/trajectory.txt  /home/wen/SLAM/Datasets/data/trajectory.txt  -a 
echo "--------------------------------------------------------"


# evo_rpe tum  /home/wen/SLAM/Datasets/data/trajectory.txt ./trajectory.txt  -a

# if [ "${comment}" == '' ];then
# 	evo_rpe tum ./trajectory.txt /home/wen/SLAM/Datasets/data/trajectory.txt -p  --plot_mode=xy;
# else
# 	evo_ape tum ./trajectory.txt  /home/wen/SLAM/Datasets/data/trajectory.txt -p  --plot_mode=xy --save_plot ./Expriments/${comment}.jpg --save_results ./Expriments/${comment}.tar
# fi

# evo_rpe tum ./trajectory.txt ./backward.txt ../data/trajectory.txt -p -a --plot_mode=xy --save_plot ./Expriments/${comment}.jpg --save_results ./Expriments/${comment}.tar
