#!/bin/bash

DATASET_NAME="JORD"
SEQUENCES=("01" "02" "03" "04" "05")

# 设置配置文件相关路径
CONFIG_PATH="/home/jlurobot/catkin_ws/src/CTD/config/config_jord.yaml"


# 遍历所有序列号
for SEQ in "${SEQUENCES[@]}"; do
    # 修改配置文件参数
    sed -i "s/dataset_sequence:.*/dataset_sequence: $SEQ/" $CONFIG_PATH

    # 使用grep查找并用awk提取dataset_sequence的值
    DATASET_SEQ=$(grep 'dataset_sequence:' $CONFIG_PATH | awk '{print $2}')
    echo "dataset_sequence: $DATASET_SEQ"
    # 启动算法
    # ./build/hsc_new

done







# # 定义序列号和Descriptor参数组合
# SEQUENCES=("KITTI00" "KITTI02" "KITTI05" "KITTI08" "JLU01" "JLU02" "JLU03" "JLU04")
# # TimeSeries=("false" "true") # TimeSeries=("false")
# LEVELNUM=(2 3 4 5 6 7 8 9 10)
# # EDGENUM=(0.90 0.91 0.92 0.93 0.94 0.95 0.96 0.97 0.98 0.99)


# # 设置算法评估相关路径
# ALG_base_path="/home/jlurobot/Loop_final/result"
# # 设置配置文件相关路径
# Config_path="/home/jlurobot/Loop_final/config/parameter.yaml"
# # 设置临时文件
# temp="/home/jlurobot/Loop_final/result/temp.txt"
# # 设置绘制PR曲线文件路径
# PR="/home/jlurobot/Loop_final/tool/batch_PRCurve.py"
# # 设置结果文件路径
# RESULT_FILE="/home/jlurobot/Loop_final/result/level_result.txt"

# # 遍历所有序列号
# for SEQ in "${SEQUENCES[@]}"; do
#     # # 遍历搜索参数组合
#     # for TIME in "${TimeSeries[@]}"; do
#         # 遍历层数
#         for LEVEL in "${LEVELNUM[@]}"; do
#     #         # 边界阈值选择
#     #         for EDGE in "${EDGENUM[@]}"; do
#                 # 修改配置文件参数
#                 sed -i "s/Sequence:.*/Sequence: $SEQ/" $Config_path
#                 # sed -i "s/Use_TimeSeries:.*/Use_TimeSeries: $TIME/" $Config_path
#                 sed -i "s/LEVEL:.*/LEVEL: $LEVEL/" $Config_path
#                 # sed -i "s/Edge_th:.*/Edge_th: $EDGE/" $Config_path

#                 # 启动算法
#                 ./build/hsc_new

#                 # 真值及算法结果路径
#                 GT_path=$ALG_base_path/$SEQ/lcdgt_$SEQ.txt
#                 alg_result_path=$ALG_base_path/$SEQ/result_$SEQ.txt # 需要注意是否正确

#                 # 执行评估程序
#                 python $PR $GT_path $alg_result_path > $temp

#                 # 读取评估结果
#                 F1_SCORE=$(sed -n '1p' $temp)
#                 EP_VALUE=$(sed -n '2p' $temp)

#                 xiaorong="Time"
#                 # if [ "$TIME" = "true" ];then
#                 #     xiaorong="Time"
#                 # fi


#                 # 将结果写入结果文件
#                 echo -e "$SEQ\t$xiaorong\t$LEVEL\t$F1_SCORE\t$EP_VALUE" >> "$RESULT_FILE"
#                 # 将结果写入结果文件
#                 # echo -e "$SEQ\t$xiaorong\t$F1_SCORE\t$EP_VALUE" >> "$RESULT_FILE"
#                 # 将结果写入结果文件
#                 # echo -e "$SEQ\t$xiaorong\t$EDGE\t$F1_SCORE\t$EP_VALUE" >> "$RESULT_FILE"
#     #         done
#         done
#     # done
# done

# # 清理临时文件
# # rm $temp