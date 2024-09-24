#!/bin/bash
# 读入三个参数
# -build：1表示重新构建项目，0表示不重新构建项目
# -draw：1表示绘制所有算法PR曲线，0表示只绘制自己算法PR曲线
# -run: 1表示运行算法，0表示不运行算法，默认运行

rebuild=1
draw_all=0
run=1

# 处理命令行参数
while [[ $# -gt 0 ]]; do
    key="$1"
    case $key in
        -build | -b)
        rebuild="$2"
        shift
        shift
        ;;
        -run | -r)
        run="$2"
        shift
        shift
        ;;
        -draw | -d)
        draw_all="$2"
        shift
        shift
        ;;
        *)
        shift
        ;;
    esac
done


# 判断是否需要重新构建项目
if [ "$rebuild" -eq 1 ]; then
    echo "Building the project..."
    cd build
    cmake ..
    make

    # 检查构建的返回值
    build_status=$?
    
    cd ..
    
    if [ $build_status -ne 0 ]; then
        echo "Error: Project build failed."
        exit 1  # 退出脚本，不继续执行后续步骤
    fi
fi


# 判断是否运行程序
if [ "$run" -eq 1 ]; then
    # 运行C++程序执行算法
    ./build/hsc_new

    # 检查程序运行的返回值
    run_status=$?
        
    if [ $run_status -ne 0 ]; then
        echo "Error: Project run failed."
        exit 1  # 退出脚本，不继续执行后续步骤
    fi
fi


# 函数：从参数文件中获取值
get_yaml_value() {
    python -c "
import yaml
data = yaml.safe_load(open('config/parameter.yaml'))
print(data['$1']['$2'])
"
}

# 获取参数值
parameter=$(get_yaml_value "Seq" "Sequence")
result_path=$(get_yaml_value "Path" "Result_Path")
# 使用正则表达式来分离序列和序号
if [[ $parameter =~ ^([A-Za-z]+)([0-9]+)$ ]]; then
    dataset="${BASH_REMATCH[1]}"
    sequence="${BASH_REMATCH[2]}"
    echo "序列值为: $dataset"
    echo "序号值为: $sequence"
else
    echo "无法识别的参数格式: $parameter"
fi

# 构建文件路径
base_path="${result_path}${parameter}/"
gt="${base_path}lcdgt_${parameter}.txt"
ours="${base_path}result_${parameter}.txt"
SC_compare="${base_path}Compare/SC_${dataset}_${sequence}.txt"
# SC_corr_cmpare="${base_path}Compare/SC+corr_${dataset}_${sequence}.txt"
NDD_compare="${base_path}Compare/NDD_${dataset}_${sequence}.txt"


# 运行Python程序绘制PR曲线
if [ "$draw_all" -eq 1 ]; then
    SC="${base_path}Compare/SC_${dataset}_${sequence}.txt"
    ISC="${base_path}Compare/ISC_${dataset}_${sequence}.txt"
    SC_Intensity="${base_path}Compare/SC_Intensity_${dataset}_${sequence}.txt"

    M2DP="${base_path}Compare/M2DP_${dataset}_${sequence}.txt"
    CSSC="${base_path}Compare/CSSC_${dataset}_${sequence}.txt"

    Iris="${base_path}Compare/Iris_${dataset}_${sequence}.txt" # 结果有点问题
    NDD="${base_path}Compare/NDD_${dataset}_${sequence}.txt"
    python ./tool/PRCurve.py "$gt" "$ours" "$SC" "$ISC" "$SC_Intensity" "$M2DP" "$CSSC" "$NDD" "$Iris" 
else
    # python ./tool/batch_PRCurve.py "$gt" "$ours" "$SC_compare" "$NDD_compare"
    python ./tool/PRCurve.py "$gt" "$ours" # "$SC_compare" "$NDD_compare"
fi

# 运行Python程序绘制轨迹
# ...（保持原有的绘制轨迹部分不变）
