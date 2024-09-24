# !/bin/bash
# 绘制PR曲线的脚本，根据输入序列号绘制当前序列下的所有算法的PR曲线

# 函数：从参数文件中获取值
get_yaml_value() {
    python -c "
import yaml
data = yaml.safe_load(open('config/parameter.yaml'))
print(data['$1']['$2'])
"
}

# 获取参数值
sequence=$(get_yaml_value "seq" "sequence")
result_path=$(get_yaml_value "path" "result_path")

# 构建文件路径
base_path="${result_path}${sequence}/"
gt="${base_path}lcdgt${sequence}.txt"
ours="${base_path}loop_result_${sequence}.txt"

# 运行Python程序绘制PR曲线
SC="${base_path}Compare/SC_KITTI_${sequence}.txt"
ISC="${base_path}Compare/ISC_KITTI_${sequence}.txt"
M2DP="${base_path}Compare/M2DP_KITTI_${sequence}.txt"
CSSC="${base_path}Compare/CSSC_KITTI_${sequence}.txt"
Iris="${base_path}Compare/Iris_KITTI_${sequence}.txt" # 结果有点问题
NDD="${base_path}Compare/NDD_KITTI_${sequence}.txt"
python ./tool/PRCurve.py "$gt" "$ours" "$SC" "$ISC" "$M2DP" "$CSSC" "$Iris" "$NDD" #"$BoW3D"  