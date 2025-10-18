#!/bin/bash
#
# 多相机数据集合并脚本
# 
# 用于合并多个由multi_camera_collector生成的数据集
#

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 打印带颜色的消息
print_info() {
    echo -e "${BLUE}ℹ${NC} $1"
}

print_success() {
    echo -e "${GREEN}✓${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}⚠${NC} $1"
}

print_error() {
    echo -e "${RED}✗${NC} $1"
}

# 显示使用帮助
show_help() {
    cat << EOF
多相机数据集合并脚本

用法:
    $0 [选项] --output <输出路径> <数据集1> <数据集2> [<数据集3> ...]

选项:
    -o, --output PATH       合并后数据集的输出路径 (必需)
    -f, --format FORMAT     CSV文件输出格式: combined, separate, both (默认: combined)
    -c, --conflicts METHOD  时间戳冲突解决策略: offset, skip, overwrite (默认: offset)
    -v, --verbose           显示详细输出信息
    -h, --help              显示此帮助信息

示例:
    # 合并两个数据集
    $0 --output merged_dataset dataset1/ dataset2/
    
    # 合并多个数据集并生成单独的CSV文件
    $0 --output merged_dataset --format separate dataset1/ dataset2/ dataset3/
    
    # 使用覆盖策略解决冲突
    $0 --output merged_dataset --conflicts overwrite dataset1/ dataset2/

说明:
    此脚本会将多个数据集合并为一个新数据集，并生成统一的CSV索引文件。
    生成的CSV文件格式与make_dataset.py生成的完全相同。
    
    冲突解决策略:
    - offset: 为后续数据集的时间戳添加偏移量（推荐）
    - skip: 跳过重复的时间戳
    - overwrite: 用后续数据集覆盖重复的数据
EOF
}

# 检查ROS2环境
check_ros2_environment() {
    if [ -z "$ROS_DISTRO" ]; then
        print_warning "ROS2环境未加载，尝试加载..."
        
        # 尝试常见的ROS2安装路径
        if [ -f "/opt/ros/humble/setup.bash" ]; then
            source /opt/ros/humble/setup.bash
            print_success "已加载ROS2 Humble环境"
        elif [ -f "/opt/ros/foxy/setup.bash" ]; then
            source /opt/ros/foxy/setup.bash
            print_success "已加载ROS2 Foxy环境"
        elif [ -f "/opt/ros/galactic/setup.bash" ]; then
            source /opt/ros/galactic/setup.bash
            print_success "已加载ROS2 Galactic环境"
        else
            print_warning "未找到ROS2安装，将尝试直接运行Python脚本"
        fi
    else
        print_success "ROS2环境已加载: $ROS_DISTRO"
    fi
}

# 检查工作空间
check_workspace() {
    # 尝试加载本地工作空间
    if [ -f "$SCRIPT_DIR/../../install/setup.bash" ]; then
        source "$SCRIPT_DIR/../../install/setup.bash"
        print_success "已加载工作空间环境"
    elif [ -f "$SCRIPT_DIR/../../../install/setup.bash" ]; then
        source "$SCRIPT_DIR/../../../install/setup.bash"
        print_success "已加载工作空间环境"
    fi
}

# 主函数
main() {
    echo "🔀 多相机数据集合并工具"
    echo "=================================="
    echo
    
    # 解析命令行参数
    OUTPUT_PATH=""
    FORMAT="combined"
    CONFLICTS="offset"
    VERBOSE=""
    DATASETS=()
    
    while [[ $# -gt 0 ]]; do
        case $1 in
            -o|--output)
                OUTPUT_PATH="$2"
                shift 2
                ;;
            -f|--format)
                FORMAT="$2"
                shift 2
                ;;
            -c|--conflicts)
                CONFLICTS="$2"
                shift 2
                ;;
            -v|--verbose)
                VERBOSE="-v"
                shift
                ;;
            -h|--help)
                show_help
                exit 0
                ;;
            *)
                DATASETS+=("$1")
                shift
                ;;
        esac
    done
    
    # 验证参数
    if [ -z "$OUTPUT_PATH" ]; then
        print_error "错误: 必须指定输出路径 (--output)"
        echo
        show_help
        exit 1
    fi
    
    if [ ${#DATASETS[@]} -lt 2 ]; then
        print_error "错误: 至少需要2个数据集进行合并"
        echo
        show_help
        exit 1
    fi
    
    # 检查环境
    check_ros2_environment
    check_workspace
    
    echo
    print_info "输出路径: $OUTPUT_PATH"
    print_info "CSV格式: $FORMAT"
    print_info "冲突策略: $CONFLICTS"
    print_info "数据集数量: ${#DATASETS[@]}"
    echo
    
    # 构建Python命令
    PYTHON_SCRIPT="$SCRIPT_DIR/multi_camera_collector/dataset_merger.py"
    
    # 检查Python脚本是否存在
    if [ ! -f "$PYTHON_SCRIPT" ]; then
        print_error "错误: 找不到dataset_merger.py"
        print_error "路径: $PYTHON_SCRIPT"
        exit 1
    fi
    
    # 运行Python脚本
    python3 "$PYTHON_SCRIPT" \
        --output "$OUTPUT_PATH" \
        --format "$FORMAT" \
        --conflicts "$CONFLICTS" \
        $VERBOSE \
        "${DATASETS[@]}"
    
    EXIT_CODE=$?
    
    echo
    if [ $EXIT_CODE -eq 0 ]; then
        print_success "数据集合并完成！"
    else
        print_error "数据集合并失败 (退出码: $EXIT_CODE)"
    fi
    
    exit $EXIT_CODE
}

# 运行主函数
main "$@"
