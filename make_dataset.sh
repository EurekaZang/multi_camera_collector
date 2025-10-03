#!/bin/bash

# 多相机数据集一键制作脚本
# 此脚本提供了便捷的方式来生成数据集CSV文件

set -e

# 脚本信息
SCRIPT_NAME="多相机数据集一键制作工具"
VERSION="1.0.0"

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# 打印带颜色的消息
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[成功]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[警告]${NC} $1"
}

print_error() {
    echo -e "${RED}[错误]${NC} $1"
}

print_header() {
    echo -e "${PURPLE}🤖 ${SCRIPT_NAME} v${VERSION}${NC}"
    echo -e "${PURPLE}=====================================${NC}"
}

# 显示帮助信息
show_help() {
    cat << EOF
用法: $0 [选项] <数据集路径>

选项:
  -f, --format FORMAT   输出格式 (combined|separate|both)，默认: combined
  -v, --verbose        显示详细输出
  -h, --help           显示此帮助信息
  --version           显示版本信息

数据集路径:
  指向包含camera和camera_femto子目录的数据集根目录

输出格式说明:
  combined   - 生成单个CSV文件包含所有相机数据（推荐用于深度学习）
  separate   - 为每个相机生成单独的CSV文件
  both       - 同时生成组合和单独的CSV文件

示例:
  $0 /path/to/dataset                          # 使用默认设置
  $0 /path/to/dataset -f both                  # 生成所有类型的CSV文件
  $0 /path/to/dataset --verbose                # 显示详细信息
  $0 ./dataset -f separate -v                  # 组合使用选项

数据集目录结构应为:
  dataset/
  ├── camera/
  │   ├── rgb/           # 标准相机RGB图像
  │   ├── depth/         # 标准相机深度图像
  │   └── camera_info/   # 标准相机参数文件
  └── camera_femto/
      ├── rgb/           # Femto相机RGB图像
      ├── depth/         # Femto相机深度图像
      └── camera_info/   # Femto相机参数文件

生成的文件:
  • dataset.csv - 主数据索引文件（推荐用于深度学习）
  • camera_dataset.csv - 标准相机数据索引（如果选择separate或both）
  • camera_femto_dataset.csv - Femto相机数据索引（如果选择separate或both）
  • dataset_statistics.json - 数据集统计信息

EOF
}

# 检查依赖
check_dependencies() {
    # 检查Python
    if ! command -v python3 &> /dev/null; then
        print_error "Python3 未安装或不在PATH中"
        exit 1
    fi
    
    # 检查ROS2环境
    if [ -z "$ROS_DISTRO" ]; then
        print_warning "ROS2环境未设置，尝试source setup文件..."
        if [ -f "install/setup.bash" ]; then
            source install/setup.bash
            print_info "已加载本地ROS2环境"
        elif [ -f "/opt/ros/*/setup.bash" ]; then
            source /opt/ros/*/setup.bash
            print_info "已加载系统ROS2环境"
        else
            print_warning "未找到ROS2环境，将尝试直接运行Python脚本"
        fi
    fi
}

# 验证数据集路径
validate_dataset_path() {
    local dataset_path="$1"
    
    if [ -z "$dataset_path" ]; then
        print_error "请提供数据集路径"
        echo
        show_help
        exit 1
    fi
    
    if [ ! -d "$dataset_path" ]; then
        print_error "数据集路径不存在: $dataset_path"
        exit 1
    fi
    
    # 检查基本目录结构
    local missing_dirs=()
    
    if [ ! -d "$dataset_path/camera" ] && [ ! -d "$dataset_path/camera_femto" ]; then
        missing_dirs+=("camera 或 camera_femto")
    fi
    
    if [ ${#missing_dirs[@]} -gt 0 ]; then
        print_error "数据集目录结构不完整，缺少: ${missing_dirs[*]}"
        print_info "期望的目录结构:"
        print_info "  $dataset_path/"
        print_info "  ├── camera/"
        print_info "  └── camera_femto/"
        exit 1
    fi
}

# 主函数
main() {
    # 解析命令行参数
    FORMAT="combined"
    VERBOSE=""
    DATASET_PATH=""
    
    while [[ $# -gt 0 ]]; do
        case $1 in
            -f|--format)
                FORMAT="$2"
                if [[ ! "$FORMAT" =~ ^(combined|separate|both)$ ]]; then
                    print_error "无效的格式: $FORMAT"
                    echo "支持的格式: combined, separate, both"
                    exit 1
                fi
                shift 2
                ;;
            -v|--verbose)
                VERBOSE="--verbose"
                shift
                ;;
            -h|--help)
                show_help
                exit 0
                ;;
            --version)
                echo "$VERSION"
                exit 0
                ;;
            -*)
                print_error "未知选项: $1"
                show_help
                exit 1
                ;;
            *)
                if [ -z "$DATASET_PATH" ]; then
                    DATASET_PATH="$1"
                else
                    print_error "只能指定一个数据集路径"
                    exit 1
                fi
                shift
                ;;
        esac
    done
    
    # 显示头部信息
    print_header
    echo
    
    # 验证输入
    validate_dataset_path "$DATASET_PATH"
    
    # 检查依赖
    check_dependencies
    
    # 转换为绝对路径
    DATASET_PATH=$(realpath "$DATASET_PATH")
    
    print_info "数据集路径: $DATASET_PATH"
    print_info "输出格式: $FORMAT"
    echo
    
    # 尝试使用ROS2命令运行
    if command -v ros2 &> /dev/null && ros2 pkg list | grep -q "multi_camera_collector"; then
        print_info "使用ROS2包中的工具..."
        if ros2 run multi_camera_collector make_dataset "$DATASET_PATH" --format "$FORMAT" $VERBOSE; then
            print_success "数据集制作完成！"
        else
            print_error "使用ROS2工具失败，尝试直接运行Python脚本..."
            FALLBACK=true
        fi
    else
        print_info "ROS2包未安装，使用本地Python脚本..."
        FALLBACK=true
    fi
    
    # 如果ROS2方式失败，使用备用方案
    if [ "$FALLBACK" = true ]; then
        # 查找Python脚本
        SCRIPT_DIR=$(dirname "$0")
        PYTHON_SCRIPT=""
        
        # 搜索可能的脚本位置
        POSSIBLE_PATHS=(
            "$SCRIPT_DIR/../multi_camera_collector/make_dataset.py"
            "$SCRIPT_DIR/make_dataset.py"
            "./src/multi_camera_collector/multi_camera_collector/make_dataset.py"
            "./multi_camera_collector/make_dataset.py"
        )
        
        for path in "${POSSIBLE_PATHS[@]}"; do
            if [ -f "$path" ]; then
                PYTHON_SCRIPT="$path"
                break
            fi
        done
        
        if [ -z "$PYTHON_SCRIPT" ]; then
            print_error "找不到make_dataset.py脚本"
            print_info "请确保脚本在以下位置之一："
            for path in "${POSSIBLE_PATHS[@]}"; do
                print_info "  $path"
            done
            exit 1
        fi
        
        print_info "使用Python脚本: $PYTHON_SCRIPT"
        
        # 运行Python脚本
        if python3 "$PYTHON_SCRIPT" "$DATASET_PATH" --format "$FORMAT" $VERBOSE; then
            print_success "数据集制作完成！"
        else
            print_error "数据集制作失败"
            exit 1
        fi
    fi
    
    echo
    print_success "✨ 数据集CSV文件已生成！"
    print_info "💡 生成的CSV文件可直接用于深度学习训练"
    print_info "📊 查看 dataset_statistics.json 了解详细统计信息"
}

# 错误处理
trap 'print_error "脚本执行被中断"; exit 130' INT
trap 'print_error "脚本执行出错，退出码: $?"; exit 1' ERR

# 运行主函数
main "$@"