#!/bin/bash
# 功能：扫描指定文件夹下的所有 .srv，生成 create_service 调用代码
# 用法：./gen_service_creators.sh [目录路径]
#       若不指定目录，默认使用当前目录

set -euo pipefail

TARGET_DIR="${1:-.}"  # 第一个参数为目录，默认为当前目录

if [ ! -d "$TARGET_DIR" ]; then
    echo "错误：目录 '$TARGET_DIR' 不存在" >&2
    exit 1
fi

cd "$TARGET_DIR"

shopt -s nullglob
files=( *.srv )
if [ ${#files[@]} -eq 0 ]; then
    echo "当前目录没有 .srv 文件" >&2
    exit 0
fi

for f in "${files[@]}"; do
    base="${f%.srv}"                     # 原始下划线文件名（如 config_clear_enc_error）
    # 下划线转驼峰（首字母大写）
    camel=$(echo "$base" | perl -pe 's/_([a-z])/\u$1/g; s/^([a-z])/\u$1/')
    # 去掉第一个单词（如 Config, Runtime 等），得到剩余驼峰
    remaining=$(echo "$camel" | perl -pe 's/^[A-Z][a-z]*//')
    # 若去掉后为空，则保留完整驼峰
    if [ -z "$remaining" ]; then
        remaining="$camel"
    fi
    # 生成 create_service 调用（严格遵循示例格式）
    cat <<EOF
  service${remaining}_ = this->create_service<${camel}>(
      "${base}", std::bind(&BasicOrin::handle_${camel}_request, this,
                           std::placeholders::_1, std::placeholders::_2));
EOF
done
