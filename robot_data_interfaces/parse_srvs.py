#!/usr/bin/env python3
import os
import sys

def parse_fields(lines):
    """解析一个服务部分（请求或响应）的行列表，返回字段列表 [(类型, 变量名), ...]"""
    fields = []
    for line in lines:
        # 去掉注释
        line = line.split('#')[0]
        line = line.strip()
        if not line:
            continue
        # 跳过常量定义（包含 '='）
        if '=' in line:
            continue
        # 分割成 [类型, 变量名, ...]
        parts = line.split()
        if len(parts) >= 2:
            field_type = parts[0]
            field_name = parts[1]
            fields.append((field_type, field_name))
    return fields

def parse_srv_file(filepath):
    """解析 .srv 文件，返回 (请求字段列表, 响应字段列表)"""
    with open(filepath, 'r', encoding='utf-8') as f:
        content = f.read()
    # 按 '---' 分割，注意可能前后有空格
    parts = content.split('---')
    if len(parts) < 2:
        print(f"警告：{filepath} 缺少 '---' 分隔符", file=sys.stderr)
        return [], []
    req_lines = parts[0].splitlines()
    res_lines = parts[1].splitlines()
    req_fields = parse_fields(req_lines)
    res_fields = parse_fields(res_lines)
    return req_fields, res_fields

def main():
    dir_path = sys.argv[1] if len(sys.argv) > 1 else '.'
    if not os.path.isdir(dir_path):
        print(f"错误：'{dir_path}' 不是一个有效目录", file=sys.stderr)
        sys.exit(1)

    srv_files = sorted([f for f in os.listdir(dir_path) if f.endswith('.srv')])
    output_lines = []

    for filename in srv_files:
        filepath = os.path.join(dir_path, filename)
        req_fields, res_fields = parse_srv_file(filepath)

        # 构造请求字段字符串
        req_strs = [f"{ftype} {fname}" for ftype, fname in req_fields]
        req_line = '，'.join(req_strs)

        # 构造响应字段字符串
        res_strs = [f"{ftype} {fname}" for ftype, fname in res_fields]
        res_line = '，'.join(res_strs)

        # 生成输出块
        block = f"{filename} : \nrequest   - {req_line}\nresponse  - {res_line}\n"
        output_lines.append(block)
        print(block)  # 同时打印到终端

    # 保存到文件
    out_file = os.path.join(os.getcwd(), 'parsed_srvs.txt')
    with open(out_file, 'w', encoding='utf-8') as f:
        f.write('\n'.join(output_lines))
    print(f"\n✅ 结果已保存到：{out_file}")

if __name__ == "__main__":
    main()
