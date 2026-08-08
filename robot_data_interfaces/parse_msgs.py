#!/usr/bin/env python3
import os
import sys

def parse_msg_file(filepath):
    fields = []
    with open(filepath, 'r', encoding='utf-8') as f:
        for line in f:
            line = line.split('#')[0].strip()
            if not line:
                continue
            if '=' in line:
                continue
            parts = line.split()
            if len(parts) >= 2:
                fields.append((parts[0], parts[1]))
    return fields

def main():
    dir_path = sys.argv[1] if len(sys.argv) > 1 else '.'
    if not os.path.isdir(dir_path):
        print(f"错误：'{dir_path}' 不是一个有效目录", file=sys.stderr)
        sys.exit(1)

    # 列出所有 .msg 文件
    msg_files = sorted([f for f in os.listdir(dir_path) if f.endswith('.msg')])
    print(f"📁 在目录 '{dir_path}' 中找到 {len(msg_files)} 个 .msg 文件")
    if not msg_files:
        print("⚠️  没有找到任何 .msg 文件，请检查目录是否正确。")
        # 仍然会创建空文件，但提示用户
        out_file = os.path.join(os.getcwd(), 'parsed_msgs.txt')
        with open(out_file, 'w', encoding='utf-8') as f:
            f.write("")  # 空文件
        print(f"❌ 结果文件已创建但为空：{out_file}")
        return

    output_lines = []
    for filename in msg_files:
        filepath = os.path.join(dir_path, filename)
        fields = parse_msg_file(filepath)
        print(f"📄 {filename}: 解析到 {len(fields)} 个字段")
        if not fields:
            # 即使字段为空，也保留文件名输出
            output_lines.append(f"{filename} : (无字段)")
        else:
            field_strs = [f"{ftype} {fname}" for ftype, fname in fields]
            fields_line = '，'.join(field_strs)
            output_lines.append(f"{filename} : {fields_line}")

    # 写入文件
    out_file = os.path.join(os.getcwd(), 'parsed_msgs.txt')
    with open(out_file, 'w', encoding='utf-8') as f:
        f.write('\n'.join(output_lines))
    print(f"\n✅ 结果已保存到：{out_file}")
    print("📝 内容预览：")
    for line in output_lines[:5]:  # 只显示前5行
        print("  " + line)

if __name__ == "__main__":
    main()
