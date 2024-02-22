import os
import json
import sys
folder_path = sys.argv[1] # 设置要查看的文件夹路径
out_path = sys.argv[2]

with open(folder_path, 'r') as infile, open(out_path, 'w') as outfile:
    for line in infile:
        # 使用空格分隔每行数据
        items = line.split()
        # 去掉第一个数据
        new_items = items[1:]
        # 使用空格将数据合并为一行，然后写入到输出文件
        new_line = ' '.join(new_items)
        outfile.write(new_line + '\n')