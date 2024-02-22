import os
import sys
def rename_files_in_dir(directory):
    # 获取文件夹中的文件名
    filenames = os.listdir(directory)

    # 按照字典序对文件名进行排序
    filenames.sort()
    print(filenames[:10])
    # 遍历文件名
    for i, filename in enumerate(filenames):
        # 创建新的文件名，格式为六位数字
        new_filename = f"{i:06}.pcd"

        # 创建旧文件和新文件的完整路径
        old_file = os.path.join(directory, filename)
        new_file = os.path.join(directory, new_filename)

        # 重命名文件
        os.rename(old_file, new_file)

# 使用上述函数重命名指定文件夹中的文件
rename_files_in_dir(os.path.join(sys.argv[1], "pcd"))