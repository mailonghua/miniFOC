# -*- coding:utf8 -*-
import datetime
import re
from SCons.Script import DefaultEnvironment

Import("env")
print("-----Run pre_extra_script.py-----")
#下面的函数用于在每次编译前更新版本号，其在platformio.ini文件中引用包含
def on_pre_build():
    print("-----Enter on_pre_build-----")
    # 获取当前日期和时间
    now = datetime.datetime.now()
    date_str = now.strftime("%Y%m%d%H%M")

    # 读取原始版本文件
    version_file_path = "src/version.h"
    with open(version_file_path, "r") as file:
        content = file.read()

    # 更新版本号
    new_version = f'"v{date_str}"'
    updated_content = re.sub(r'#define FIRMWARE_VERSION ".*"', f'#define FIRMWARE_VERSION {new_version}', content)

    # 写入新的版本号
    with open(version_file_path, "w") as file:
        file.write(updated_content)
#直接调用-修改.h文件对应的版本号
on_pre_build()
# 注册预处理动作，确保在构建前执行版本号更新
# env.AddPreAction("buildprog", on_pre_build)#编译之前。目前这样设置回调并没有触发
# env.AddPreAction("upload", on_pre_build)#上传之前